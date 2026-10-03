package com.merlin.app

import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.Response
import okhttp3.WebSocket
import okhttp3.WebSocketListener
import okio.ByteString
import okio.ByteString.Companion.toByteString
import org.json.JSONArray
import org.json.JSONObject
import java.net.InetAddress
import java.net.Socket
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.concurrent.TimeUnit
import javax.net.SocketFactory

/**
 * Minimal Foxglove WebSocket client: publishes /cmd_vel (TwistStamped) and the LED command as JSON
 * through foxglove_bridge's clientPublish capability, and reads the battery and the robot's uptime.
 * Later topics (scan, map...) subscribe here.
 */
class FoxgloveClient(
    private val onState: (String) -> Unit,
    private val onBattery: (volts: Float, percent: Float) -> Unit = { _, _ -> },
    private val onUptime: (seconds: Long) -> Unit = {},
) {
    // Only flags a dead link in the UI; the robot itself stops on the controller's 0.5 s cmd_vel_timeout.
    // 2 s tripped on Wi-Fi while the same link carries the video stream.
    private val http = OkHttpClient.Builder()
        .pingInterval(10, TimeUnit.SECONDS)
        .socketFactory(SmallSendBufferSocketFactory)
        .build()
    @Volatile private var ws: WebSocket? = null
    @Volatile private var ready = false
    private val clock = RobotClock()
    private val subscribed = mutableSetOf<Int>()  // subscription ids already requested on this socket

    fun connect(url: String) {
        close()
        val request = Request.Builder().url(url)
            .header("Sec-WebSocket-Protocol", "foxglove.sdk.v1, foxglove.websocket.v1")
            .build()
        onState("connecting")
        subscribed.clear()
        ws = http.newWebSocket(request, object : WebSocketListener() {
            // A reconnect replaces ws; late callbacks from the old socket must not touch state.
            override fun onMessage(webSocket: WebSocket, text: String) {
                if (webSocket !== ws) return
                val msg = JSONObject(text)
                when (msg.optString("op")) {
                    "serverInfo" -> {
                        val caps = msg.optJSONArray("capabilities") ?: JSONArray()
                        if ((0 until caps.length()).none { caps.getString(it) == "clientPublish" }) {
                            onState("bridge has no clientPublish")
                            return
                        }
                        webSocket.send(advertiseChannels())
                        ready = true
                        onState("connected")
                    }
                    "advertise" -> {
                        val channels = msg.optJSONArray("channels") ?: return
                        for (i in 0 until channels.length()) {
                            val c = channels.getJSONObject(i)
                            val sub = SUBSCRIPTIONS[c.optString("topic")] ?: continue
                            if (subscribed.add(sub)) webSocket.send(subscribe(sub, c.getInt("id")))
                        }
                    }
                }
            }

            // Server "Message Data": opcode 0x01, u32 subscription id, u64 robot log time (ns), payload.
            override fun onMessage(webSocket: WebSocket, bytes: ByteString) {
                if (webSocket !== ws || bytes.size < 13 || bytes[0] != 0x01.toByte()) return
                val b = ByteBuffer.wrap(bytes.toByteArray()).order(ByteOrder.LITTLE_ENDIAN)
                val payload = bytes.toByteArray().copyOfRange(13, bytes.size)
                when (b.getInt(1)) {
                    BATTERY_SUB -> {
                        clock.sample(b.getLong(5) / 1_000_000, System.currentTimeMillis())
                        batteryState(payload)?.let { (v, pct) -> onBattery(v, pct) }
                    }
                    UPTIME_SUB -> uint32(payload)?.let(onUptime)
                }
            }

            override fun onClosed(webSocket: WebSocket, code: Int, reason: String) {
                if (webSocket !== ws) return
                ready = false
                onState("closed")
            }

            override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
                if (webSocket !== ws) return
                ready = false
                onState("error: ${t.message}")
            }
        })
    }

    /**
     * Thread-safe. Returns false (command dropped) until the bridge accepted the advertisement, or
     * while the previous command is still waiting to go out: on a stalled link, queued commands would
     * reach the robot seconds late and be executed as fresh.
     */
    fun sendCmdVel(linear: Double, angular: Double): Boolean {
        val socket = ws ?: return false
        if (!ready || socket.queueSize() > 0) return false
        return socket.send(clientMessage(CMD_VEL_CHANNEL, twistStampedJson(linear, angular, clock.nowMs() ?: 0)).toByteString())
    }

    /** Onboard RGB LED (led_controller): mode 0 off, 1 solid, 2 blink, 3 alternate; colours 0xRRGGBB. */
    fun sendLed(mode: Int, color: Int, altColor: Int = 0, periodMs: Int = 500): Boolean {
        val socket = ws ?: return false
        if (!ready) return false
        return socket.send(clientMessage(LED_CHANNEL, ledJson(mode, color, altColor, periodMs)).toByteString())
    }

    fun close() {
        ready = false
        clock.reset()
        ws?.close(1000, null)
        ws = null
    }

    companion object {
        const val CMD_VEL_CHANNEL = 1
        const val BATTERY_SUB = 1
        const val UPTIME_SUB = 2
        const val LED_CHANNEL = 2
        val SUBSCRIPTIONS = mapOf(
            "/battery_state_broadcaster/battery_state" to BATTERY_SUB,
            "/merlin/uptime" to UPTIME_SUB,  // merlin_bluetooth, std_msgs/UInt32 seconds
        )

        fun subscribe(subscriptionId: Int, channelId: Int): String = JSONObject()
            .put("op", "subscribe")
            .put("subscriptions", JSONArray().put(JSONObject().put("id", subscriptionId).put("channelId", channelId)))
            .toString()

        /** sensor_msgs/BatteryState CDR -> (voltage, percentage 0-100), or null if malformed. */
        fun batteryState(cdr: ByteArray): Pair<Float, Float>? = runCatching {
            val b = ByteBuffer.wrap(cdr).order(ByteOrder.LITTLE_ENDIAN)
            // 4 B encapsulation, stamp (8 B), frame_id (u32 length + bytes), floats aligned to 4.
            var pos = 16 + b.getInt(12)
            pos += (4 - (pos - 4) % 4) % 4
            b.getFloat(pos) to b.getFloat(pos + 24)
        }.getOrNull()

        /** std_msgs/UInt32 CDR: 4 B encapsulation, then the value. */
        fun uint32(cdr: ByteArray): Long? =
            if (cdr.size < 8) null else ByteBuffer.wrap(cdr).order(ByteOrder.LITTLE_ENDIAN).getInt(4).toLong() and 0xFFFFFFFFL

        fun advertiseChannels(): String {
            fun channel(id: Int, topic: String, schema: String) = JSONObject()
                .put("id", id).put("topic", topic).put("encoding", "json").put("schemaName", schema)
            return JSONObject()
                .put("op", "advertise")
                .put("channels", JSONArray()
                    .put(channel(CMD_VEL_CHANNEL, "/cmd_vel", "geometry_msgs/msg/TwistStamped"))
                    .put(channel(LED_CHANNEL, "/led_controller/commands", "control_msgs/msg/DynamicInterfaceGroupValues")))
                .toString()
        }

        fun ledJson(mode: Int, color: Int, altColor: Int, periodMs: Int): String =
            """{"header":{"stamp":{"sec":0,"nanosec":0},"frame_id":""},"interface_groups":["led"],""" +
                """"interface_values":[{"interface_names":["led_mode","led_color","led_color_alt","led_period_ms"],""" +
                """"values":[$mode.0,$color.0,$altColor.0,$periodMs.0]}]}"""

        // Stamped in ROBOT time (see RobotClock) so diff_drive_controller drops anything older than its
        // 0.5 s cmd_vel_timeout: a Wi-Fi stall that delivers commands late can't replay them. 0 = no
        // robot time yet: the controller then stamps on arrival (dead-man still works, no stale check).
        fun twistStampedJson(linear: Double, angular: Double, nowMs: Long): String =
            """{"header":{"stamp":{"sec":${nowMs / 1000},"nanosec":${nowMs % 1000 * 1_000_000}},"frame_id":""},""" +
                """"twist":{"linear":{"x":$linear,"y":0.0,"z":0.0},"angular":{"x":0.0,"y":0.0,"z":$angular}}}"""

        /** Foxglove "Client Message Data": opcode 0x01, u32 LE channel id, payload. */
        fun clientMessage(channelId: Int, json: String): ByteArray {
            val payload = json.toByteArray()
            return ByteBuffer.allocate(5 + payload.size).order(ByteOrder.LITTLE_ENDIAN)
                .put(0x01).putInt(channelId).put(payload).array()
        }
    }
}

/**
 * Estimates the robot clock from foxglove_bridge's per-message log times, so command stamps never depend
 * on the phone and robot clocks agreeing. A stamp in the robot's future would defeat the dead-man stop
 * (measured: stamps 5 s ahead kept the wheels turning 5.8 s after the link went silent), and a Jetson
 * booted without NTP can be hours off.
 *
 * Each sample is (robot log time - phone receive time) = offset - transit latency, which is never above
 * the true offset. Keeping the max over a sliding window takes the lowest-latency sample, so estimated
 * robot time trails the real one by a few ms and can never run ahead of it.
 */
class RobotClock(private val window: Int = 30) {
    private val samples = ArrayDeque<Long>()

    @Synchronized fun sample(robotMs: Long, phoneMs: Long) {
        samples.addLast(robotMs - phoneMs)
        if (samples.size > window) samples.removeFirst()
    }

    /** Estimated robot wall clock (ms), or null before the first sample. */
    @Synchronized fun nowMs(phoneMs: Long = System.currentTimeMillis()): Long? = samples.maxOrNull()?.plus(phoneMs)

    @Synchronized fun reset() = samples.clear()
}

/**
 * Caps the kernel send buffer (Linux doubles it, minimum ~4.6 KB: roughly a second of commands), so a
 * stalled Wi-Fi link can't hold a long backlog that later replays on the robot.
 */
private object SmallSendBufferSocketFactory : SocketFactory() {
    private val base = getDefault()
    private fun Socket.tune() = apply { sendBufferSize = 2048; tcpNoDelay = true }
    override fun createSocket(): Socket = base.createSocket().tune()
    override fun createSocket(host: String, port: Int): Socket = base.createSocket(host, port).tune()
    override fun createSocket(host: String, port: Int, localHost: InetAddress, localPort: Int): Socket =
        base.createSocket(host, port, localHost, localPort).tune()
    override fun createSocket(host: InetAddress, port: Int): Socket = base.createSocket(host, port).tune()
    override fun createSocket(address: InetAddress, port: Int, localAddress: InetAddress, localPort: Int): Socket =
        base.createSocket(address, port, localAddress, localPort).tune()
}
