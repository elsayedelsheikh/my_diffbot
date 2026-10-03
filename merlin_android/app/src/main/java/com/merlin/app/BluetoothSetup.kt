package com.merlin.app

import android.Manifest
import android.content.Context
import android.content.SharedPreferences
import android.net.ConnectivityManager
import android.net.Network
import android.net.NetworkCapabilities
import android.net.NetworkRequest
import android.net.wifi.WifiNetworkSpecifier
import android.os.Build
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.interaction.MutableInteractionSource
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.WindowInsets
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.ime
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.safeDrawing
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.union
import androidx.compose.foundation.layout.widthIn
import androidx.compose.foundation.layout.windowInsetsPadding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.FilterChip
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Surface
import androidx.compose.material3.Switch
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.text.input.PasswordVisualTransformation
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import kotlinx.coroutines.CompletableDeferred
import kotlinx.coroutines.launch
import kotlinx.coroutines.withTimeout
import java.net.InetAddress
import java.net.NetworkInterface

private val BLE_PERMISSIONS = if (Build.VERSION.SDK_INT >= 31) {
    arrayOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
} else {
    arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)
}

/**
 * The phone's side of the robot hotspot: joins it as a local-only network (no internet) and binds
 * the whole app to it, so the bridge and video sockets go to the robot, not over mobile data.
 */
object RobotWifi {
    private var callback: ConnectivityManager.NetworkCallback? = null
    @Volatile var joined = false
        private set

    suspend fun join(context: Context, ssid: String, password: String) {
        release(context)
        if (Build.VERSION.SDK_INT < 29) {
            throw BleException("old_android", "join $ssid (password $password) in Android Wi-Fi settings")
        }
        val cm = context.getSystemService(ConnectivityManager::class.java)
        val done = CompletableDeferred<Network>()
        val cb = object : ConnectivityManager.NetworkCallback() {
            override fun onAvailable(network: Network) {
                cm.bindProcessToNetwork(network)
                joined = true
                done.complete(network)
            }
            override fun onUnavailable() { done.completeExceptionally(BleException("wifi", "didn't join $ssid")) }
            override fun onLost(network: Network) {
                joined = false
                cm.bindProcessToNetwork(null)
            }
        }
        val spec = WifiNetworkSpecifier.Builder().setSsid(ssid).setWpa2Passphrase(password).build()
        val request = NetworkRequest.Builder()
            .addTransportType(NetworkCapabilities.TRANSPORT_WIFI)
            .removeCapability(NetworkCapabilities.NET_CAPABILITY_INTERNET)
            .setNetworkSpecifier(spec)
            .build()
        callback = cb
        cm.requestNetwork(request, cb)  // Android asks the user to confirm the network the first time
        withTimeout(90_000) { done.await() }
    }

    /** Back to the phone's normal networks (robot on a shared Wi-Fi or the phone's own hotspot). */
    fun release(context: Context) {
        val cm = context.getSystemService(ConnectivityManager::class.java)
        callback?.let { runCatching { cm.unregisterNetworkCallback(it) } }
        callback = null
        joined = false
        cm.bindProcessToNetwork(null)
    }

    /**
     * How the phone reaches `host`, for the status bar: the robot's hotspot, this phone's hotspot, or a
     * shared Wi-Fi. Call off the main thread (resolves the host).
     */
    fun method(context: Context, host: String): String {
        if (joined) return "Robot hotspot"
        val cm = context.getSystemService(ConnectivityManager::class.java)
        val wifi = cm.allNetworks.firstOrNull {
            cm.getNetworkCapabilities(it)?.hasTransport(NetworkCapabilities.TRANSPORT_WIFI) == true
        }?.let { cm.getLinkProperties(it)?.interfaceName }
        val target = runCatching { InetAddress.getByName(host).address }.getOrNull() ?: return "Network"
        val via = NetworkInterface.getNetworkInterfaces().toList().firstOrNull { nif ->
            nif.interfaceAddresses.any { a ->
                val mine = a.address.address
                val bits = a.networkPrefixLength.toInt()
                mine.size == target.size && (0 until bits).all { i ->
                    (mine[i / 8].toInt() xor target[i / 8].toInt()) and (0x80 shr (i % 8)) == 0
                }
            }
        } ?: return "Network"  // routed (VPN, another subnet)
        // A local subnet that isn't the phone's Wi-Fi client link is the one its own hotspot serves.
        return if (via.name == wifi) "Wi-Fi" else "Phone hotspot"
    }
}

private val LED_COLORS = listOf(0xFF0000, 0x00FF00, 0x0000FF, 0xFFC800, 0x8B5CF6, 0xFFFFFF)

/**
 * The setup sheet, laid out for a landscape phone: connecting on the left (Bluetooth first; the IP and
 * camera password are filled in by it and only typed under Advanced), video and LED on the right.
 */
@Composable
fun SetupPanel(
    prefs: SharedPreferences,
    host: String,
    cameraPassword: String,
    summary: String,
    videoOn: Boolean,
    hd: Boolean,
    onRobot: (host: String, cameraPassword: String) -> Unit,
    onVideo: (Boolean) -> Unit,
    onHd: (Boolean) -> Unit,
    onLed: (mode: Int, color: Int) -> Unit,
    onClose: () -> Unit,
) {
    val context = LocalContext.current
    val scope = rememberCoroutineScope()
    val ble = remember { MerlinBle(context.applicationContext) }
    DisposableEffect(Unit) { onDispose { ble.close() } }
    var pin by remember { mutableStateOf(prefs.getString("bt_pin", "") ?: "") }
    var ssid by remember { mutableStateOf(prefs.getString("wifi_ssid", "") ?: "") }
    var wifiPassword by remember { mutableStateOf("") }
    var paired by remember { mutableStateOf(false) }
    var busy by remember { mutableStateOf(false) }
    var status by remember { mutableStateOf(if (host.isEmpty()) "Enter the robot PIN to find it over Bluetooth" else summary) }
    var advanced by remember { mutableStateOf(false) }
    var ledMode by remember { mutableIntStateOf(prefs.getInt("led_mode", 0)) }
    var ledColor by remember { mutableIntStateOf(prefs.getInt("led_color", LED_COLORS[1])) }

    fun run(label: String, work: suspend () -> Unit) {
        if (busy) return
        busy = true
        status = label
        scope.launch {
            try {
                work()
            } catch (e: BleException) {
                status = e.message?.ifEmpty { null } ?: e.code
                if (e.code in setOf("disconnected", "gatt", "busy", "bad_pin", "locked", "bad_robot", "replay", "unauthorized")) {
                    ble.close()
                    paired = false
                }
            } catch (e: Exception) {
                status = "Failed: ${e.message ?: e.javaClass.simpleName}"
                ble.close()
                paired = false
            } finally {
                busy = false
            }
        }
    }

    /** Read the robot's network + camera password and connect the app to it straight away. */
    suspend fun refresh() {
        val r = ble.call("status")
        val w = r.getJSONObject("wifi")
        status = when (w.optString("mode")) {
            "hotspot" -> "Robot hotspot ${w.optString("ssid")} · ${w.optString("ip")}"
            "station" -> "Robot on ${w.optString("ssid")} · ${w.optString("ip")}"
            else -> "Robot is not on any Wi-Fi: pick a network below"
        }
        if (w.optString("ip").isNotEmpty()) onRobot(w.optString("ip"), r.getJSONObject("camera").optString("password"))
    }

    val permissions = rememberLauncherForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { granted ->
        if (granted.values.all { it }) {
            run("Searching for the robot…") {
                ble.connect()
                ble.pair(pin)
                prefs.edit().putString("bt_pin", pin).apply()
                paired = true
                refresh()
            }
        } else {
            status = "Bluetooth permission denied"
        }
    }

    fun led(mode: Int, color: Int) {
        ledMode = mode
        ledColor = color
        prefs.edit().putInt("led_mode", mode).putInt("led_color", color).apply()
        onLed(mode, color)
    }

    // Scrim: tap outside to close. The sheet rides above the keyboard and clears the cutout.
    Box(
        Modifier.fillMaxSize().background(Color(0xB3000000))
            .clickable(remember { MutableInteractionSource() }, null, onClick = onClose)
            .windowInsetsPadding(WindowInsets.safeDrawing.union(WindowInsets.ime))
            .padding(horizontal = 24.dp, vertical = 12.dp),
        contentAlignment = Alignment.Center,
    ) {
        Surface(
            Modifier.widthIn(max = 920.dp).fillMaxHeight()
                .clickable(remember { MutableInteractionSource() }, null) {},  // swallow taps inside
            shape = RoundedCornerShape(24.dp), tonalElevation = 6.dp,
        ) {
            Column(Modifier.padding(horizontal = 20.dp, vertical = 12.dp)) {
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Text("MERLIN", fontWeight = FontWeight.SemiBold, letterSpacing = 3.sp, fontSize = 14.sp)
                    Text(
                        status, Modifier.weight(1f).padding(horizontal = 16.dp), fontSize = 13.sp, maxLines = 2,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                    )
                    TextButton(onClick = onClose) { Text("Done") }
                }
                Row(Modifier.weight(1f), horizontalArrangement = Arrangement.spacedBy(24.dp)) {
                    // ---- Connect -------------------------------------------------------------
                    Column(
                        Modifier.weight(1.2f).verticalScroll(rememberScrollState()),
                        verticalArrangement = Arrangement.spacedBy(8.dp),
                    ) {
                        Section("Connect")
                        if (!paired) {
                            Row(horizontalArrangement = Arrangement.spacedBy(8.dp), verticalAlignment = Alignment.CenterVertically) {
                                OutlinedTextField(
                                    pin, { pin = it.filter(Char::isDigit).take(12) }, singleLine = true,
                                    label = { Text("Robot PIN") }, modifier = Modifier.weight(1f),
                                    visualTransformation = PasswordVisualTransformation(),
                                    keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.NumberPassword),
                                )
                                Button(enabled = !busy && pin.length >= 6, onClick = { permissions.launch(BLE_PERMISSIONS) }) {
                                    Text(if (busy) "Searching…" else "Find robot")
                                }
                            }
                            Hint("Bluetooth finds the robot and sets everything up; the PIN is on the robot (MERLIN_BT_PIN).")
                        } else {
                            Button(enabled = !busy, modifier = Modifier.fillMaxWidth(), onClick = {
                                run("Starting the robot hotspot…") {
                                    val r = ble.call("start_hotspot")
                                    status = "Joining ${r.optString("ssid")}…"
                                    RobotWifi.join(context, r.optString("ssid"), r.optString("password"))
                                    refresh()
                                }
                            }) { Text("Use the robot's hotspot") }
                            Hint("Direct link, best for HD video. Or put the robot on a Wi-Fi: this phone's hotspot or a shared router.")
                            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                                OutlinedTextField(ssid, { ssid = it }, singleLine = true, label = { Text("Wi-Fi name") }, modifier = Modifier.weight(1f))
                                OutlinedTextField(
                                    wifiPassword, { wifiPassword = it }, singleLine = true, modifier = Modifier.weight(1f),
                                    label = { Text("Password") }, placeholder = { Text("saved on robot") },
                                    visualTransformation = PasswordVisualTransformation(),
                                )
                            }
                            OutlinedButton(enabled = !busy && ssid.isNotBlank(), modifier = Modifier.fillMaxWidth(), onClick = {
                                run("Moving the robot to ${ssid.trim()}…") {
                                    prefs.edit().putString("wifi_ssid", ssid.trim()).apply()
                                    RobotWifi.release(context)
                                    ble.call("join_wifi", mapOf("ssid" to ssid.trim(), "password" to wifiPassword))
                                    refresh()
                                }
                            }) { Text("Join this Wi-Fi") }
                        }
                        TextButton(onClick = { advanced = !advanced }) { Text(if (advanced) "Hide advanced" else "Advanced: enter the address by hand") }
                        if (advanced) {
                            var draftHost by remember { mutableStateOf(host) }
                            var draftPassword by remember { mutableStateOf(cameraPassword) }
                            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                                OutlinedTextField(draftHost, { draftHost = it.trim() }, singleLine = true, label = { Text("Robot IP") }, modifier = Modifier.weight(1f))
                                OutlinedTextField(
                                    draftPassword, { draftPassword = it.trim() }, singleLine = true, modifier = Modifier.weight(1f),
                                    label = { Text("Camera password") }, visualTransformation = PasswordVisualTransformation(),
                                )
                            }
                            OutlinedButton(enabled = draftHost.isNotEmpty(), modifier = Modifier.fillMaxWidth(), onClick = {
                                RobotWifi.release(context)
                                onRobot(draftHost, draftPassword)
                                status = "Connecting to $draftHost"
                            }) { Text("Connect to this address") }
                        }
                    }
                    // ---- Video + LED ---------------------------------------------------------
                    Column(
                        Modifier.weight(1f).verticalScroll(rememberScrollState()),
                        verticalArrangement = Arrangement.spacedBy(8.dp),
                    ) {
                        Section("Video")
                        Toggle("Video", "Off frees the Wi-Fi for driving", videoOn, onVideo)
                        Toggle("HD", "1080p; needs a direct link (a hotspot)", hd, onHd)
                        Section("LED")
                        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                            listOf("Off" to 0, "Solid" to 1, "Blink" to 2).forEach { (label, mode) ->
                                FilterChip(ledMode == mode, { led(mode, ledColor) }, { Text(label) })
                            }
                        }
                        Row(horizontalArrangement = Arrangement.spacedBy(12.dp)) {
                            LED_COLORS.forEach { c ->
                                Box(
                                    Modifier.size(34.dp).background(Color(0xFF000000 or c.toLong()), CircleShape)
                                        .border(
                                            if (c == ledColor) 3.dp else 1.dp,
                                            if (c == ledColor) MaterialTheme.colorScheme.primary else Color(0x55FFFFFF), CircleShape,
                                        )
                                        .clickable { led(if (ledMode == 0) 1 else ledMode, c) },
                                )
                            }
                        }
                    }
                }
            }
        }
    }
}

@Composable
private fun Section(title: String) =
    Text(title, style = MaterialTheme.typography.titleSmall, color = MaterialTheme.colorScheme.primary, modifier = Modifier.padding(top = 4.dp))

@Composable
private fun Hint(text: String) = Text(text, fontSize = 12.sp, color = MaterialTheme.colorScheme.onSurfaceVariant)

@Composable
private fun Toggle(title: String, hint: String, checked: Boolean, onChange: (Boolean) -> Unit) =
    Row(verticalAlignment = Alignment.CenterVertically) {
        Column(Modifier.weight(1f)) {
            Text(title)
            Hint(hint)
        }
        Switch(checked, onChange)
    }
