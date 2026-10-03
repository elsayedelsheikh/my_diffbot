package com.merlin.app

import android.os.Bundle
import android.view.WindowManager
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.compose.animation.core.animateFloatAsState
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.gestures.awaitEachGesture
import androidx.compose.foundation.gestures.awaitFirstDown
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.hapticfeedback.HapticFeedbackType
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.LocalHapticFeedback
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.view.WindowInsetsControllerCompat
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.withContext
import kotlinx.coroutines.launch
import org.webrtc.EglBase
import org.webrtc.RendererCommon
import org.webrtc.SurfaceViewRenderer
import java.util.Locale
import kotlin.math.hypot

// merlin_base_controller limits (base_controllers.yaml); full stick deflection maps to these.
private const val MAX_LINEAR = 0.35   // m/s
private const val MAX_ANGULAR = 1.5   // rad/s

// Lean app bridge (docker compose service foxglove-app), not Foxglove Studio's 8765: see
// merlin_bringup/config/foxglove_app.yaml for why the full bridge stalls a Wi-Fi phone.
private const val BRIDGE_PORT = 8766

private val Ok = Color(0xFF34D399)
private val Busy = Color(0xFFFBBF24)
private val Bad = Color(0xFFF87171)
private val Idle = Color(0xFF6B7280)
private val Accent = Color(0xFF8B5CF6)

class MainActivity : ComponentActivity() {
    private val eglBase: EglBase = EglBase.create()
    private var videoState by mutableStateOf("idle")
    private var bridgeState by mutableStateOf("idle")
    private var batteryPercent by mutableStateOf<Float?>(null)
    private lateinit var whep: WhepClient
    private var uptimeSeconds by mutableStateOf<Long?>(null)
    private val foxglove = FoxgloveClient({ bridgeState = it }, { _, pct -> batteryPercent = pct }, { uptimeSeconds = it })

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        // Immersive: no status/navigation bars; a swipe from the edge shows them briefly.
        WindowCompat.setDecorFitsSystemWindows(window, false)
        if (android.os.Build.VERSION.SDK_INT >= 28) {  // draw under the camera cutout too
            window.attributes.layoutInDisplayCutoutMode = WindowManager.LayoutParams.LAYOUT_IN_DISPLAY_CUTOUT_MODE_SHORT_EDGES
        }
        WindowInsetsControllerCompat(window, window.decorView).apply {
            hide(WindowInsetsCompat.Type.systemBars())
            systemBarsBehavior = WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
        }
        whep = WhepClient(this, eglBase) { videoState = it }
        val prefs = getPreferences(MODE_PRIVATE)

        val renderer = SurfaceViewRenderer(this).apply {
            init(eglBase.eglBaseContext, null)
            setScalingType(RendererCommon.ScalingType.SCALE_ASPECT_FILL)
        }

        setContent {
            MaterialTheme(colorScheme = darkColorScheme(primary = Accent)) {
                val scope = rememberCoroutineScope()
                var host by remember { mutableStateOf(prefs.getString("host", "") ?: "") }
                // ponytail: plain app-private prefs; move to EncryptedSharedPreferences if the phone is shared.
                var password by remember { mutableStateOf(prefs.getString("password", "") ?: "") }
                var videoOn by remember { mutableStateOf(prefs.getBoolean("video", true)) }
                // Off: cam_low (640x360, ~200 kbit/s) fits a router relaying Wi-Fi to Wi-Fi; on: full
                // 1080p, for when the phone talks to the robot directly (robot hotspot).
                var hd by remember { mutableStateOf(prefs.getBoolean("hd", false)) }
                var settingsOpen by remember { mutableStateOf(host.isEmpty()) }
                // "Robot hotspot" / "Phone hotspot" / "Wi-Fi": how this phone reaches the robot right now.
                var method by remember { mutableStateOf("") }
                var linear by remember { mutableFloatStateOf(0f) }
                var angular by remember { mutableFloatStateOf(0f) }

                fun startVideo() = scope.launch {
                    whep.connect("http://$host:8889/${if (hd) "cam" else "cam_low"}/whep", password, renderer)
                }
                fun stopVideo() {
                    whep.close()
                    renderer.clearImage()
                    videoState = "off"
                }
                fun connect() {
                    foxglove.connect("ws://$host:$BRIDGE_PORT")
                    if (videoOn) startVideo() else stopVideo()
                }

                // 20 Hz while a stick is deflected; on release, retry the stop until it actually goes out.
                // If the link drops, the controller's 0.5 s cmd_vel_timeout stops the robot.
                LaunchedEffect(Unit) {
                    var stopPending = false
                    while (true) {
                        if (linear != 0f || angular != 0f) {
                            foxglove.sendCmdVel(linear * MAX_LINEAR, angular * MAX_ANGULAR)
                            stopPending = true
                        } else if (stopPending) {
                            stopPending = !foxglove.sendCmdVel(0.0, 0.0)
                        }
                        delay(50)
                    }
                }

                // Connect on launch, then keep both links up: retry whatever dropped every 2 s.
                LaunchedEffect(host, password) {
                    if (host.isEmpty()) return@LaunchedEffect
                    connect()
                    while (true) {
                        delay(2000)
                        if (bridgeState.isDown()) foxglove.connect("ws://$host:$BRIDGE_PORT")
                        if (videoOn && videoState.isDown() && !videoState.contains("password")) startVideo()
                    }
                }

                // Re-checked while connected: the phone can change networks under a live link, and an
                // effect keyed on the bridge state misses a reconnect that completes within one frame.
                LaunchedEffect(host) {
                    while (true) {
                        if (bridgeState == "connected") {
                            method = withContext(Dispatchers.IO) { RobotWifi.method(this@MainActivity, host) }
                        } else {
                            method = ""
                            uptimeSeconds = null
                        }
                        delay(2000)
                    }
                }

                Box(Modifier.fillMaxSize().background(Color.Black)) {
                    AndroidView({ renderer }, Modifier.fillMaxSize())
                    if (!videoOn || videoState != "connected") {
                        Text(
                            if (!videoOn) "Video off" else "Waiting for camera…",
                            Modifier.align(Alignment.Center), color = Color(0x99FFFFFF), fontSize = 14.sp,
                        )
                    }

                    StatusPill(
                        videoOn, videoState, bridgeState, batteryPercent, uptimeSeconds, method,
                        Modifier.align(Alignment.TopCenter).padding(top = 12.dp).clickable { settingsOpen = true },
                    )

                    // Left stick: forward/back. Right stick: turn (left = positive yaw).
                    Joystick(
                        Modifier.align(Alignment.BottomStart).padding(start = 48.dp, bottom = 36.dp),
                        label = "%+.2f m/s".format(Locale.US, linear * MAX_LINEAR),
                    ) { _, y -> linear = -y }
                    Joystick(
                        Modifier.align(Alignment.BottomEnd).padding(end = 48.dp, bottom = 36.dp),
                        label = "%+.1f rad/s".format(Locale.US, angular * MAX_ANGULAR),
                    ) { x, _ -> angular = -x }

                    if (settingsOpen) {
                        SetupPanel(
                            prefs, host, password,
                            summary = if (method.isEmpty()) "Connecting to $host…" else "$method · $host",
                            videoOn = videoOn, hd = hd,
                            onRobot = { h, p ->
                                prefs.edit().putString("host", h).putString("password", p).apply()
                                val same = h == host && p == password
                                host = h
                                password = p
                                if (same) connect()  // otherwise the LaunchedEffect above reconnects
                            },
                            // Video off: the Jetson stops the camera 10 s later.
                            onVideo = {
                                videoOn = it
                                prefs.edit().putBoolean("video", it).apply()
                                if (host.isNotEmpty()) { if (it) startVideo() else stopVideo() }
                            },
                            onHd = {
                                hd = it
                                prefs.edit().putBoolean("hd", it).apply()
                                if (host.isNotEmpty() && videoOn) startVideo()
                            },
                            onLed = { mode, color -> foxglove.sendLed(mode, color) },
                            onClose = { settingsOpen = false },
                        )
                    }
                }
            }
        }
    }

    override fun onDestroy() {
        foxglove.close()
        whep.close()
        eglBase.release()
        super.onDestroy()
    }
}

private fun String.isDown() = startsWith("error") || this in setOf("closed", "failed", "disconnected")

private fun String.color() = when {
    this == "connected" -> Ok
    this == "off" || this == "idle" -> Idle
    isDown() -> Bad
    else -> Busy
}

/** Minimal glass pill: one dot per link; tap opens the settings. */
@Composable
private fun StatusPill(
    videoOn: Boolean, video: String, drive: String, battery: Float?, uptime: Long?, method: String, modifier: Modifier,
) {
    Row(
        modifier
            .background(Color(0x66000000), RoundedCornerShape(50))
            .border(1.dp, Color(0x22FFFFFF), RoundedCornerShape(50))
            .padding(horizontal = 14.dp, vertical = 6.dp),
        verticalAlignment = Alignment.CenterVertically,
        horizontalArrangement = Arrangement.spacedBy(6.dp),
    ) {
        Dot(if (videoOn) video.color() else Idle)
        Text("Video", color = Color.White, fontSize = 12.sp)
        Spacer(Modifier.width(6.dp))
        Dot(drive.color())
        Text("Drive", color = Color.White, fontSize = 12.sp)
        if (battery != null && !battery.isNaN()) {
            Spacer(Modifier.width(6.dp))
            Text(
                "%.0f%%".format(Locale.US, battery), fontSize = 12.sp,
                color = if (battery < 20f) Bad else Color.White,
            )
        }
        if (uptime != null) {
            Spacer(Modifier.width(6.dp))
            Text("up ${uptimeText(uptime)}", color = Color(0xCCFFFFFF), fontSize = 12.sp)
        }
        if (method.isNotEmpty()) {
            Spacer(Modifier.width(6.dp))
            Text(method, color = Accent, fontSize = 12.sp, fontWeight = FontWeight.Medium)
        }
        Spacer(Modifier.width(6.dp))
        Text("MERLIN", color = Color(0x99FFFFFF), fontSize = 11.sp, fontWeight = FontWeight.SemiBold, letterSpacing = 2.sp)
    }
}

/** 42 -> "0m", 3700 -> "1h 01m", 200000 -> "2d 7h". */
internal fun uptimeText(seconds: Long): String {
    val m = seconds / 60
    return when {
        m < 60 -> "${m}m"
        m < 24 * 60 -> "%dh %02dm".format(Locale.US, m / 60, m % 60)
        else -> "${m / 1440}d ${m / 60 % 24}h"
    }
}

@Composable
private fun Dot(color: Color) = Box(Modifier.size(8.dp).background(color, CircleShape))

/**
 * Thumbstick that tracks the finger directly (no touch slop, the knob jumps under the thumb) and springs
 * back on release. Reports x/y in [-1, 1] (screen axes, y down), 0/0 on release.
 */
@Composable
private fun Joystick(modifier: Modifier, label: String, onMove: (Float, Float) -> Unit) {
    val haptics = LocalHapticFeedback.current
    var knob by remember { mutableStateOf(Offset.Zero) }  // normalized
    var active by remember { mutableStateOf(false) }
    val glow by animateFloatAsState(if (active) 1f else 0f, label = "glow")
    Column(modifier, horizontalAlignment = Alignment.CenterHorizontally) {
        Canvas(
            Modifier
                .size(170.dp)
                .pointerInput(Unit) {
                    val r = size.width / 2f
                    val center = Offset(r, r)
                    fun track(p: Offset) {
                        val d = (p - center) / r
                        val len = hypot(d.x, d.y)
                        knob = if (len > 1f) d / len else d
                        onMove(knob.x, knob.y)
                    }
                    awaitEachGesture {
                        val down = awaitFirstDown()
                        down.consume()
                        active = true
                        haptics.performHapticFeedback(HapticFeedbackType.TextHandleMove)
                        track(down.position)
                        val id = down.id
                        while (true) {
                            val change = awaitPointerEvent().changes.firstOrNull { it.id == id } ?: break
                            if (!change.pressed) break
                            change.consume()
                            track(change.position)
                        }
                        active = false
                        knob = Offset.Zero
                        onMove(0f, 0f)
                    }
                },
        ) {
            val r = size.minDimension / 2f
            drawCircle(Color(0x33000000), r)
            drawCircle(Color.White.copy(alpha = 0.18f + 0.25f * glow), r - 1.dp.toPx(), style = Stroke(1.5.dp.toPx()))
            drawCircle(Color.White.copy(alpha = 0.08f), r * 0.5f, style = Stroke(1.dp.toPx()))
            val k = center + knob * (r - 30.dp.toPx())
            drawCircle(Accent.copy(alpha = 0.35f * glow), 46.dp.toPx(), k)
            drawCircle(
                Brush.radialGradient(listOf(Color.White, Color(0xFFD4D4D8)), center = k, radius = 28.dp.toPx()),
                28.dp.toPx(), k,
            )
        }
        Text(label, color = Color.White.copy(alpha = 0.55f + 0.45f * glow), fontSize = 12.sp, modifier = Modifier.padding(top = 6.dp))
    }
}
