package com.merlin.app

import android.content.Context
import kotlinx.coroutines.CompletableDeferred
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.suspendCancellableCoroutine
import kotlinx.coroutines.withContext
import kotlinx.coroutines.withTimeoutOrNull
import org.webrtc.DefaultVideoDecoderFactory
import org.webrtc.EglBase
import org.webrtc.MediaConstraints
import org.webrtc.PeerConnection
import org.webrtc.PeerConnectionFactory
import org.webrtc.RtpTransceiver
import org.webrtc.SdpObserver
import org.webrtc.SessionDescription
import org.webrtc.VideoSink
import org.webrtc.VideoTrack
import java.net.HttpURLConnection
import java.net.URL
import kotlin.coroutines.resume
import kotlin.coroutines.resumeWithException

/** Receive-only WebRTC video over WHEP (MediaMTX): one HTTP POST of the offer, the answer comes back. */
class WhepClient(context: Context, eglBase: EglBase, private val onState: (String) -> Unit) {
    private val factory: PeerConnectionFactory
    private var pc: PeerConnection? = null

    init {
        PeerConnectionFactory.initialize(
            PeerConnectionFactory.InitializationOptions.builder(context.applicationContext)
                .createInitializationOptions()
        )
        factory = PeerConnectionFactory.builder()
            .setVideoDecoderFactory(DefaultVideoDecoderFactory(eglBase.eglBaseContext))
            .createPeerConnectionFactory()
    }

    suspend fun connect(url: String, password: String, sink: VideoSink) {
        close()
        onState("connecting")
        val gathered = CompletableDeferred<Unit>()
        // No ICE servers: robot and phone share the LAN, host candidates are enough.
        val config = PeerConnection.RTCConfiguration(emptyList()).apply {
            sdpSemantics = PeerConnection.SdpSemantics.UNIFIED_PLAN
        }
        val peer = factory.createPeerConnection(config, object : PeerConnection.Observer {
            override fun onTrack(transceiver: RtpTransceiver) {
                (transceiver.receiver.track() as? VideoTrack)?.addSink(sink)
            }
            override fun onIceGatheringChange(state: PeerConnection.IceGatheringState) {
                if (state == PeerConnection.IceGatheringState.COMPLETE) gathered.complete(Unit)
            }
            override fun onConnectionChange(state: PeerConnection.PeerConnectionState) {
                if (pc != null && state != PeerConnection.PeerConnectionState.CLOSED) onState(state.name.lowercase())
            }
            override fun onSignalingChange(state: PeerConnection.SignalingState) {}
            override fun onIceConnectionChange(state: PeerConnection.IceConnectionState) {}
            override fun onIceConnectionReceivingChange(receiving: Boolean) {}
            override fun onIceCandidate(candidate: org.webrtc.IceCandidate) {}
            override fun onIceCandidatesRemoved(candidates: Array<out org.webrtc.IceCandidate>) {}
            override fun onAddStream(stream: org.webrtc.MediaStream) {}
            override fun onRemoveStream(stream: org.webrtc.MediaStream) {}
            override fun onDataChannel(channel: org.webrtc.DataChannel) {}
            override fun onRenegotiationNeeded() {}
        }) ?: error("createPeerConnection failed")
        pc = peer
        peer.addTransceiver(
            org.webrtc.MediaStreamTrack.MediaType.MEDIA_TYPE_VIDEO,
            RtpTransceiver.RtpTransceiverInit(RtpTransceiver.RtpTransceiverDirection.RECV_ONLY)
        )

        try {
            val offer = sdp { peer.createOffer(it, MediaConstraints()) }
            unit { peer.setLocalDescription(it, offer) }
            // Non-trickle WHEP: send the offer once it carries every host candidate.
            withTimeoutOrNull(2000) { gathered.await() }
            val answer = post(url, password, peer.localDescription.description)
            unit { peer.setRemoteDescription(it, SessionDescription(SessionDescription.Type.ANSWER, answer)) }
        } catch (e: Exception) {
            if (pc === peer) onState("error: ${e.message}")  // else a newer connect() superseded this one
        }
    }

    fun close() {
        pc?.dispose()
        pc = null
    }

    private suspend fun post(url: String, password: String, offer: String): String = withContext(Dispatchers.IO) {
        val conn = URL(url).openConnection() as HttpURLConnection
        try {
            conn.requestMethod = "POST"
            conn.connectTimeout = 3000
            conn.readTimeout = 20000  // MediaMTX holds the answer until the camera pipeline is up
            conn.doOutput = true
            conn.setRequestProperty("Content-Type", "application/sdp")
            // MediaMTX viewer account (merlin_camera_streamer/install.sh prints the password).
            val token = java.util.Base64.getEncoder().encodeToString("merlin:$password".toByteArray())
            conn.setRequestProperty("Authorization", "Basic $token")
            conn.outputStream.use { it.write(offer.toByteArray()) }
            if (conn.responseCode == HttpURLConnection.HTTP_UNAUTHORIZED) error("wrong camera password")
            if (conn.responseCode != HttpURLConnection.HTTP_CREATED) {
                error("WHEP ${conn.responseCode}: ${conn.errorStream?.bufferedReader()?.readText().orEmpty()}")
            }
            conn.inputStream.bufferedReader().readText()
        } finally {
            conn.disconnect()
        }
    }

    private suspend fun sdp(call: (SdpObserver) -> Unit): SessionDescription =
        suspendCancellableCoroutine { cont ->
            call(object : SdpObserver {
                override fun onCreateSuccess(desc: SessionDescription) = cont.resume(desc)
                override fun onCreateFailure(error: String) = cont.resumeWithException(Exception(error))
                override fun onSetSuccess() {}
                override fun onSetFailure(error: String) {}
            })
        }

    private suspend fun unit(call: (SdpObserver) -> Unit): Unit =
        suspendCancellableCoroutine { cont ->
            call(object : SdpObserver {
                override fun onCreateSuccess(desc: SessionDescription) {}
                override fun onCreateFailure(error: String) {}
                override fun onSetSuccess() = cont.resume(Unit)
                override fun onSetFailure(error: String) = cont.resumeWithException(Exception(error))
            })
        }
}
