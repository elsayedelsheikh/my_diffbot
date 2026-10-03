package com.merlin.app

import android.annotation.SuppressLint
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanFilter
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.Context
import android.os.Build
import android.os.ParcelUuid
import kotlinx.coroutines.CompletableDeferred
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import kotlinx.coroutines.withTimeout
import org.json.JSONObject
import java.util.UUID

// merlin_bluetooth/README.md: the robot's provisioning service.
private val SERVICE = UUID.fromString("7f1d0000-5c2a-4b8e-9d3f-6a1e2b4c8d01")
private val REQUEST = UUID.fromString("7f1d0001-5c2a-4b8e-9d3f-6a1e2b4c8d01")
private val RESPONSE = UUID.fromString("7f1d0002-5c2a-4b8e-9d3f-6a1e2b4c8d01")
private val CCCD = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")

class BleException(val code: String, message: String) : Exception(message)

/**
 * One BLE session with the robot (merlin_bluetooth/README.md). Write a JSON line, wait for the
 * "reply ready" notification, then read the reply: the robot keeps one reply per device. No bonding:
 * pair() runs SRP-6a on the PIN and everything after it is sealed with AES-GCM.
 */
@SuppressLint("MissingPermission")  // the caller holds BLUETOOTH_SCAN/CONNECT before connect()
class MerlinBle(private val context: Context) {
    private var gatt: BluetoothGatt? = null
    private var pending: CompletableDeferred<Pair<Int, ByteArray>>? = null  // the one GATT op in flight
    private val ready = Channel<Unit>(Channel.CONFLATED)
    private val lock = Mutex()  // one request at a time: the robot keeps one reply per phone
    private var nextId = 1

    private val callback = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(g: BluetoothGatt, status: Int, state: Int) {
            if (state == BluetoothProfile.STATE_CONNECTED) g.requestMtu(517)
            else pending?.completeExceptionally(BleException("disconnected", "Bluetooth link lost ($status)"))
        }
        override fun onMtuChanged(g: BluetoothGatt, mtu: Int, status: Int) { g.discoverServices() }
        override fun onServicesDiscovered(g: BluetoothGatt, status: Int) { pending?.complete(status to ByteArray(0)) }
        override fun onDescriptorWrite(g: BluetoothGatt, d: BluetoothGattDescriptor, status: Int) {
            pending?.complete(status to ByteArray(0))
        }
        override fun onCharacteristicWrite(g: BluetoothGatt, c: BluetoothGattCharacteristic, status: Int) {
            pending?.complete(status to ByteArray(0))
        }
        override fun onCharacteristicRead(g: BluetoothGatt, c: BluetoothGattCharacteristic, value: ByteArray, status: Int) {
            pending?.complete(status to value)
        }
        @Deprecated("API < 33")
        override fun onCharacteristicRead(g: BluetoothGatt, c: BluetoothGattCharacteristic, status: Int) {
            @Suppress("DEPRECATION")
            if (Build.VERSION.SDK_INT < 33) pending?.complete(status to (c.value ?: ByteArray(0)))
        }
        override fun onCharacteristicChanged(g: BluetoothGatt, c: BluetoothGattCharacteristic, value: ByteArray) {
            ready.trySend(Unit)
        }
        @Deprecated("API < 33")
        override fun onCharacteristicChanged(g: BluetoothGatt, c: BluetoothGattCharacteristic) {
            if (Build.VERSION.SDK_INT < 33) ready.trySend(Unit)
        }
    }

    /** Scan for the robot's service, connect over LE and subscribe. */
    suspend fun connect() = withTimeout(60_000) {
        val device = scan()
        val op = CompletableDeferred<Pair<Int, ByteArray>>().also { pending = it }
        gatt = device.connectGatt(context, false, callback, BluetoothDevice.TRANSPORT_LE)
        check(op.await().first == BluetoothGatt.GATT_SUCCESS) { "service discovery failed" }
        val g = gatt!!
        val rsp = g.getService(SERVICE)?.getCharacteristic(RESPONSE)
            ?: throw BleException("not_merlin", "robot has no Merlin service")
        g.setCharacteristicNotification(rsp, true)
        gattOp {
            val d = rsp.getDescriptor(CCCD)
            if (Build.VERSION.SDK_INT >= 33) {
                g.writeDescriptor(d, BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE) == 0
            } else {
                @Suppress("DEPRECATION")
                d.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                @Suppress("DEPRECATION")
                g.writeDescriptor(d)
            }
        }
    }

    private suspend fun scan(): BluetoothDevice {
        val adapter = context.getSystemService(BluetoothManager::class.java).adapter
        if (adapter?.isEnabled != true) throw BleException("bluetooth_off", "Turn Bluetooth on, then try again")
        val scanner = adapter.bluetoothLeScanner ?: throw BleException("bluetooth_off", "Turn Bluetooth on, then try again")
        val found = CompletableDeferred<BluetoothDevice>()
        val cb = object : ScanCallback() {
            override fun onScanResult(type: Int, result: ScanResult) { found.complete(result.device) }
            override fun onScanFailed(code: Int) { found.completeExceptionally(BleException("scan", "scan failed ($code)")) }
        }
        val filter = ScanFilter.Builder().setServiceUuid(ParcelUuid(SERVICE)).build()
        scanner.startScan(listOf(filter), ScanSettings.Builder().setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build(), cb)
        try {
            return withTimeout(20_000) { found.await() }
        } catch (e: kotlinx.coroutines.TimeoutCancellationException) {
            throw BleException("not_found", "robot not found over Bluetooth")
        } finally {
            scanner.stopScan(cb)
        }
    }

    private var session: SealedSession? = null

    /** SRP-6a against the robot PIN: the PIN never crosses the air; a wrong one throws bad_pin. */
    suspend fun pair(pin: String) {
        session = null
        val start = exchange(JSONObject().put("id", nextId++).put("op", "pair_start"))
        val proof = Srp.prove(pin, Srp.unb64(start.getString("salt")), Srp.unb64(start.getString("B")))
        val done = exchange(
            JSONObject().put("id", nextId++).put("op", "pair_verify")
                .put("A", Srp.b64(proof.A)).put("M1", Srp.b64(proof.m1)),
        )
        // The robot proves it knows the PIN too, so a look-alike robot can't take the session.
        if (!Srp.unb64(done.getString("M2")).contentEquals(proof.m2)) throw BleException("bad_robot", "robot failed its proof")
        session = SealedSession(proof.key)
    }

    /** Send one request over the sealed session; returns the reply or throws BleException(error code). */
    suspend fun call(op: String, args: Map<String, String> = emptyMap(), timeoutMs: Long = 90_000): JSONObject {
        val s = session ?: throw BleException("unauthorized", "pair first")
        val id = nextId++
        val inner = JSONObject(args as Map<*, *>).put("id", id).put("op", op).toString()
        return lock.withLock {
            val (n, c) = s.seal(inner)
            val envelope = exchange(JSONObject().put("n", n).put("c", c), timeoutMs, check = false)
            if (!envelope.has("c")) throw BleException(envelope.optString("error"), envelope.optString("msg"))
            JSONObject(s.open(envelope.getLong("n"), envelope.getString("c"))).also { checkOk(it) }
        }
    }

    private fun checkOk(reply: JSONObject) {
        if (!reply.optBoolean("ok")) throw BleException(reply.optString("error"), reply.optString("msg", reply.optString("error")))
    }

    /** Write one line, wait for "reply ready", read this phone's reply (GATT values are per device). */
    private suspend fun exchange(msg: JSONObject, timeoutMs: Long = 30_000, check: Boolean = true): JSONObject {
        val g = gatt ?: throw BleException("disconnected", "not connected")
        val req = g.getService(SERVICE).getCharacteristic(REQUEST)
        val rsp = g.getService(SERVICE).getCharacteristic(RESPONSE)
        val body = (msg.toString() + "\n").toByteArray()
        while (ready.tryReceive().isSuccess) Unit  // drop a stale "ready"
        gattOp {
            if (Build.VERSION.SDK_INT >= 33) {
                g.writeCharacteristic(req, body, BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE) == 0
            } else {
                req.writeType = BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE
                @Suppress("DEPRECATION")
                req.value = body
                @Suppress("DEPRECATION")
                g.writeCharacteristic(req)
            }
        }
        val reply = withTimeout(timeoutMs) {
            ready.receive()
            JSONObject(String(gattOp { g.readCharacteristic(rsp) }))
        }
        if (check) checkOk(reply)
        return reply
    }

    /** Run one GATT operation and wait for its callback; Android allows only one at a time. */
    private suspend fun gattOp(start: () -> Boolean): ByteArray {
        val op = CompletableDeferred<Pair<Int, ByteArray>>().also { pending = it }
        if (!start()) throw BleException("busy", "Bluetooth operation refused")
        val (status, value) = withTimeout(60_000) { op.await() }
        if (status != BluetoothGatt.GATT_SUCCESS) throw BleException("gatt", "Bluetooth error $status")
        return value
    }

    fun close() {
        gatt?.close()
        gatt = null
    }
}
