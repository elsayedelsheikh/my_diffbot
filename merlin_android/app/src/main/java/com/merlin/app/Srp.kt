package com.merlin.app

import java.math.BigInteger
import java.security.MessageDigest
import java.security.SecureRandom
import java.util.Base64
import javax.crypto.Cipher
import javax.crypto.Mac
import javax.crypto.spec.GCMParameterSpec
import javax.crypto.spec.SecretKeySpec

/**
 * SRP-6a client (RFC 5054 math) + the AES-GCM session it keys. Mirrors merlin_bluetooth/srp.py
 * byte for byte: the PIN never leaves the phone, and a fake robot can't learn it.
 */
object Srp {
    // RFC 5054 Appendix A, 2048-bit group.
    val N = BigInteger(
        "AC6BDB41324A9A9BF166DE5E1389582FAF72B6651987EE07FC3192943DB56050A37329CBB4A099ED" +
            "8193E0757767A13DD52312AB4B03310DCD7F48A9DA04FD50E8083969EDB767B0CF6095179A163AB3" +
            "661A05FBD5FAAAE82918A9962F0B93B855F97993EC975EEAA80D740ADBF4FF747359D041D5C33EA7" +
            "1D281E446B14773BCA97B43A23FB801676BD207A436C6481F1D2B9078717461A5B9D32E688F87748" +
            "544523B524B0D57D5EA77A2775D2ECFA032CFBDBF52FB37861602790" +
            "04E57AE6AF874E7303CE53299CCC041C7BC308D82A5698F3A8D0C38271AE35F8E9DBFBB6" +
            "94B5C803D89F7AE435DE236D525F54759B65E372FCD68EF20FA7111F9E4AFF73", 16,
    )
    val G: BigInteger = BigInteger.valueOf(2)
    private val IDENTITY = "merlin".toByteArray()

    /** Fixed-length unsigned big-endian (toByteArray() adds a sign byte or drops leading zeros). */
    fun pad(n: BigInteger, length: Int = (N.bitLength() + 7) / 8): ByteArray {
        val raw = n.toByteArray()
        val out = ByteArray(length)
        val src = if (raw.size > length) raw.copyOfRange(raw.size - length, raw.size) else raw
        System.arraycopy(src, 0, out, length - src.size, src.size)
        return out
    }

    fun hash(algorithm: String, vararg parts: ByteArray): ByteArray =
        MessageDigest.getInstance(algorithm).run { parts.forEach(::update); digest() }

    private fun int(b: ByteArray) = BigInteger(1, b)

    class Proof(val A: ByteArray, val m1: ByteArray, val m2: ByteArray, val key: ByteArray)

    /** The phone's half of the handshake, from pair_start's salt and B. `a` is fixed only in tests. */
    fun prove(
        pin: String, salt: ByteArray, bBytes: ByteArray, alg: String = "SHA-256",
        a: BigInteger = BigInteger(256, SecureRandom()),
    ): Proof {
        val b = int(bBytes)
        if (b.mod(N) == BigInteger.ZERO) throw BleException("bad_robot", "robot sent an invalid key")
        val aPub = G.modPow(a, N)
        val u = int(hash(alg, pad(aPub), pad(b)))
        if (u == BigInteger.ZERO) throw BleException("bad_robot", "robot sent an invalid key")
        val k = int(hash(alg, pad(N), pad(G)))
        val x = int(hash(alg, salt, hash(alg, IDENTITY, ":".toByteArray(), pin.toByteArray())))
        val s = b.subtract(k.multiply(G.modPow(x, N))).mod(N).modPow(a.add(u.multiply(x)), N)
        val key = hash(alg, pad(s))
        val m1 = hash(alg, pad(aPub), pad(b), key)
        return Proof(pad(aPub), m1, hash(alg, pad(aPub), m1, key), key)
    }

    /** RFC 5869 HKDF-SHA256, 32 bytes. */
    fun hkdf(key: ByteArray, info: String): ByteArray {
        val prk = Mac.getInstance("HmacSHA256").run { init(SecretKeySpec("merlin-ble-2".toByteArray(), "HmacSHA256")); doFinal(key) }
        return Mac.getInstance("HmacSHA256").run { init(SecretKeySpec(prk, "HmacSHA256")); doFinal(info.toByteArray() + 1.toByte()) }
    }

    fun b64(b: ByteArray): String = Base64.getEncoder().encodeToString(b)
    fun unb64(s: String): ByteArray = Base64.getDecoder().decode(s)
}

/** AES-256-GCM both ways; the per-direction message counter is the nonce, so replays fail. */
class SealedSession(sessionKey: ByteArray) {
    private val tx = SecretKeySpec(Srp.hkdf(sessionKey, "phone->robot"), "AES")
    private val rx = SecretKeySpec(Srp.hkdf(sessionKey, "robot->phone"), "AES")
    private var txCount = 0L
    private var rxCount = 0L

    private fun nonce(counter: Long) = Srp.pad(BigInteger.valueOf(counter), 12)

    /** (counter, base64 ciphertext) for the {"n","c"} envelope. */
    fun seal(plain: String): Pair<Long, String> {
        txCount++
        val c = Cipher.getInstance("AES/GCM/NoPadding").apply { init(Cipher.ENCRYPT_MODE, tx, GCMParameterSpec(128, nonce(txCount))) }
        return txCount to Srp.b64(c.doFinal(plain.toByteArray()))
    }

    fun open(counter: Long, sealed: String): String {
        if (counter <= rxCount) throw BleException("replay", "replayed reply from the robot")
        val c = Cipher.getInstance("AES/GCM/NoPadding").apply { init(Cipher.DECRYPT_MODE, rx, GCMParameterSpec(128, nonce(counter))) }
        return String(c.doFinal(Srp.unb64(sealed))).also { rxCount = counter }
    }
}
