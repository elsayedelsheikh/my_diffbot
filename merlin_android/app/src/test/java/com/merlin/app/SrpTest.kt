package com.merlin.app

import org.junit.Assert.assertEquals
import org.junit.Test
import java.math.BigInteger

class SrpTest {
    private fun hex(b: ByteArray) = b.joinToString("") { "%02x".format(it) }
    private fun big(s: String) = BigInteger(s.filterNot(Char::isWhitespace), 16)

    /** RFC 5054 Appendix B (SHA-1, 1024-bit group): the client's view of the handshake. */
    @Test
    fun rfc5054Vectors() {
        val n = big("""EEAF0AB9 ADB38DD6 9C33F80A FA8FC5E8 60726187 75FF3C0B 9EA2314C 9C256576 D674DF74 96EA81D3
            383B4813 D692C6E0 E0D5D8E2 50B98BE4 8E495C1D 6089DAD1 5DC7D7B4 6154D6B6 CE8EF4AD 69B15D49
            82559B29 7BCF1885 C529F566 660E57EC 68EDBC3C 05726CC0 2FD4CBF4 976EAA9A FD5138FE 8376435B
            9FC61D2F C0EB06E3""")
        val len = 128
        val salt = big("BEB25379D1A8581EB5A727673A2441EE").let { Srp.pad(it, 16) }
        val a = big("60975527 035CF2AD 1989806F 0407210B C81EDC04 E2762A56 AFD529DD DA2D4393")
        val b = big("""BD0C6151 2C692C0C B6D041FA 01BB152D 4916A1E7 7AF46AE1 05393011 BAF38964 DC46A067 0DD125B9
            5A981652 236F99D9 B681CBF8 7837EC99 6C6DA044 53728610 D0C6DDB5 8B318885 D7D82C7F 8DEB75CE
            7BD4FBAA 37089E6F 9C6059F3 88838E7A 00030B33 1EB76840 910440B1 B27AAEAE EB4012B7 D7665238
            A8E3FB00 4B117B58""")
        val g = BigInteger.valueOf(2)
        val k = BigInteger(1, Srp.hash("SHA-1", Srp.pad(n, len), Srp.pad(g, len)))
        assertEquals(big("7556AA04 5AEF2CDD 07ABAF0F 665C3E81 8913186F"), k)
        val x = BigInteger(1, Srp.hash("SHA-1", salt, Srp.hash("SHA-1", "alice:password123".toByteArray())))
        assertEquals(big("94B7555A ABE9127C C58CCF49 93DB6CF8 4D16C124"), x)
        val aPub = g.modPow(a, n)
        val u = BigInteger(1, Srp.hash("SHA-1", Srp.pad(aPub, len), Srp.pad(b, len)))
        assertEquals(big("CE38B959 3487DA98 554ED47D 70A7AE5F 462EF019"), u)
        val s = b.subtract(k.multiply(g.modPow(x, n))).mod(n).modPow(a.add(u.multiply(x)), n)
        assertEquals(big("""B0DC82BA BCF30674 AE450C02 87745E79 90A3381F 63B387AA F271A10D 233861E3 59B48220 F7C4693C
            9AE12B0A 6F67809F 0876E2D0 13800D6C 41BB59B6 D5979B5C 00A172B4 A2A5903A 0BDCAF8A 709585EB
            2AFAFA8F 3499B200 210DCC1F 10EB3394 3CD67FC8 8A2F39A4 BE5BEC4E C0A3212D C346D7E4 74B29EDE
            8A469FFE CA686E5A"""), s)
    }

    /** Same fixed inputs as merlin_bluetooth test_cross_language_vector: identical bytes on both sides. */
    @Test
    fun matchesTheRobot() {
        val salt = ByteArray(16) { it.toByte() }
        // The robot's B for b = 0xB0B and PIN 12345678 (B = k*v + g^b), computed here from the same formula.
        val k = BigInteger(1, Srp.hash("SHA-256", Srp.pad(Srp.N), Srp.pad(Srp.G)))
        val x = BigInteger(1, Srp.hash("SHA-256", salt, Srp.hash("SHA-256", "merlin:12345678".toByteArray())))
        val bPub = k.multiply(Srp.G.modPow(x, Srp.N)).add(Srp.G.modPow(BigInteger.valueOf(0xB0B), Srp.N)).mod(Srp.N)
        val proof = Srp.prove("12345678", salt, Srp.pad(bPub), a = BigInteger.valueOf(0xA11CE))
        assertEquals("87c7ef7a3c047ef4d93a1e7f504c38e13b6e2b0067ada0c3df0f0371d15ae916", hex(proof.m1))
        assertEquals("4cb4b53e291346546feb9247fad37522c132f3559c2d035e09e614bfe6eaea8b", hex(proof.m2))
        assertEquals("7d47a18953b8a606bc8a31324b68b644730802fc376556c688d70c9b111a014b", hex(Srp.hkdf(proof.key, "phone->robot")))
        assertEquals("leu41egdgn37KLtWNsW3C0ayDD8ryNiAmnQgoFW9pw==", SealedSession(proof.key).seal("{\"op\":\"status\"}").second)
    }
}
