package com.merlin.app

import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Test

class FoxgloveClientTest {
    @Test
    fun clientMessageFraming() {
        val frame = FoxgloveClient.clientMessage(0x01020304, "{}")
        // opcode, u32 LE channel id, payload
        assertArrayEquals(byteArrayOf(0x01, 0x04, 0x03, 0x02, 0x01, '{'.code.toByte(), '}'.code.toByte()), frame)
    }

    @Test
    fun twistUsesGivenStamp() {
        val json = FoxgloveClient.twistStampedJson(0.35, -1.5, 1_790_000_123_456)
        assert(json.contains(""""stamp":{"sec":1790000123,"nanosec":456000000}"""))
        assert(json.contains(""""linear":{"x":0.35,"""))
        assertEquals(1, Regex(""""z":-1.5""").findAll(json).count())
    }

    @Test
    fun robotClockNeverRunsAhead() {
        val clock = RobotClock()
        assertEquals(null, clock.nowMs(1_000))
        // True offset +5000 ms; transit latencies 40, 8 and 120 ms -> samples 4960, 4992, 4880.
        clock.sample(robotMs = 10_000, phoneMs = 5_040)
        clock.sample(robotMs = 11_000, phoneMs = 6_008)
        clock.sample(robotMs = 12_000, phoneMs = 7_120)
        assertEquals(4_992L + 8_000, clock.nowMs(8_000))  // lowest-latency sample wins, still <= true time
    }

    @Test
    fun parsesBatteryStateCdr() {
        // Captured from the robot's /battery_state_broadcaster/battery_state: 7.53 V, 63.75 %.
        val hex = "00010000d33dc06a8204e7260100000000000000c3f5f0400000c07f0000c07f0000c07f0000c07fe17a844002007f42" +
            "000002010000000000000000030000002c200000030000002c200000"
        val cdr = hex.chunked(2).map { it.toInt(16).toByte() }.toByteArray()
        val (volts, pct) = FoxgloveClient.batteryState(cdr)!!
        assertEquals(7.53f, volts, 0.01f)
        assertEquals(63.75f, pct, 0.01f)
    }

    @Test
    fun parsesUptimeAndBuildsLedCommand() {
        assertEquals(1235L, FoxgloveClient.uint32(byteArrayOf(0, 1, 0, 0, 0xD3.toByte(), 0x04, 0, 0)))
        assertEquals(null, FoxgloveClient.uint32(byteArrayOf(0, 1, 0, 0)))
        // 0x00FF00 = 65280: the README's "green, blinking every 500 ms" example.
        assert(FoxgloveClient.ledJson(2, 0x00FF00, 0, 500).endsWith(""""values":[2.0,65280.0,0.0,500.0]}]}"""))
    }
}
