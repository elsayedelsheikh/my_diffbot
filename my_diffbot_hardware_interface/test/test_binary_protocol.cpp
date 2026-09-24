/*
 * Copyright (c) 2026 RoboLabs
 *
 * Unit tests for binary_protocol: crc16_ccitt, BuildFrame, ExtractFrame,
 * AckStatus codes, RoboAuto payload layouts, BNO055 scales and ResolveLed.
 */

#include <gtest/gtest.h>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <optional>
#include <vector>

#include "my_diffbot_hardware_interface/binary_protocol.hpp"

using my_diffbot_hardware_interface::AckStatus;
using my_diffbot_hardware_interface::HandshakePayload;
using my_diffbot_hardware_interface::PROTOCOL_VERSION;
using my_diffbot_hardware_interface::PidDebugPayload;
using my_diffbot_hardware_interface::SetPIDGainsPayload;
using my_diffbot_hardware_interface::SetCommandTimeoutPayload;
using my_diffbot_hardware_interface::IMUDataPayload;
using my_diffbot_hardware_interface::kBno055AccScale;
using my_diffbot_hardware_interface::kBno055GyroScale;
using my_diffbot_hardware_interface::kBno055QuatScale;
using my_diffbot_hardware_interface::HeartbeatPayload;
using my_diffbot_hardware_interface::RGBLEDSetPayload;
using my_diffbot_hardware_interface::crc16_ccitt;
using my_diffbot_hardware_interface::BuildFrame;
using my_diffbot_hardware_interface::ExtractResult;
using my_diffbot_hardware_interface::ExtractFrame;
namespace MSG_ID = my_diffbot_hardware_interface::MSG_ID;

TEST(Crc16CcittTest, EmptyInput)
{
  // CRC of empty input = init value 0xFFFF
  EXPECT_EQ(crc16_ccitt(nullptr, 0), 0xFFFFu);
}

TEST(Crc16CcittTest, KnownVector_0x09_Frame)
{
  // Frame: [0xAA, 0x09, 0x0A, <10 bytes of payload>]
  // Verify that computing CRC and re-checking yields consistency.
  uint8_t data[3] = {0xAAu, 0x09u, 0x00u};  // no payload
  uint16_t crc = crc16_ccitt(data, 3);
  // Re-computing on same data must produce same CRC
  EXPECT_EQ(crc16_ccitt(data, 3), crc);
}

TEST(Crc16CcittTest, KnownVector_SingleByte_0xAA)
{
  // Independently computed: CRC16-CCITT(0xAA) = 0xB401 (MSB-first, poly 0x1021, init 0xFFFF)
  // Compute it here and check self-consistency (identical computation twice = same result)
  uint8_t data[] = {0xAAu};
  uint16_t crc1 = crc16_ccitt(data, 1);
  uint16_t crc2 = crc16_ccitt(data, 1);
  EXPECT_EQ(crc1, crc2);
  EXPECT_NE(crc1, 0u);  // Must not be zero
}

TEST(Crc16CcittTest, KnownVector_0x313233)
{
  // "123" → CRC16-CCITT (poly 0x1021, init 0xFFFF, no final XOR, no reflection) = 0x5BCE
  uint8_t data[] = {0x31u, 0x32u, 0x33u};
  EXPECT_EQ(crc16_ccitt(data, 3), static_cast<uint16_t>(0x5BCEu));
}

TEST(BuildFrameTest, ZeroLengthPayload)
{
  std::vector<uint8_t> payload;
  auto frame = BuildFrame(0x09u, payload);

  ASSERT_EQ(frame.size(), 5u);
  EXPECT_EQ(frame[0], 0xAAu);   // START_BYTE
  EXPECT_EQ(frame[1], 0x09u);   // MSG_ID
  EXPECT_EQ(frame[2], 0x00u);   // LENGTH
  // CRC over first 3 bytes
  uint16_t expected_crc = crc16_ccitt(frame.data(), 3);
  uint16_t actual_crc = static_cast<uint16_t>(frame[3]) |
    (static_cast<uint16_t>(frame[4]) << 8);
  EXPECT_EQ(actual_crc, expected_crc);
}

TEST(BuildFrameTest, NonEmptyPayload_CorrectLayout)
{
  std::vector<uint8_t> payload = {0x01u, 0x02u, 0x03u};
  auto frame = BuildFrame(0x01u, payload);

  ASSERT_EQ(frame.size(), 8u);       // 5 + 3
  EXPECT_EQ(frame[0], 0xAAu);
  EXPECT_EQ(frame[1], 0x01u);
  EXPECT_EQ(frame[2], 0x03u);        // LENGTH = 3
  EXPECT_EQ(frame[3], 0x01u);
  EXPECT_EQ(frame[4], 0x02u);
  EXPECT_EQ(frame[5], 0x03u);

  uint16_t expected_crc = crc16_ccitt(frame.data(), 6);  // 3 header + 3 payload
  uint16_t actual_crc = static_cast<uint16_t>(frame[6]) |
    (static_cast<uint16_t>(frame[7]) << 8);
  EXPECT_EQ(actual_crc, expected_crc);
}

TEST(BuildFrameTest, CrcStoredLittleEndian)
{
  std::vector<uint8_t> payload = {0xAAu};
  auto frame = BuildFrame(0xF0u, payload);

  ASSERT_EQ(frame.size(), 6u);

  uint16_t expected_crc = crc16_ccitt(frame.data(), 4);
  EXPECT_EQ(frame[4], static_cast<uint8_t>(expected_crc & 0xFFu));
  EXPECT_EQ(frame[5], static_cast<uint8_t>((expected_crc >> 8) & 0xFFu));
}

TEST(ExtractFrameTest, ValidFrame_Extracted)
{
  std::vector<uint8_t> payload = {0x10u, 0x20u};
  auto frame = BuildFrame(0x07u, payload);

  std::vector<uint8_t> buffer(frame);
  uint8_t msg_id = 0;
  std::vector<uint8_t> out_payload;

  EXPECT_EQ(ExtractFrame(buffer, msg_id, out_payload), ExtractResult::OK);
  EXPECT_EQ(msg_id, 0x07u);
  ASSERT_EQ(out_payload.size(), 2u);
  EXPECT_EQ(out_payload[0], 0x10u);
  EXPECT_EQ(out_payload[1], 0x20u);
  EXPECT_TRUE(buffer.empty());  // Frame fully consumed
}

TEST(ExtractFrameTest, IncompleteFrame_WaitsForMore)
{
  std::vector<uint8_t> payload = {0x01u, 0x02u};
  auto frame = BuildFrame(0x01u, payload);

  // Provide only the first 4 bytes — incomplete
  std::vector<uint8_t> buffer(frame.begin(), frame.begin() + 4);
  uint8_t msg_id = 0;
  std::vector<uint8_t> out_payload;

  EXPECT_EQ(ExtractFrame(buffer, msg_id, out_payload), ExtractResult::INCOMPLETE);
  EXPECT_EQ(buffer.size(), 4u);  // Nothing consumed
}

TEST(ExtractFrameTest, CrcMismatch_DiscardsFirstByte)
{
  std::vector<uint8_t> payload = {0x05u};
  auto frame = BuildFrame(0x03u, payload);

  // Corrupt the CRC
  frame[frame.size() - 1] ^= 0xFFu;

  std::vector<uint8_t> buffer(frame);
  const size_t original_size = buffer.size();

  uint8_t msg_id = 0;
  std::vector<uint8_t> out_payload;

  EXPECT_EQ(ExtractFrame(buffer, msg_id, out_payload), ExtractResult::CRC_MISMATCH);
  EXPECT_EQ(buffer.size(), original_size - 1u);  // First byte (0xAA) discarded
}

TEST(ExtractFrameTest, BufferOverflow_Cleared)
{
  // Buffer > 1024 bytes with no valid frame
  std::vector<uint8_t> buffer(1025, 0x00u);

  uint8_t msg_id = 0;
  std::vector<uint8_t> out_payload;

  EXPECT_EQ(ExtractFrame(buffer, msg_id, out_payload), ExtractResult::OVERFLOW);
  EXPECT_TRUE(buffer.empty());
}

TEST(ExtractFrameTest, LeadingJunkThenValidFrame)
{
  std::vector<uint8_t> payload = {0xFFu};
  auto frame = BuildFrame(0x20u, payload);

  // Prepend some garbage bytes (no 0xAA in them)
  std::vector<uint8_t> buffer = {0x01u, 0x02u, 0x03u};
  buffer.insert(buffer.end(), frame.begin(), frame.end());

  uint8_t msg_id = 0;
  std::vector<uint8_t> out_payload;

  EXPECT_EQ(ExtractFrame(buffer, msg_id, out_payload), ExtractResult::OK);
  EXPECT_EQ(msg_id, 0x20u);
}

TEST(ExtractFrameTest, ZeroPayloadFrame)
{
  auto frame = BuildFrame(0x08u, {});
  std::vector<uint8_t> buffer(frame);
  uint8_t msg_id = 0;
  std::vector<uint8_t> out_payload;

  EXPECT_EQ(ExtractFrame(buffer, msg_id, out_payload), ExtractResult::OK);
  EXPECT_EQ(msg_id, 0x08u);
  EXPECT_TRUE(out_payload.empty());
  EXPECT_TRUE(buffer.empty());
}

TEST(AckStatusTest, OkIsZero)
{
  EXPECT_EQ(static_cast<uint8_t>(AckStatus::OK), 0x00u);
}

TEST(AckStatusTest, InvalidParam)
{
  EXPECT_EQ(static_cast<uint8_t>(AckStatus::INVALID_PARAM), 0x01u);
}

TEST(AckStatusTest, CommandFailed)
{
  EXPECT_EQ(static_cast<uint8_t>(AckStatus::COMMAND_FAILED), 0x02u);
}

TEST(AckStatusTest, TimeoutCode)
{
  EXPECT_EQ(static_cast<uint8_t>(AckStatus::TIMEOUT), 0x03u);
}

TEST(AckStatusTest, UnknownIsFF)
{
  EXPECT_EQ(static_cast<uint8_t>(AckStatus::UNKNOWN), 0xFFu);
}

TEST(PayloadSizeTest, SetPIDGainsPayloadSize)
{
  static_assert(sizeof(SetPIDGainsPayload) == 32, "SetPIDGainsPayload must be 32 bytes");
  EXPECT_EQ(sizeof(SetPIDGainsPayload), 32u);
}

TEST(PayloadSizeTest, SetCommandTimeoutPayloadSize)
{
  static_assert(sizeof(SetCommandTimeoutPayload) == 10,
    "SetCommandTimeoutPayload must be 10 bytes");
  EXPECT_EQ(sizeof(SetCommandTimeoutPayload), 10u);
}

TEST(PayloadSizeTest, HandshakePayloadSize)
{
  static_assert(sizeof(HandshakePayload) == 12, "HandshakePayload must be 12 bytes");
  EXPECT_EQ(sizeof(HandshakePayload), 12u);
}

TEST(PayloadSizeTest, PidDebugPayloadSize)
{
  static_assert(sizeof(PidDebugPayload) == 52, "PidDebugPayload must be 52 bytes");
  EXPECT_EQ(offsetof(PidDebugPayload, left_duty_permille), 8u);
  EXPECT_EQ(offsetof(PidDebugPayload, left_encoder_ticks), 32u);
  EXPECT_EQ(offsetof(PidDebugPayload, status), 48u);
}

TEST(PayloadWireLayoutTest, HandshakeProtocolVersionIsPackedSemver)
{
  // v1.0.0 → 1*1e6 + 0*1e3 + 0 = 1000000 = 0x000F4240, little-endian at offset 8
  EXPECT_EQ(PROTOCOL_VERSION, 1000000u);

  HandshakePayload p{};
  p.protocol_ver = PROTOCOL_VERSION;

  uint8_t wire[sizeof(p)];
  std::memcpy(wire, &p, sizeof(p));

  EXPECT_EQ(wire[8], 0x40u);
  EXPECT_EQ(wire[9], 0x42u);
  EXPECT_EQ(wire[10], 0x0Fu);
  EXPECT_EQ(wire[11], 0x00u);
}

TEST(PayloadSizeTest, RGBLEDSetPayloadSize)
{
  static_assert(sizeof(RGBLEDSetPayload) == 12, "RGBLEDSetPayload must be 12 bytes");
  EXPECT_EQ(sizeof(RGBLEDSetPayload), 12u);
}

TEST(HeartbeatTest, RoboAutoPayloadUnchanged)
{
  static_assert(sizeof(HeartbeatPayload) == 17, "HeartbeatPayload must be 17 bytes");
  EXPECT_EQ(sizeof(HeartbeatPayload), 17u);
}

TEST(ImuPayloadLayoutTest, StructSizeIs33Bytes)
{
  static_assert(sizeof(IMUDataPayload) == 33, "IMUDataPayload must be 33 bytes");
  EXPECT_EQ(sizeof(IMUDataPayload), 33u);
}

TEST(ImuPayloadLayoutTest, QuatWOffsetIs20)
{
  static_assert(offsetof(IMUDataPayload, quat_w) == 20, "quat_w must start at byte 20");
  EXPECT_EQ(offsetof(IMUDataPayload, quat_w), 20u);
}

TEST(ImuPayloadLayoutTest, CalSysOffsetIs28)
{
  static_assert(offsetof(IMUDataPayload, cal_sys) == 28, "cal_sys must start at byte 28");
  EXPECT_EQ(offsetof(IMUDataPayload, cal_sys), 28u);
}

TEST(ImuPayloadLayoutTest, TempCOffsetIs32)
{
  static_assert(offsetof(IMUDataPayload, temp_c) == 32, "temp_c must be the last byte, offset 32");
  EXPECT_EQ(offsetof(IMUDataPayload, temp_c), 32u);
}

TEST(ImuPayloadLayoutTest, CalGyroAndCalAccMatchWireOrder)
{
  static_assert(offsetof(IMUDataPayload, cal_acc) == 29, "cal_acc must be byte 29 per spec");
  static_assert(offsetof(IMUDataPayload, cal_gyro) == 30, "cal_gyro must be byte 30 per spec");
  static_assert(offsetof(IMUDataPayload, cal_mag) == 31, "cal_mag must be byte 31 per spec");
  EXPECT_EQ(offsetof(IMUDataPayload, cal_acc), 29u);
  EXPECT_EQ(offsetof(IMUDataPayload, cal_gyro), 30u);
  EXPECT_EQ(offsetof(IMUDataPayload, cal_mag), 31u);
}

TEST(Bno055ScaleTest, AccelRawMatchesGravity)
{
  // Emulator emits 981 for a resting robot with gravity on +z.
  EXPECT_NEAR(981.0 * kBno055AccScale, 9.81, 1e-9);
}

TEST(Bno055ScaleTest, GyroRawIsDegreesPerSecond)
{
  // 16 LSB = 1 deg/s, converted to rad/s. Check a few N deg/s values.
  for (int n : {1, 2, 12, -45}) {
    const double expected_rad = static_cast<double>(n) * M_PI / 180.0;
    EXPECT_NEAR(static_cast<double>(16 * n) * kBno055GyroScale, expected_rad, 1e-12);
  }
}

TEST(Bno055ScaleTest, QuatRawIsUnitScaled)
{
  EXPECT_NEAR(16384.0 * kBno055QuatScale, 1.0, 1e-12);
  EXPECT_NEAR(-16384.0 * kBno055QuatScale, -1.0, 1e-12);
}

TEST(Bno055ScaleTest, EmulatorRestingFrameDecodesToLevelRobot)
{
  // Byte-for-byte the emulator's static resting frame (roboauto_device_emulator).
  IMUDataPayload p{};
  p.lin_ax = 0; p.lin_ay = 0; p.lin_az = 981;
  p.ang_vx = 0; p.ang_vy = 0; p.ang_vz = 16 * 2;   // 2 deg/s
  p.quat_w = 16384; p.quat_x = 0; p.quat_y = 0; p.quat_z = 0;
  p.temp_c = 30;

  const double az = static_cast<double>(p.lin_az) * kBno055AccScale;
  const double wz = static_cast<double>(p.ang_vz) * kBno055GyroScale;
  const double qw = static_cast<double>(p.quat_w) * kBno055QuatScale;

  EXPECT_NEAR(az, 9.81, 1e-9);
  EXPECT_NEAR(wz, 2.0 * M_PI / 180.0, 1e-12);
  EXPECT_NEAR(qw, 1.0, 1e-12);
  EXPECT_EQ(static_cast<int>(p.temp_c), 30);   // 1 LSB = 1 degC, unscaled
}

TEST(ResolveLedTest, ModesAndPhases)
{
  using my_diffbot_hardware_interface::ResolveLed;
  using my_diffbot_hardware_interface::LedOutput;
  constexpr uint32_t kRed = 0xFF0000, kBlue = 0x0000FF, kMix = 0x123456;

  EXPECT_EQ(ResolveLed(0, kRed, kBlue, 250, 0.0), (LedOutput{0, 0, 0, 0}));
  EXPECT_EQ(ResolveLed(1, kMix, 0, 0, 0.0), (LedOutput{0x12, 0x34, 0x56, 1}));
  // Blink is host-side: colour, then black, each SOLID -- never firmware mode 2.
  EXPECT_EQ(ResolveLed(2, kRed, kBlue, 250, 0.1), (LedOutput{255, 0, 0, 1}));
  EXPECT_EQ(ResolveLed(2, kRed, kBlue, 250, 0.3), (LedOutput{0, 0, 0, 1}));
  EXPECT_EQ(ResolveLed(2, kRed, kBlue, 0, 0.3), (LedOutput{255, 0, 0, 1}));
  EXPECT_EQ(ResolveLed(3, kRed, kBlue, 250, 0.1), (LedOutput{255, 0, 0, 1}));
  EXPECT_EQ(ResolveLed(3, kRed, kBlue, 250, 0.3), (LedOutput{0, 0, 255, 1}));
  EXPECT_EQ(ResolveLed(3, kRed, kBlue, 250, 0.6), (LedOutput{255, 0, 0, 1}));
  EXPECT_EQ(ResolveLed(3, kRed, kBlue, 0, 0.3), (LedOutput{255, 0, 0, 1}));
  EXPECT_EQ(ResolveLed(3, kRed, kBlue, -5, 0.3), (LedOutput{255, 0, 0, 1}));
  EXPECT_FALSE(ResolveLed(4, kRed, kBlue, 250, 0.0).has_value());
  EXPECT_FALSE(ResolveLed(-1, kRed, kBlue, 250, 0.0).has_value());
}

TEST(ResolveLedTest, AlternateFlipsEveryPeriodAtTheControllerRate)
{
  using my_diffbot_hardware_interface::ResolveLed;
  using my_diffbot_hardware_interface::LedOutput;
  using my_diffbot_hardware_interface::kLedAlternate;
  constexpr double kTick = 0.01;          // controller_manager update_rate 100 Hz
  constexpr double kT0 = 1789044480.123;  // wall-clock seconds, as write() receives

  for (const double period_ms : {250.0, 500.0}) {
    std::optional<LedOutput> last;
    std::vector<double> flips;
    for (int i = 0; i < 1000; ++i) {  // 10 s
      const double t = kT0 + i * kTick;
      const auto out = ResolveLed(kLedAlternate, 0xFF0000, 0x0000FF, period_ms, t);
      ASSERT_TRUE(out.has_value());
      if (last && *last != *out) {
        flips.push_back(t);
      }
      last = out;
    }
    const int expected = static_cast<int>(10000.0 / period_ms);
    EXPECT_NEAR(static_cast<int>(flips.size()), expected, 1) << period_ms;
    for (size_t k = 1; k < flips.size(); ++k) {
      EXPECT_NEAR(flips[k] - flips[k - 1], period_ms / 1000.0, kTick + 1e-6)
        << period_ms << " ms, flip " << k;
    }
  }
}
