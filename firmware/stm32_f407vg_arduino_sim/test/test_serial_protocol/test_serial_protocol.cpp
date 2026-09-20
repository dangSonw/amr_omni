#include <stdint.h>
#include <string.h>
#include <unity.h>

#include "serial_protocol.h"
#include "../../src/serial_protocol.cpp"

void setUp(void) {}
void tearDown(void) {}

namespace {

void test_crc16_standard_ccitt_vector() {
    // Standard CCITT test vector: "123456789" -> 0x29B1 with init 0xFFFF
    const uint8_t test_data[] = "123456789";
    uint16_t crc = compute_crc16_ccitt(test_data, 9, 0xFFFF);
    TEST_ASSERT_EQUAL_HEX16(0x29B1, crc);
}

void test_serial_frame_overhead_constant() {
    TEST_ASSERT_EQUAL_UINT32(8, SERIAL_FRAME_OVERHEAD);
    TEST_ASSERT_EQUAL_UINT32(8, sizeof(SerialProgressPayload));
    TEST_ASSERT_EQUAL_UINT32(40, sizeof(SerialImuResultPayload));
    TEST_ASSERT_EQUAL_UINT32(28, sizeof(SerialWheelResultPayload));
    TEST_ASSERT_EQUAL_UINT32(16, sizeof(SerialNoiseResultPayload));
}

void test_serialize_empty_payload() {
    SerialFrame tx_frame;
    tx_frame.seq = 42;
    tx_frame.msg_id = CMD_CALIB_ABORT;
    tx_frame.length = 0;

    uint8_t buffer[64];
    size_t written = serialize_serial_frame(&tx_frame, buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL_UINT32(8, written);
    TEST_ASSERT_EQUAL_HEX8(SERIAL_HEADER_SYNC_0, buffer[0]);
    TEST_ASSERT_EQUAL_HEX8(SERIAL_HEADER_SYNC_1, buffer[1]);
    TEST_ASSERT_EQUAL_UINT8(0, buffer[2]);
    TEST_ASSERT_EQUAL_UINT8(42, buffer[3]);
    TEST_ASSERT_EQUAL_HEX8(CMD_CALIB_ABORT, buffer[4]);
    TEST_ASSERT_EQUAL_HEX8(SERIAL_TAIL_BYTE, buffer[7]);

    SerialFrame rx_frame;
    size_t consumed = 0;
    bool ok = deserialize_serial_frame(buffer, written, &rx_frame, &consumed);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT32(8, consumed);
    TEST_ASSERT_EQUAL_UINT8(42, rx_frame.seq);
    TEST_ASSERT_EQUAL_HEX8(CMD_CALIB_ABORT, rx_frame.msg_id);
    TEST_ASSERT_EQUAL_UINT8(0, rx_frame.length);
}

void test_serialize_max_payload() {
    SerialFrame tx_frame;
    tx_frame.seq = 99;
    tx_frame.msg_id = 0x55;
    tx_frame.length = SERIAL_MAX_PAYLOAD_LEN;
    for (uint8_t i = 0; i < SERIAL_MAX_PAYLOAD_LEN; ++i) {
        tx_frame.payload[i] = static_cast<uint8_t>(i * 3 + 1);
    }

    uint8_t buffer[128];
    size_t written = serialize_serial_frame(&tx_frame, buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL_UINT32(8 + SERIAL_MAX_PAYLOAD_LEN, written);
    TEST_ASSERT_EQUAL_HEX8(SERIAL_TAIL_BYTE, buffer[written - 1]);

    SerialFrame rx_frame;
    size_t consumed = 0;
    bool ok = deserialize_serial_frame(buffer, written, &rx_frame, &consumed);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT32(written, consumed);
    TEST_ASSERT_EQUAL_UINT8(99, rx_frame.seq);
    TEST_ASSERT_EQUAL_HEX8(0x55, rx_frame.msg_id);
    TEST_ASSERT_EQUAL_UINT8(SERIAL_MAX_PAYLOAD_LEN, rx_frame.length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(tx_frame.payload, rx_frame.payload, SERIAL_MAX_PAYLOAD_LEN);
}

void test_serialize_bounds_checking() {
    SerialFrame frame;
    frame.seq = 1;
    frame.msg_id = CMD_CALIB_FLASH_COMMIT;
    frame.length = 0;

    uint8_t small_buf[7];
    // Required size is 8; buffer size 7 must be rejected
    TEST_ASSERT_EQUAL_UINT32(0, serialize_serial_frame(&frame, small_buf, 7));

    // Null pointers must be rejected
    TEST_ASSERT_EQUAL_UINT32(0, serialize_serial_frame(NULL, small_buf, 7));
    TEST_ASSERT_EQUAL_UINT32(0, serialize_serial_frame(&frame, NULL, 64));

    // Payload length > 64 must be rejected
    frame.length = 65;
    uint8_t normal_buf[128];
    TEST_ASSERT_EQUAL_UINT32(0, serialize_serial_frame(&frame, normal_buf, sizeof(normal_buf)));
}

void test_deserialize_corrupt_frame_rejection() {
    SerialFrame tx_frame;
    tx_frame.seq = 10;
    tx_frame.msg_id = CMD_CALIB_TRIGGER_IMU;
    tx_frame.length = 1;
    tx_frame.payload[0] = 0x01;

    uint8_t valid_buffer[32];
    size_t written = serialize_serial_frame(&tx_frame, valid_buffer, sizeof(valid_buffer));
    TEST_ASSERT_EQUAL_UINT32(9, written);

    SerialFrame rx_frame;
    size_t consumed = 0;

    // 1. Buffer too short
    TEST_ASSERT_FALSE(deserialize_serial_frame(valid_buffer, written - 1, &rx_frame, &consumed));

    // 2. Corrupt header sync 0
    uint8_t bad_sync0[32];
    memcpy(bad_sync0, valid_buffer, written);
    bad_sync0[0] = 0x00;
    TEST_ASSERT_FALSE(deserialize_serial_frame(bad_sync0, written, &rx_frame, &consumed));

    // 3. Corrupt header sync 1
    uint8_t bad_sync1[32];
    memcpy(bad_sync1, valid_buffer, written);
    bad_sync1[1] = 0x00;
    TEST_ASSERT_FALSE(deserialize_serial_frame(bad_sync1, written, &rx_frame, &consumed));

    // 4. Corrupt tail byte
    uint8_t bad_tail[32];
    memcpy(bad_tail, valid_buffer, written);
    bad_tail[written - 1] = 0x00;
    TEST_ASSERT_FALSE(deserialize_serial_frame(bad_tail, written, &rx_frame, &consumed));

    // 5. Corrupt CRC byte
    uint8_t bad_crc[32];
    memcpy(bad_crc, valid_buffer, written);
    bad_crc[written - 3] ^= 0xFF;  // Flip bits in CRC LSB
    TEST_ASSERT_FALSE(deserialize_serial_frame(bad_crc, written, &rx_frame, &consumed));

    // 6. Corrupt payload data
    uint8_t bad_payload[32];
    memcpy(bad_payload, valid_buffer, written);
    bad_payload[5] ^= 0x55;  // Corrupt payload byte
    TEST_ASSERT_FALSE(deserialize_serial_frame(bad_payload, written, &rx_frame, &consumed));
}

void test_roundtrip_all_message_ids() {
    const SerialMsgId msg_ids[] = {
        CMD_CALIB_TRIGGER_IMU,
        CMD_CALIB_TRIGGER_WHEEL,
        CMD_CALIB_START_NOISE_PROFILE,
        CMD_CALIB_ABORT,
        CMD_CALIB_FLASH_COMMIT,
        RESP_ACK_NACK,
        TELEM_CALIB_PROGRESS,
        RESP_CALIB_IMU_RESULT,
        RESP_CALIB_WHEEL_RESULT,
        RESP_CALIB_NOISE_RESULT
    };

    uint8_t buffer[128];
    for (size_t i = 0; i < sizeof(msg_ids) / sizeof(msg_ids[0]); ++i) {
        SerialFrame tx;
        tx.seq = static_cast<uint8_t>(i + 1);
        tx.msg_id = msg_ids[i];
        tx.length = static_cast<uint8_t>((i * 3) % 40);
        for (uint8_t b = 0; b < tx.length; ++b) {
            tx.payload[b] = static_cast<uint8_t>(0xA0 + b);
        }

        size_t written = serialize_serial_frame(&tx, buffer, sizeof(buffer));
        TEST_ASSERT_EQUAL_UINT32(8 + tx.length, written);

        SerialFrame rx;
        size_t consumed = 0;
        bool ok = deserialize_serial_frame(buffer, written, &rx, &consumed);
        TEST_ASSERT_TRUE(ok);
        TEST_ASSERT_EQUAL_UINT32(written, consumed);
        TEST_ASSERT_EQUAL_UINT8(tx.seq, rx.seq);
        TEST_ASSERT_EQUAL_HEX8(tx.msg_id, rx.msg_id);
        TEST_ASSERT_EQUAL_UINT8(tx.length, rx.length);
        if (tx.length > 0) {
            TEST_ASSERT_EQUAL_UINT8_ARRAY(tx.payload, rx.payload, tx.length);
        }
    }
}

void test_typed_payload_progress_roundtrip() {
    SerialProgressPayload prog;
    prog.calib_type = 1;
    prog.stage = 2;
    prog.progress_percent = 75;
    prog.status_code = 0;
    prog.live_metric = 0.0125F;

    SerialFrame tx;
    tx.seq = 7;
    tx.msg_id = TELEM_CALIB_PROGRESS;
    tx.length = sizeof(SerialProgressPayload);
    memcpy(tx.payload, &prog, sizeof(prog));

    uint8_t buffer[64];
    size_t written = serialize_serial_frame(&tx, buffer, sizeof(buffer));
    TEST_ASSERT_EQUAL_UINT32(8 + sizeof(SerialProgressPayload), written);

    SerialFrame rx;
    size_t consumed = 0;
    bool ok = deserialize_serial_frame(buffer, written, &rx, &consumed);
    TEST_ASSERT_TRUE(ok);

    const SerialProgressPayload *rx_prog = reinterpret_cast<const SerialProgressPayload *>(rx.payload);
    TEST_ASSERT_EQUAL_UINT8(1, rx_prog->calib_type);
    TEST_ASSERT_EQUAL_UINT8(2, rx_prog->stage);
    TEST_ASSERT_EQUAL_UINT8(75, rx_prog->progress_percent);
    TEST_ASSERT_EQUAL_UINT8(0, rx_prog->status_code);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.0125F, rx_prog->live_metric);
}

void test_typed_payload_imu_result_roundtrip() {
    SerialImuResultPayload imu;
    imu.bias_g[0] = 0.01F;
    imu.bias_g[1] = -0.02F;
    imu.bias_g[2] = 0.03F;
    imu.bias_a[0] = 0.10F;
    imu.bias_a[1] = -0.15F;
    imu.bias_a[2] = 0.20F;
    imu.scale_a[0] = 1.01F;
    imu.scale_a[1] = 0.99F;
    imu.scale_a[2] = 1.02F;
    imu.residual_norm = 0.005F;

    SerialFrame tx;
    tx.seq = 8;
    tx.msg_id = RESP_CALIB_IMU_RESULT;
    tx.length = sizeof(SerialImuResultPayload);
    memcpy(tx.payload, &imu, sizeof(imu));

    uint8_t buffer[64];
    size_t written = serialize_serial_frame(&tx, buffer, sizeof(buffer));
    TEST_ASSERT_EQUAL_UINT32(8 + sizeof(SerialImuResultPayload), written);

    SerialFrame rx;
    size_t consumed = 0;
    bool ok = deserialize_serial_frame(buffer, written, &rx, &consumed);
    TEST_ASSERT_TRUE(ok);

    const SerialImuResultPayload *rx_imu = reinterpret_cast<const SerialImuResultPayload *>(rx.payload);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.01F, rx_imu->bias_g[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, -0.02F, rx_imu->bias_g[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.03F, rx_imu->bias_g[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.10F, rx_imu->bias_a[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, -0.15F, rx_imu->bias_a[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.20F, rx_imu->bias_a[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 1.01F, rx_imu->scale_a[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.99F, rx_imu->scale_a[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 1.02F, rx_imu->scale_a[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.005F, rx_imu->residual_norm);
}

void test_typed_payload_wheel_result_roundtrip() {
    SerialWheelResultPayload wheel;
    wheel.radii[0] = 0.0301F;
    wheel.radii[1] = 0.0299F;
    wheel.radii[2] = 0.0300F;
    wheel.radii[3] = 0.0302F;
    wheel.leff = 0.1312F;
    wheel.weff = 0.1312F;
    wheel.residual_err = 0.0004F;

    SerialFrame tx;
    tx.seq = 9;
    tx.msg_id = RESP_CALIB_WHEEL_RESULT;
    tx.length = sizeof(SerialWheelResultPayload);
    memcpy(tx.payload, &wheel, sizeof(wheel));

    uint8_t buffer[64];
    size_t written = serialize_serial_frame(&tx, buffer, sizeof(buffer));
    TEST_ASSERT_EQUAL_UINT32(8 + sizeof(SerialWheelResultPayload), written);

    SerialFrame rx;
    size_t consumed = 0;
    bool ok = deserialize_serial_frame(buffer, written, &rx, &consumed);
    TEST_ASSERT_TRUE(ok);

    const SerialWheelResultPayload *rx_wheel = reinterpret_cast<const SerialWheelResultPayload *>(rx.payload);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.0301F, rx_wheel->radii[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.0299F, rx_wheel->radii[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.0300F, rx_wheel->radii[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.0302F, rx_wheel->radii[3]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.1312F, rx_wheel->leff);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.1312F, rx_wheel->weff);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.0004F, rx_wheel->residual_err);
}

void test_typed_payload_noise_result_roundtrip() {
    SerialNoiseResultPayload noise;
    noise.ng = 0.002F;
    noise.kg = 0.0001F;
    noise.na = 0.015F;
    noise.ka = 0.0005F;

    SerialFrame tx;
    tx.seq = 10;
    tx.msg_id = RESP_CALIB_NOISE_RESULT;
    tx.length = sizeof(SerialNoiseResultPayload);
    memcpy(tx.payload, &noise, sizeof(noise));

    uint8_t buffer[64];
    size_t written = serialize_serial_frame(&tx, buffer, sizeof(buffer));
    TEST_ASSERT_EQUAL_UINT32(8 + sizeof(SerialNoiseResultPayload), written);

    SerialFrame rx;
    size_t consumed = 0;
    bool ok = deserialize_serial_frame(buffer, written, &rx, &consumed);
    TEST_ASSERT_TRUE(ok);

    const SerialNoiseResultPayload *rx_noise = reinterpret_cast<const SerialNoiseResultPayload *>(rx.payload);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.002F, rx_noise->ng);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.0001F, rx_noise->kg);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.015F, rx_noise->na);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.0005F, rx_noise->ka);
}

}  // namespace

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_crc16_standard_ccitt_vector);
    RUN_TEST(test_serial_frame_overhead_constant);
    RUN_TEST(test_serialize_empty_payload);
    RUN_TEST(test_serialize_max_payload);
    RUN_TEST(test_serialize_bounds_checking);
    RUN_TEST(test_deserialize_corrupt_frame_rejection);
    RUN_TEST(test_roundtrip_all_message_ids);
    RUN_TEST(test_typed_payload_progress_roundtrip);
    RUN_TEST(test_typed_payload_imu_result_roundtrip);
    RUN_TEST(test_typed_payload_wheel_result_roundtrip);
    RUN_TEST(test_typed_payload_noise_result_roundtrip);
    return UNITY_END();
}
