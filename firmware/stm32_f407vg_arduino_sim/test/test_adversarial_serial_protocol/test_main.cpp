#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "serial_protocol.h"
#include "../../src/serial_protocol.cpp"

void setUp(void) {}
void tearDown(void) {}

namespace {

// Deterministic LCG
uint32_t rng_state = 555555U;

uint32_t rng_next() {
    rng_state = rng_state * 1664525U + 1013904223U;
    return rng_state;
}

// ---------------------------------------------------------------------------
// 1. Framing Structure, Overhead, and Bounds Checks
// ---------------------------------------------------------------------------
void test_adversarial_framing_overhead_and_bounds() {
    // Exact constant verification
    TEST_ASSERT_EQUAL_UINT32(8, SERIAL_FRAME_OVERHEAD);
    TEST_ASSERT_EQUAL_UINT32(64, SERIAL_MAX_PAYLOAD_LEN);
    TEST_ASSERT_EQUAL_HEX8(0xAA, SERIAL_HEADER_SYNC_0);
    TEST_ASSERT_EQUAL_HEX8(0x55, SERIAL_HEADER_SYNC_1);
    TEST_ASSERT_EQUAL_HEX8(0x7D, SERIAL_TAIL_BYTE);

    // Struct sizes with #pragma pack(1)
    TEST_ASSERT_EQUAL_UINT32(8, sizeof(SerialProgressPayload));
    TEST_ASSERT_EQUAL_UINT32(40, sizeof(SerialImuResultPayload));
    TEST_ASSERT_EQUAL_UINT32(28, sizeof(SerialWheelResultPayload));
    TEST_ASSERT_EQUAL_UINT32(16, sizeof(SerialNoiseResultPayload));

    // Null pointer defense
    uint8_t buffer[128];
    SerialFrame frame;
    TEST_ASSERT_EQUAL_UINT32(0, serialize_serial_frame(NULL, buffer, sizeof(buffer)));
    TEST_ASSERT_EQUAL_UINT32(0, serialize_serial_frame(&frame, NULL, sizeof(buffer)));
    TEST_ASSERT_FALSE(deserialize_serial_frame(NULL, sizeof(buffer), &frame, NULL));
    TEST_ASSERT_FALSE(deserialize_serial_frame(buffer, sizeof(buffer), NULL, NULL));

    // Payload length boundary violations during serialization
    frame.seq = 1;
    frame.msg_id = CMD_CALIB_TRIGGER_IMU;

    // Boundary: length = 64 (valid)
    frame.length = 64;
    memset(frame.payload, 0x42, 64);
    size_t written = serialize_serial_frame(&frame, buffer, sizeof(buffer));
    TEST_ASSERT_EQUAL_UINT32(72, written);

    // Boundary: length = 65 (invalid, exceeds SERIAL_MAX_PAYLOAD_LEN)
    frame.length = 65;
    TEST_ASSERT_EQUAL_UINT32(0, serialize_serial_frame(&frame, buffer, sizeof(buffer)));

    // Buffer capacity violations
    frame.length = 10;
    // Buffer size = 17 (needs 18) -> must reject
    TEST_ASSERT_EQUAL_UINT32(0, serialize_serial_frame(&frame, buffer, 17));
    // Buffer size = 18 -> must succeed
    TEST_ASSERT_EQUAL_UINT32(18, serialize_serial_frame(&frame, buffer, 18));
}

// ---------------------------------------------------------------------------
// 2. CRC16-CCITT Exhaustive Error Detection: Bit-Flips & Burst Corruptions
// ---------------------------------------------------------------------------
void test_adversarial_crc16_bit_flip_detection() {
    SerialFrame frame;
    frame.seq = 42;
    frame.msg_id = TELEM_CALIB_PROGRESS;
    frame.length = sizeof(SerialProgressPayload);

    SerialProgressPayload payload;
    payload.calib_type = 1;
    payload.stage = 3;
    payload.progress_percent = 75;
    payload.status_code = 0;
    payload.live_metric = 123.456F;
    memcpy(frame.payload, &payload, sizeof(payload));

    uint8_t valid_buffer[64];
    size_t total_len = serialize_serial_frame(&frame, valid_buffer, sizeof(valid_buffer));
    TEST_ASSERT_EQUAL_UINT32(16, total_len);

    // Verify clean packet deserialization first
    SerialFrame rx_frame;
    size_t consumed = 0;
    TEST_ASSERT_TRUE(deserialize_serial_frame(valid_buffer, total_len, &rx_frame, &consumed));
    TEST_ASSERT_EQUAL_UINT32(total_len, consumed);

    // Exhaustive single-bit flips across every bit of the packet:
    // 16 bytes * 8 bits/byte = 128 bits.
    // Every single 1-bit flip MUST be detected and rejected!
    for (size_t byte_idx = 0; byte_idx < total_len; ++byte_idx) {
        for (uint8_t bit = 0; bit < 8; ++bit) {
            uint8_t corrupt_buffer[64];
            memcpy(corrupt_buffer, valid_buffer, total_len);
            corrupt_buffer[byte_idx] ^= (1U << bit); // flip single bit

            size_t dummy_consumed = 0;
            SerialFrame dummy_frame;
            bool ok = deserialize_serial_frame(corrupt_buffer, total_len, &dummy_frame, &dummy_consumed);
            TEST_ASSERT_FALSE_MESSAGE(ok, "Single-bit flip went undetected!");
        }
    }

    // Exhaustive two-bit adjacent flips: 127 pairs
    for (size_t bit_idx = 0; bit_idx < (total_len * 8 - 1); ++bit_idx) {
        uint8_t corrupt_buffer[64];
        memcpy(corrupt_buffer, valid_buffer, total_len);

        size_t byte1 = bit_idx / 8;
        uint8_t bit1 = bit_idx % 8;
        size_t byte2 = (bit_idx + 1) / 8;
        uint8_t bit2 = (bit_idx + 1) % 8;

        corrupt_buffer[byte1] ^= (1U << bit1);
        corrupt_buffer[byte2] ^= (1U << bit2);

        size_t dummy_consumed = 0;
        SerialFrame dummy_frame;
        bool ok = deserialize_serial_frame(corrupt_buffer, total_len, &dummy_frame, &dummy_consumed);
        TEST_ASSERT_FALSE_MESSAGE(ok, "Adjacent two-bit flip went undetected!");
    }

    // 10,000 Random Corruptions Stress (Monte Carlo)
    uint32_t rejected_corruptions = 0;
    const uint32_t kTrials = 10000;

    for (uint32_t t = 0; t < kTrials; ++t) {
        uint8_t corrupt_buffer[64];
        memcpy(corrupt_buffer, valid_buffer, total_len);

        // Inject 1 to 4 random byte modifications
        uint8_t num_flips = 1 + (rng_next() % 4);
        for (uint8_t f = 0; f < num_flips; ++f) {
            size_t idx = rng_next() % total_len;
            corrupt_buffer[idx] ^= (1U + (rng_next() % 255));
        }

        size_t dummy_consumed = 0;
        SerialFrame dummy_frame;
        if (!deserialize_serial_frame(corrupt_buffer, total_len, &dummy_frame, &dummy_consumed)) {
            rejected_corruptions++;
        }
    }
    // 100% of corruptions rejected
    TEST_ASSERT_EQUAL_UINT32(kTrials, rejected_corruptions);
}

// ---------------------------------------------------------------------------
// 3. Stream Deserialization, Truncation, and Concatenated Packets
// ---------------------------------------------------------------------------
void test_adversarial_stream_concatenation_and_truncation() {
    // Subtest A: Truncated frame buffer lengths (from 0 to total_len - 1)
    SerialFrame f1;
    f1.seq = 10;
    f1.msg_id = RESP_ACK_NACK;
    f1.length = 2;
    f1.payload[0] = 0x10;
    f1.payload[1] = 0x00;

    uint8_t buf1[32];
    size_t len1 = serialize_serial_frame(&f1, buf1, sizeof(buf1));
    TEST_ASSERT_EQUAL_UINT32(10, len1);

    for (size_t truncated_len = 0; truncated_len < len1; ++truncated_len) {
        SerialFrame rx;
        size_t c = 0;
        bool ok = deserialize_serial_frame(buf1, truncated_len, &rx, &c);
        TEST_ASSERT_FALSE(ok);
    }

    // Subtest B: Concatenated multi-frame stream buffer
    // Serialize Frame 1 (10 bytes), Frame 2 (Progress: 16 bytes), Frame 3 (Wheel: 36 bytes)
    SerialFrame f2;
    f2.seq = 11;
    f2.msg_id = TELEM_CALIB_PROGRESS;
    f2.length = 8;
    memset(f2.payload, 0x11, 8);

    SerialFrame f3;
    f3.seq = 12;
    f3.msg_id = RESP_CALIB_WHEEL_RESULT;
    f3.length = sizeof(SerialWheelResultPayload);
    SerialWheelResultPayload wheel_res = {{0.03F, 0.0301F, 0.0299F, 0.03F}, 0.1312F, 0.1312F, 0.001F};
    memcpy(f3.payload, &wheel_res, sizeof(wheel_res));

    uint8_t stream_buf[128];
    size_t off1 = serialize_serial_frame(&f1, stream_buf, sizeof(stream_buf));
    size_t off2 = serialize_serial_frame(&f2, stream_buf + off1, sizeof(stream_buf) - off1);
    size_t off3 = serialize_serial_frame(&f3, stream_buf + off1 + off2, sizeof(stream_buf) - off1 - off2);
    size_t total_stream_len = off1 + off2 + off3;

    // Sequentially parse 3 frames using consumed_bytes
    size_t stream_ptr = 0;
    size_t consumed = 0;

    // Frame 1
    SerialFrame rx1;
    TEST_ASSERT_TRUE(deserialize_serial_frame(stream_buf + stream_ptr, total_stream_len - stream_ptr, &rx1, &consumed));
    TEST_ASSERT_EQUAL_UINT8(10, rx1.seq);
    TEST_ASSERT_EQUAL_UINT8(RESP_ACK_NACK, rx1.msg_id);
    TEST_ASSERT_EQUAL_UINT32(off1, consumed);
    stream_ptr += consumed;

    // Frame 2
    SerialFrame rx2;
    TEST_ASSERT_TRUE(deserialize_serial_frame(stream_buf + stream_ptr, total_stream_len - stream_ptr, &rx2, &consumed));
    TEST_ASSERT_EQUAL_UINT8(11, rx2.seq);
    TEST_ASSERT_EQUAL_UINT8(TELEM_CALIB_PROGRESS, rx2.msg_id);
    TEST_ASSERT_EQUAL_UINT32(off2, consumed);
    stream_ptr += consumed;

    // Frame 3
    SerialFrame rx3;
    TEST_ASSERT_TRUE(deserialize_serial_frame(stream_buf + stream_ptr, total_stream_len - stream_ptr, &rx3, &consumed));
    TEST_ASSERT_EQUAL_UINT8(12, rx3.seq);
    TEST_ASSERT_EQUAL_UINT8(RESP_CALIB_WHEEL_RESULT, rx3.msg_id);
    TEST_ASSERT_EQUAL_UINT32(off3, consumed);
    stream_ptr += consumed;

    TEST_ASSERT_EQUAL_UINT32(total_stream_len, stream_ptr);

    // Subtest C: False sync pattern inside payload
    // A frame whose payload contains 0xAA 0x55 0x10 0x7D ...
    SerialFrame f_deceptive;
    f_deceptive.seq = 99;
    f_deceptive.msg_id = CMD_CALIB_TRIGGER_IMU;
    f_deceptive.length = 6;
    f_deceptive.payload[0] = 0xAA;
    f_deceptive.payload[1] = 0x55;
    f_deceptive.payload[2] = 0x01;
    f_deceptive.payload[3] = 0x02;
    f_deceptive.payload[4] = 0x7D;
    f_deceptive.payload[5] = 0x00;

    uint8_t dec_buf[32];
    size_t dec_len = serialize_serial_frame(&f_deceptive, dec_buf, sizeof(dec_buf));
    TEST_ASSERT_EQUAL_UINT32(14, dec_len);

    SerialFrame rx_dec;
    size_t dec_consumed = 0;
    TEST_ASSERT_TRUE(deserialize_serial_frame(dec_buf, dec_len, &rx_dec, &dec_consumed));
    TEST_ASSERT_EQUAL_UINT8(99, rx_dec.seq);
    TEST_ASSERT_EQUAL_UINT8(CMD_CALIB_TRIGGER_IMU, rx_dec.msg_id);
    TEST_ASSERT_EQUAL_HEX8(0xAA, rx_dec.payload[0]);
    TEST_ASSERT_EQUAL_HEX8(0x55, rx_dec.payload[1]);
    TEST_ASSERT_EQUAL_HEX8(0x7D, rx_dec.payload[4]);
}

// ---------------------------------------------------------------------------
// 4. Typed Payload Round-Trip & Numerical Boundary Stress
// ---------------------------------------------------------------------------
void test_adversarial_typed_payloads_numerical_bounds() {
    uint8_t tx_buf[128];
    uint8_t rx_buf[128];
    SerialFrame tx_frame, rx_frame;
    size_t consumed = 0;

    // Payload 1: SerialProgressPayload with extreme float metrics
    {
        SerialProgressPayload p_in = {1U, 2U, 100U, 0U, -99999.5F};
        tx_frame.seq = 1;
        tx_frame.msg_id = TELEM_CALIB_PROGRESS;
        tx_frame.length = sizeof(p_in);
        memcpy(tx_frame.payload, &p_in, sizeof(p_in));

        size_t len = serialize_serial_frame(&tx_frame, tx_buf, sizeof(tx_buf));
        TEST_ASSERT_TRUE(deserialize_serial_frame(tx_buf, len, &rx_frame, &consumed));

        SerialProgressPayload p_out;
        memcpy(&p_out, rx_frame.payload, sizeof(p_out));
        TEST_ASSERT_EQUAL_UINT8(p_in.calib_type, p_out.calib_type);
        TEST_ASSERT_EQUAL_UINT8(p_in.stage, p_out.stage);
        TEST_ASSERT_EQUAL_UINT8(p_in.progress_percent, p_out.progress_percent);
        TEST_ASSERT_FLOAT_WITHIN(1e-4F, p_in.live_metric, p_out.live_metric);
    }

    // Payload 2: SerialImuResultPayload with realistic and boundary floats
    {
        SerialImuResultPayload imu_in;
        imu_in.bias_g[0] = 0.00123F; imu_in.bias_g[1] = -0.00456F; imu_in.bias_g[2] = 0.00089F;
        imu_in.bias_a[0] = 0.12F;    imu_in.bias_a[1] = -0.25F;    imu_in.bias_a[2] = 0.08F;
        imu_in.scale_a[0] = 0.995F;  imu_in.scale_a[1] = 1.008F;   imu_in.scale_a[2] = 0.991F;
        imu_in.residual_norm = 0.015F;

        tx_frame.seq = 2;
        tx_frame.msg_id = RESP_CALIB_IMU_RESULT;
        tx_frame.length = sizeof(imu_in);
        memcpy(tx_frame.payload, &imu_in, sizeof(imu_in));

        size_t len = serialize_serial_frame(&tx_frame, tx_buf, sizeof(tx_buf));
        TEST_ASSERT_TRUE(deserialize_serial_frame(tx_buf, len, &rx_frame, &consumed));

        SerialImuResultPayload imu_out;
        memcpy(&imu_out, rx_frame.payload, sizeof(imu_out));
        for (int i = 0; i < 3; ++i) {
            TEST_ASSERT_FLOAT_WITHIN(1e-6F, imu_in.bias_g[i], imu_out.bias_g[i]);
            TEST_ASSERT_FLOAT_WITHIN(1e-6F, imu_in.bias_a[i], imu_out.bias_a[i]);
            TEST_ASSERT_FLOAT_WITHIN(1e-6F, imu_in.scale_a[i], imu_out.scale_a[i]);
        }
        TEST_ASSERT_FLOAT_WITHIN(1e-6F, imu_in.residual_norm, imu_out.residual_norm);
    }

    // Payload 3: SerialNoiseResultPayload with subnormal and extreme floats
    {
        SerialNoiseResultPayload noise_in = {1.4e-4F, 1.5e-5F, 1.9e-3F, 2.1e-4F};
        tx_frame.seq = 3;
        tx_frame.msg_id = RESP_CALIB_NOISE_RESULT;
        tx_frame.length = sizeof(noise_in);
        memcpy(tx_frame.payload, &noise_in, sizeof(noise_in));

        size_t len = serialize_serial_frame(&tx_frame, tx_buf, sizeof(tx_buf));
        TEST_ASSERT_TRUE(deserialize_serial_frame(tx_buf, len, &rx_frame, &consumed));

        SerialNoiseResultPayload noise_out;
        memcpy(&noise_out, rx_frame.payload, sizeof(noise_out));
        TEST_ASSERT_FLOAT_WITHIN(1e-8F, noise_in.ng, noise_out.ng);
        TEST_ASSERT_FLOAT_WITHIN(1e-8F, noise_in.kg, noise_out.kg);
        TEST_ASSERT_FLOAT_WITHIN(1e-8F, noise_in.na, noise_out.na);
        TEST_ASSERT_FLOAT_WITHIN(1e-8F, noise_in.ka, noise_out.ka);
    }
}

} // namespace

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_adversarial_framing_overhead_and_bounds);
    RUN_TEST(test_adversarial_crc16_bit_flip_detection);
    RUN_TEST(test_adversarial_stream_concatenation_and_truncation);
    RUN_TEST(test_adversarial_typed_payloads_numerical_bounds);
    return UNITY_END();
}
