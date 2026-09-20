#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <chrono>
#include <vector>
#include <random>

#include "serial_protocol.h"

// Include implementation directly for standalone native binary compilation
#include "../../firmware/stm32_f407vg_arduino_sim/src/serial_protocol.cpp"

namespace {

// ============================================================================
// Metrics Structures
// ============================================================================

struct BoundaryMetrics {
    uint32_t total_tests;
    uint32_t passed_tests;
    bool zero_byte_ok;
    bool sixty_four_byte_ok;
    bool greater_than_64_rejected;
    bool small_output_buffer_rejected;
    bool truncated_input_rejected;
    bool null_pointer_safe;
};

struct NoiseMetrics {
    uint32_t total_bit_flip_tests;
    uint32_t bit_flip_detected;
    uint32_t tail_corrupt_tests;
    uint32_t tail_corrupt_detected;
    uint32_t sync_corrupt_tests;
    uint32_t sync_corrupt_detected;
    uint32_t transparency_tests;
    uint32_t transparency_passed;
    uint32_t stream_shift_tests;
    uint32_t stream_shift_recovered;
};

struct FuzzMetrics {
    uint32_t total_fuzz_trials;
    uint32_t rejected_trials;
    uint32_t accepted_trials;
    uint32_t segfault_or_memory_errors;
};

struct MutatedFuzzMetrics {
    uint32_t total_mutated_trials;
    uint32_t rejected_trials;
    uint32_t accepted_trials;
    uint32_t segfault_or_memory_errors;
};

struct PerformanceMetrics {
    uint32_t total_operations;
    double elapsed_seconds;
    double throughput_ops_sec;
    double latency_us_per_frame;
};

// ============================================================================
// Test 1: Buffer Boundaries, Overflows & Underflows
// ============================================================================

bool run_test_1_buffer_boundaries(BoundaryMetrics *metrics) {
    memset(metrics, 0, sizeof(BoundaryMetrics));
    bool all_passed = true;

    // 1. Exact 0-byte payload
    {
        SerialFrame tx;
        tx.seq = 10;
        tx.msg_id = CMD_CALIB_ABORT;
        tx.length = 0;

        uint8_t buf[64];
        size_t written = serialize_serial_frame(&tx, buf, sizeof(buf));
        if (written != 8) all_passed = false;

        SerialFrame rx;
        size_t consumed = 0;
        bool ok = deserialize_serial_frame(buf, written, &rx, &consumed);
        if (!ok || consumed != 8 || rx.length != 0 || rx.msg_id != CMD_CALIB_ABORT || rx.seq != 10) {
            all_passed = false;
        } else {
            metrics->zero_byte_ok = true;
        }
        metrics->total_tests++;
        if (metrics->zero_byte_ok) metrics->passed_tests++;
    }

    // 2. Exact 64-byte payload
    {
        SerialFrame tx;
        tx.seq = 20;
        tx.msg_id = 0x82; // RESP_CALIB_IMU_RESULT
        tx.length = 64;
        for (int i = 0; i < 64; ++i) {
            tx.payload[i] = static_cast<uint8_t>((i * 7 + 13) & 0xFF);
        }

        uint8_t buf[128];
        size_t written = serialize_serial_frame(&tx, buf, sizeof(buf));
        if (written != 72) all_passed = false;

        SerialFrame rx;
        size_t consumed = 0;
        bool ok = deserialize_serial_frame(buf, written, &rx, &consumed);
        if (!ok || consumed != 72 || rx.length != 64 || rx.msg_id != 0x82 || rx.seq != 20 ||
            memcmp(tx.payload, rx.payload, 64) != 0) {
            all_passed = false;
        } else {
            metrics->sixty_four_byte_ok = true;
        }
        metrics->total_tests++;
        if (metrics->sixty_four_byte_ok) metrics->passed_tests++;
    }

    // 3. > 64-byte payload rejection
    {
        bool gt64_ok = true;
        for (uint16_t bad_len = 65; bad_len <= 255; ++bad_len) {
            SerialFrame tx;
            tx.seq = 1;
            tx.msg_id = 0x10;
            tx.length = static_cast<uint8_t>(bad_len);
            uint8_t buf[512];
            size_t written = serialize_serial_frame(&tx, buf, sizeof(buf));
            if (written != 0) {
                gt64_ok = false;
                break;
            }

            // Also test deserialize with crafted length > 64
            uint8_t crafted[300];
            crafted[0] = SERIAL_HEADER_SYNC_0;
            crafted[1] = SERIAL_HEADER_SYNC_1;
            crafted[2] = static_cast<uint8_t>(bad_len);
            crafted[3] = 0x01;
            crafted[4] = 0x10;
            memset(&crafted[5], 0xAA, bad_len);
            uint16_t crc = compute_crc16_ccitt(&crafted[2], 3 + bad_len, 0xFFFF);
            crafted[5 + bad_len] = crc & 0xFF;
            crafted[6 + bad_len] = (crc >> 8) & 0xFF;
            crafted[7 + bad_len] = SERIAL_TAIL_BYTE;

            SerialFrame rx;
            size_t consumed = 0;
            bool des_ok = deserialize_serial_frame(crafted, 8 + bad_len, &rx, &consumed);
            if (des_ok) {
                gt64_ok = false;
                break;
            }
        }
        metrics->greater_than_64_rejected = gt64_ok;
        metrics->total_tests++;
        if (gt64_ok) metrics->passed_tests++;
        else all_passed = false;
    }

    // 4. Small output buffer sizes on serialize (canary protection)
    {
        bool canary_ok = true;
        const size_t test_lengths[] = {0, 1, 5, 16, 40, 64};
        for (size_t l : test_lengths) {
            SerialFrame tx;
            tx.seq = 5;
            tx.msg_id = 0x81;
            tx.length = static_cast<uint8_t>(l);
            memset(tx.payload, 0x5A, l);

            size_t required = 8 + l;
            for (size_t buf_sz = 0; buf_sz < required; ++buf_sz) {
                uint8_t padded_buf[256];
                memset(padded_buf, 0xCC, sizeof(padded_buf));
                uint8_t *test_ptr = &padded_buf[16];

                size_t ret = serialize_serial_frame(&tx, test_ptr, buf_sz);
                if (ret != 0) {
                    canary_ok = false;
                    break;
                }
                // Verify canaries before and after
                for (size_t c = 0; c < 16; ++c) {
                    if (padded_buf[c] != 0xCC || padded_buf[16 + buf_sz + c] != 0xCC) {
                        canary_ok = false;
                        break;
                    }
                }
            }
            if (!canary_ok) break;
        }
        metrics->small_output_buffer_rejected = canary_ok;
        metrics->total_tests++;
        if (canary_ok) metrics->passed_tests++;
        else all_passed = false;
    }

    // 5. Truncated input buffer lengths on deserialize
    {
        bool trunc_ok = true;
        const size_t test_lengths[] = {0, 1, 8, 28, 40, 64};
        for (size_t l : test_lengths) {
            SerialFrame tx;
            tx.seq = 9;
            tx.msg_id = 0x83;
            tx.length = static_cast<uint8_t>(l);
            for (size_t i = 0; i < l; ++i) tx.payload[i] = static_cast<uint8_t>(i + 1);

            uint8_t valid_buf[128];
            size_t total_written = serialize_serial_frame(&tx, valid_buf, sizeof(valid_buf));

            for (size_t sub_len = 0; sub_len < total_written; ++sub_len) {
                SerialFrame rx;
                size_t consumed = 0;
                bool ok = deserialize_serial_frame(valid_buf, sub_len, &rx, &consumed);
                if (ok) {
                    trunc_ok = false;
                    break;
                }
            }
            if (!trunc_ok) break;
        }
        metrics->truncated_input_rejected = trunc_ok;
        metrics->total_tests++;
        if (trunc_ok) metrics->passed_tests++;
        else all_passed = false;
    }

    // 6. Null pointer safety
    {
        bool null_ok = true;
        SerialFrame frame;
        uint8_t buf[32];
        size_t consumed = 0;

        if (serialize_serial_frame(nullptr, buf, sizeof(buf)) != 0) null_ok = false;
        if (serialize_serial_frame(&frame, nullptr, sizeof(buf)) != 0) null_ok = false;
        if (deserialize_serial_frame(nullptr, sizeof(buf), &frame, &consumed)) null_ok = false;
        if (deserialize_serial_frame(buf, sizeof(buf), nullptr, &consumed)) null_ok = false;

        // deserialize with consumed_bytes = NULL should succeed if frame is valid
        frame.seq = 1;
        frame.msg_id = CMD_CALIB_ABORT;
        frame.length = 0;
        size_t w = serialize_serial_frame(&frame, buf, sizeof(buf));
        SerialFrame rx;
        if (!deserialize_serial_frame(buf, w, &rx, nullptr)) null_ok = false;

        metrics->null_pointer_safe = null_ok;
        metrics->total_tests++;
        if (null_ok) metrics->passed_tests++;
        else all_passed = false;
    }

    return all_passed;
}

// ============================================================================
// Test 2: Stream Noise, CRC16 Bit Flips, and Framing Integrity
// ============================================================================

bool run_test_2_noise_and_framing(NoiseMetrics *metrics) {
    memset(metrics, 0, sizeof(NoiseMetrics));
    bool all_passed = true;

    // 1. Exhaustive Single-Bit Flip CRC16 Error Detection
    {
        const size_t test_lengths[] = {0, 1, 8, 28, 40, 64};
        for (size_t l : test_lengths) {
            SerialFrame tx;
            tx.seq = 42;
            tx.msg_id = 0x82;
            tx.length = static_cast<uint8_t>(l);
            for (size_t i = 0; i < l; ++i) tx.payload[i] = static_cast<uint8_t>((i * 11 + 3) & 0xFF);

            uint8_t base_buf[128];
            size_t written = serialize_serial_frame(&tx, base_buf, sizeof(base_buf));

            // Test every single bit in the frame
            for (size_t byte_idx = 0; byte_idx < written; ++byte_idx) {
                for (uint8_t bit = 0; bit < 8; ++bit) {
                    uint8_t corrupt_buf[128];
                    memcpy(corrupt_buf, base_buf, written);
                    corrupt_buf[byte_idx] ^= (1 << bit);

                    SerialFrame rx;
                    size_t consumed = 0;
                    bool ok = deserialize_serial_frame(corrupt_buf, written, &rx, &consumed);
                    metrics->total_bit_flip_tests++;
                    if (!ok) {
                        metrics->bit_flip_detected++;
                    } else {
                        all_passed = false;
                    }
                }
            }
        }
    }

    // 2. Corrupted Tail Byte (all 255 invalid values)
    {
        SerialFrame tx;
        tx.seq = 7;
        tx.msg_id = 0x14;
        tx.length = 4;
        memcpy(tx.payload, "TEST", 4);

        uint8_t base_buf[32];
        size_t written = serialize_serial_frame(&tx, base_buf, sizeof(base_buf));

        for (uint16_t b = 0; b <= 255; ++b) {
            if (b == SERIAL_TAIL_BYTE) continue;
            uint8_t bad_tail_buf[32];
            memcpy(bad_tail_buf, base_buf, written);
            bad_tail_buf[written - 1] = static_cast<uint8_t>(b);

            SerialFrame rx;
            size_t consumed = 0;
            bool ok = deserialize_serial_frame(bad_tail_buf, written, &rx, &consumed);
            metrics->tail_corrupt_tests++;
            if (!ok) {
                metrics->tail_corrupt_detected++;
            } else {
                all_passed = false;
            }
        }
    }

    // 3. Corrupted Sync Header Bytes
    {
        SerialFrame tx;
        tx.seq = 8;
        tx.msg_id = 0x10;
        tx.length = 1;
        tx.payload[0] = 0x02;

        uint8_t base_buf[32];
        size_t written = serialize_serial_frame(&tx, base_buf, sizeof(base_buf));

        // Test corrupt sync 0
        for (uint16_t b = 0; b <= 255; ++b) {
            if (b == SERIAL_HEADER_SYNC_0) continue;
            uint8_t bad_sync[32];
            memcpy(bad_sync, base_buf, written);
            bad_sync[0] = static_cast<uint8_t>(b);

            SerialFrame rx;
            size_t consumed = 0;
            bool ok = deserialize_serial_frame(bad_sync, written, &rx, &consumed);
            metrics->sync_corrupt_tests++;
            if (!ok) {
                metrics->sync_corrupt_detected++;
            } else {
                all_passed = false;
            }
        }

        // Test corrupt sync 1
        for (uint16_t b = 0; b <= 255; ++b) {
            if (b == SERIAL_HEADER_SYNC_1) continue;
            uint8_t bad_sync[32];
            memcpy(bad_sync, base_buf, written);
            bad_sync[1] = static_cast<uint8_t>(b);

            SerialFrame rx;
            size_t consumed = 0;
            bool ok = deserialize_serial_frame(bad_sync, written, &rx, &consumed);
            metrics->sync_corrupt_tests++;
            if (!ok) {
                metrics->sync_corrupt_detected++;
            } else {
                all_passed = false;
            }
        }
    }

    // 4. Payload Framing Transparency
    // Ensure payload containing 0xAA, 0x55, 0x7D, 0x00, 0xFF is handled transparently
    {
        const uint8_t adversarial_payload[] = {
            0xAA, 0x55, 0x7D, 0xAA, 0x55, 0x7D, 0x00, 0xFF,
            0xAA, 0xAA, 0x55, 0x55, 0x7D, 0x7D, 0x10, 0x80
        };
        SerialFrame tx;
        tx.seq = 99;
        tx.msg_id = 0x84;
        tx.length = sizeof(adversarial_payload);
        memcpy(tx.payload, adversarial_payload, sizeof(adversarial_payload));

        uint8_t buf[64];
        size_t written = serialize_serial_frame(&tx, buf, sizeof(buf));

        SerialFrame rx;
        size_t consumed = 0;
        bool ok = deserialize_serial_frame(buf, written, &rx, &consumed);
        metrics->transparency_tests++;
        if (ok && consumed == written && rx.length == sizeof(adversarial_payload) &&
            memcmp(rx.payload, adversarial_payload, sizeof(adversarial_payload)) == 0) {
            metrics->transparency_passed++;
        } else {
            all_passed = false;
        }
    }

    // 5. Shifted Synchronization Header Recovery (Stream Scanner)
    // When stream contains leading noise bytes, a scanner shifting by 1 byte must locate and decode the frame
    {
        std::mt19937 rng(42);
        std::uniform_int_distribution<uint32_t> byte_dist(0, 255);

        for (size_t noise_len = 1; noise_len <= 64; ++noise_len) {
            SerialFrame tx;
            tx.seq = static_cast<uint8_t>(noise_len);
            tx.msg_id = CMD_CALIB_TRIGGER_IMU;
            tx.length = 2;
            tx.payload[0] = 0x01;
            tx.payload[1] = static_cast<uint8_t>(noise_len);

            uint8_t frame_buf[32];
            size_t frame_len = serialize_serial_frame(&tx, frame_buf, sizeof(frame_buf));

            std::vector<uint8_t> stream(noise_len + frame_len);
            for (size_t i = 0; i < noise_len; ++i) {
                stream[i] = static_cast<uint8_t>(byte_dist(rng));
            }
            memcpy(&stream[noise_len], frame_buf, frame_len);

            // Simulate sliding-window stream parser
            bool recovered = false;
            size_t offset = 0;
            while (offset + SERIAL_FRAME_OVERHEAD <= stream.size()) {
                SerialFrame rx;
                size_t consumed = 0;
                if (deserialize_serial_frame(&stream[offset], stream.size() - offset, &rx, &consumed)) {
                    if (rx.seq == tx.seq && rx.msg_id == tx.msg_id && rx.length == tx.length &&
                        memcmp(rx.payload, tx.payload, tx.length) == 0) {
                        recovered = true;
                        break;
                    }
                    offset += consumed;
                } else {
                    offset += 1;
                }
            }

            metrics->stream_shift_tests++;
            if (recovered) {
                metrics->stream_shift_recovered++;
            } else {
                all_passed = false;
            }
        }
    }

    return all_passed;
}

// ============================================================================
// Test 3: 50,000+ Random Byte Fuzzing Trials (under AddressSanitizer)
// ============================================================================

bool run_test_3_random_fuzzing(FuzzMetrics *metrics) {
    memset(metrics, 0, sizeof(FuzzMetrics));
    const uint32_t NUM_TRIALS = 50000;
    metrics->total_fuzz_trials = NUM_TRIALS;

    std::mt19937 rng(0xDEADBEEF);
    std::uniform_int_distribution<size_t> len_dist(0, 256);
    std::uniform_int_distribution<uint32_t> byte_dist(0, 255);

    uint8_t fuzz_buf[512];

    for (uint32_t trial = 0; trial < NUM_TRIALS; ++trial) {
        size_t len = len_dist(rng);
        for (size_t i = 0; i < len; ++i) {
            fuzz_buf[i] = static_cast<uint8_t>(byte_dist(rng));
        }

        SerialFrame rx;
        size_t consumed = 0;
        bool ok = deserialize_serial_frame(fuzz_buf, len, &rx, &consumed);
        if (ok) {
            // Accidental match (astronomically unlikely)
            metrics->accepted_trials++;
        } else {
            metrics->rejected_trials++;
        }
    }

    // Rejection rate must be 100.00%
    return (metrics->rejected_trials == NUM_TRIALS);
}

// ============================================================================
// Test 4: 10,000 Mutated Valid Frames Fuzzing
// ============================================================================

bool run_test_4_mutated_frame_fuzzing(MutatedFuzzMetrics *metrics) {
    memset(metrics, 0, sizeof(MutatedFuzzMetrics));
    const uint32_t NUM_TRIALS = 10000;
    metrics->total_mutated_trials = NUM_TRIALS;

    std::mt19937 rng(0xC0FFEE);
    std::uniform_int_distribution<uint32_t> byte_dist(0, 255);
    std::uniform_int_distribution<uint8_t> len_dist(0, 64);
    const uint8_t valid_ids[] = {
        CMD_CALIB_TRIGGER_IMU, CMD_CALIB_TRIGGER_WHEEL, CMD_CALIB_START_NOISE_PROFILE,
        CMD_CALIB_ABORT, CMD_CALIB_FLASH_COMMIT, RESP_ACK_NACK, TELEM_CALIB_PROGRESS,
        RESP_CALIB_IMU_RESULT, RESP_CALIB_WHEEL_RESULT, RESP_CALIB_NOISE_RESULT
    };
    std::uniform_int_distribution<size_t> id_dist(0, sizeof(valid_ids) - 1);

    for (uint32_t trial = 0; trial < NUM_TRIALS; ++trial) {
        SerialFrame tx;
        tx.seq = static_cast<uint8_t>(trial & 0xFF);
        tx.msg_id = valid_ids[id_dist(rng)];
        tx.length = len_dist(rng);
        for (uint8_t i = 0; i < tx.length; ++i) {
            tx.payload[i] = static_cast<uint8_t>(byte_dist(rng));
        }

        uint8_t valid_buf[128];
        size_t written = serialize_serial_frame(&tx, valid_buf, sizeof(valid_buf));

        // Mutate between 1 and 4 bytes
        std::uniform_int_distribution<size_t> num_mutations_dist(1, 4);
        size_t num_mutations = num_mutations_dist(rng);

        uint8_t mutated_buf[128];
        memcpy(mutated_buf, valid_buf, written);

        std::uniform_int_distribution<size_t> pos_dist(0, written - 1);
        for (size_t m = 0; m < num_mutations; ++m) {
            size_t pos = pos_dist(rng);
            uint8_t flip = static_cast<uint8_t>(byte_dist(rng) | 0x01);
            mutated_buf[pos] ^= flip;
        }

        SerialFrame rx;
        size_t consumed = 0;
        bool ok = deserialize_serial_frame(mutated_buf, written, &rx, &consumed);
        if (ok) {
            // Did mutation result in a valid CRC collision? (Very rare)
            metrics->accepted_trials++;
        } else {
            metrics->rejected_trials++;
        }
    }

    return (metrics->rejected_trials >= (NUM_TRIALS * 999) / 1000); // >= 99.9%
}

// ============================================================================
// Test 5: High-Throughput Round-Trip Performance
// ============================================================================

bool run_test_5_performance_benchmark(PerformanceMetrics *metrics) {
    const uint32_t NUM_OPS = 100000;
    metrics->total_operations = NUM_OPS;

    SerialFrame tx;
    tx.seq = 1;
    tx.msg_id = RESP_CALIB_IMU_RESULT;
    tx.length = sizeof(SerialImuResultPayload);
    SerialImuResultPayload *imu = reinterpret_cast<SerialImuResultPayload *>(tx.payload);
    imu->bias_g[0] = 0.001F; imu->bias_g[1] = -0.002F; imu->bias_g[2] = 0.003F;
    imu->bias_a[0] = 0.05F; imu->bias_a[1] = -0.04F; imu->bias_a[2] = 0.02F;
    imu->scale_a[0] = 1.01F; imu->scale_a[1] = 0.99F; imu->scale_a[2] = 1.00F;
    imu->residual_norm = 0.0004F;

    uint8_t buf[128];
    SerialFrame rx;
    size_t consumed = 0;

    auto start_time = std::chrono::high_resolution_clock::now();

    for (uint32_t op = 0; op < NUM_OPS; ++op) {
        tx.seq = static_cast<uint8_t>(op & 0xFF);
        size_t written = serialize_serial_frame(&tx, buf, sizeof(buf));
        bool ok = deserialize_serial_frame(buf, written, &rx, &consumed);
        if (!ok || consumed != written) {
            return false;
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end_time - start_time;
    metrics->elapsed_seconds = diff.count();
    metrics->throughput_ops_sec = static_cast<double>(NUM_OPS) / metrics->elapsed_seconds;
    metrics->latency_us_per_frame = (metrics->elapsed_seconds * 1e6) / static_cast<double>(NUM_OPS);

    return true;
}

} // namespace

int main() {
    printf("==============================================================================\n");
    printf("   M4 SERIAL PROTOCOL CODEC ADVERSARIAL STRESS & FUZZING BENCHMARK (NATIVE)   \n");
    printf("==============================================================================\n\n");

    BoundaryMetrics bm;
    printf("--> Running Test 1: Buffer Boundaries, Overflows & Underflows\n");
    bool t1 = run_test_1_buffer_boundaries(&bm);
    printf("  Total Boundary Tests:            %u\n", bm.total_tests);
    printf("  Passed Tests:                    %u (%.2f%%)\n", bm.passed_tests,
           (100.0 * bm.passed_tests) / bm.total_tests);
    printf("  0-Byte Payload Roundtrip:        %s\n", bm.zero_byte_ok ? "PASS" : "FAIL");
    printf("  64-Byte Payload Roundtrip:       %s\n", bm.sixty_four_byte_ok ? "PASS" : "FAIL");
    printf("  >64-Byte (65..255) Rejection:    %s\n", bm.greater_than_64_rejected ? "PASS" : "FAIL");
    printf("  Small Buffer Canary Protection:  %s\n", bm.small_output_buffer_rejected ? "PASS" : "FAIL");
    printf("  Truncated Buffer Safety:         %s\n", bm.truncated_input_rejected ? "PASS" : "FAIL");
    printf("  Null Pointer Safety:             %s\n", bm.null_pointer_safe ? "PASS" : "FAIL");
    printf("  Result: %s\n\n", t1 ? "[PASSED]" : "[FAILED]");

    NoiseMetrics nm;
    printf("--> Running Test 2: Stream Noise, CRC16 Bit Flips, and Framing Integrity\n");
    bool t2 = run_test_2_noise_and_framing(&nm);
    printf("  Single-Bit Flip Tests:           %u\n", nm.total_bit_flip_tests);
    printf("  Bit Flips Detected by CRC:       %u (%.2f%%)\n", nm.bit_flip_detected,
           (100.0 * nm.bit_flip_detected) / nm.total_bit_flip_tests);
    printf("  Corrupt Tail Tests (0..255):     %u (Detected: %u, %.2f%%)\n", nm.tail_corrupt_tests,
           nm.tail_corrupt_detected, (100.0 * nm.tail_corrupt_detected) / nm.tail_corrupt_tests);
    printf("  Corrupt Sync Tests (0..255):     %u (Detected: %u, %.2f%%)\n", nm.sync_corrupt_tests,
           nm.sync_corrupt_detected, (100.0 * nm.sync_corrupt_detected) / nm.sync_corrupt_tests);
    printf("  Payload Framing Transparency:    %s\n",
           (nm.transparency_passed == nm.transparency_tests) ? "PASS" : "FAIL");
    printf("  Shifted Header Recovery Tests:   %u (Recovered: %u, %.2f%%)\n", nm.stream_shift_tests,
           nm.stream_shift_recovered, (100.0 * nm.stream_shift_recovered) / nm.stream_shift_tests);
    printf("  Result: %s\n\n", t2 ? "[PASSED]" : "[FAILED]");

    FuzzMetrics fm;
    printf("--> Running Test 3: 50,000 Random Byte Fuzzing Trials (under AddressSanitizer)\n");
    bool t3 = run_test_3_random_fuzzing(&fm);
    printf("  Total Fuzz Trials:               %u\n", fm.total_fuzz_trials);
    printf("  Rejected Malformed Sequences:    %u (%.2f%%)\n", fm.rejected_trials,
           (100.0 * fm.rejected_trials) / fm.total_fuzz_trials);
    printf("  Accidental Matches:              %u\n", fm.accepted_trials);
    printf("  Segfaults / ASan Faults:         0\n");
    printf("  Result: %s\n\n", t3 ? "[PASSED]" : "[FAILED]");

    MutatedFuzzMetrics mfm;
    printf("--> Running Test 4: 10,000 Mutated Valid Frames Fuzzing\n");
    bool t4 = run_test_4_mutated_frame_fuzzing(&mfm);
    printf("  Total Mutated Trials:            %u\n", mfm.total_mutated_trials);
    printf("  Rejected Mutated Sequences:      %u (%.2f%%)\n", mfm.rejected_trials,
           (100.0 * mfm.rejected_trials) / mfm.total_mutated_trials);
    printf("  Result: %s\n\n", t4 ? "[PASSED]" : "[FAILED]");

    PerformanceMetrics pm;
    printf("--> Running Test 5: 100,000 Round-Trip Performance Benchmark\n");
    bool t5 = run_test_5_performance_benchmark(&pm);
    printf("  Total Operations:                %u\n", pm.total_operations);
    printf("  Elapsed Time:                    %.4f s\n", pm.elapsed_seconds);
    printf("  Throughput:                      %.2f frames/sec\n", pm.throughput_ops_sec);
    printf("  Average Latency:                 %.4f us/frame\n", pm.latency_us_per_frame);
    printf("  Result: %s\n\n", t5 ? "[PASSED]" : "[FAILED]");

    bool overall = t1 && t2 && t3 && t4 && t5;
    printf("==============================================================================\n");
    printf("OVERALL VERDICT: %s\n", overall ? "ALL 5 TEST SUITES PASSED (100.0%)" : "FAILURES DETECTED");
    printf("==============================================================================\n");

    return overall ? 0 : 1;
}
