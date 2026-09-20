#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define SERIAL_HEADER_SYNC_0 0xAA
#define SERIAL_HEADER_SYNC_1 0x55
#define SERIAL_TAIL_BYTE     0x7D

#define SERIAL_MAX_PAYLOAD_LEN 64
#define SERIAL_FRAME_OVERHEAD   8  // Header(2) + Len(1) + Seq(1) + MsgID(1) + CRC(2) + Tail(1)

// Message IDs
enum SerialMsgId : uint8_t {
    CMD_CALIB_TRIGGER_IMU        = 0x10,
    CMD_CALIB_TRIGGER_WHEEL      = 0x11,
    CMD_CALIB_START_NOISE_PROFILE= 0x12,
    CMD_CALIB_ABORT              = 0x13,
    CMD_CALIB_FLASH_COMMIT       = 0x14,

    RESP_ACK_NACK                = 0x80,
    TELEM_CALIB_PROGRESS         = 0x81,
    RESP_CALIB_IMU_RESULT        = 0x82,
    RESP_CALIB_WHEEL_RESULT      = 0x83,
    RESP_CALIB_NOISE_RESULT      = 0x84
};

#pragma pack(push, 1)

struct SerialProgressPayload {
    uint8_t calib_type;
    uint8_t stage;
    uint8_t progress_percent;
    uint8_t status_code;
    float live_metric;
};

struct SerialImuResultPayload {
    float bias_g[3];
    float bias_a[3];
    float scale_a[3];
    float residual_norm;
};

struct SerialWheelResultPayload {
    float radii[4];
    float leff;
    float weff;
    float residual_err;
};

struct SerialNoiseResultPayload {
    float ng;
    float kg;
    float na;
    float ka;
};

#pragma pack(pop)

struct SerialFrame {
    uint8_t seq;
    uint8_t msg_id;
    uint8_t length;
    uint8_t payload[SERIAL_MAX_PAYLOAD_LEN];
};

#ifdef __cplusplus
extern "C" {
#endif

uint16_t compute_crc16_ccitt(const uint8_t *data, size_t length, uint16_t init_val);
size_t serialize_serial_frame(const SerialFrame *frame, uint8_t *buffer, size_t buffer_size);
bool deserialize_serial_frame(const uint8_t *buffer, size_t buffer_len, SerialFrame *frame, size_t *consumed_bytes);

#ifdef __cplusplus
}
#endif

#endif // SERIAL_PROTOCOL_H

