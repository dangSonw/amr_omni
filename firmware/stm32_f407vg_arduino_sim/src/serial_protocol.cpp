#include "serial_protocol.h"
#include <string.h>

uint16_t compute_crc16_ccitt(const uint8_t *data, size_t length, uint16_t init_val) {
    uint16_t crc = init_val;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (static_cast<uint16_t>(data[i]) << 8);
        for (uint8_t bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000) {
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF;
            } else {
                crc = (crc << 1) & 0xFFFF;
            }
        }
    }
    return crc;
}

size_t serialize_serial_frame(const SerialFrame *frame, uint8_t *buffer, size_t buffer_size) {
    if (!frame || !buffer) return 0;
    if (frame->length > SERIAL_MAX_PAYLOAD_LEN) return 0;

    size_t total_len = SERIAL_FRAME_OVERHEAD + frame->length;
    if (buffer_size < total_len) return 0;

    buffer[0] = SERIAL_HEADER_SYNC_0;
    buffer[1] = SERIAL_HEADER_SYNC_1;
    buffer[2] = frame->length;
    buffer[3] = frame->seq;
    buffer[4] = frame->msg_id;

    if (frame->length > 0 && frame->payload) {
        memcpy(&buffer[5], frame->payload, frame->length);
    }

    // CRC computed over [Length, Seq, MsgID] + Payload
    uint16_t crc = compute_crc16_ccitt(&buffer[2], 3 + frame->length, 0xFFFF);
    buffer[5 + frame->length] = static_cast<uint8_t>(crc & 0xFF);        // LSB
    buffer[6 + frame->length] = static_cast<uint8_t>((crc >> 8) & 0xFF); // MSB
    buffer[7 + frame->length] = SERIAL_TAIL_BYTE;

    return total_len;
}

bool deserialize_serial_frame(const uint8_t *buffer, size_t buffer_len, SerialFrame *frame, size_t *consumed_bytes) {
    if (!buffer || !frame || buffer_len < SERIAL_FRAME_OVERHEAD) {
        return false;
    }

    if (buffer[0] != SERIAL_HEADER_SYNC_0 || buffer[1] != SERIAL_HEADER_SYNC_1) {
        return false;
    }

    uint8_t len = buffer[2];
    if (len > SERIAL_MAX_PAYLOAD_LEN) {
        return false;
    }

    size_t total_len = SERIAL_FRAME_OVERHEAD + len;
    if (buffer_len < total_len) {
        return false;
    }

    if (buffer[total_len - 1] != SERIAL_TAIL_BYTE) {
        return false;
    }

    uint16_t crc_recv = static_cast<uint16_t>(buffer[5 + len]) |
                        (static_cast<uint16_t>(buffer[6 + len]) << 8);

    uint16_t crc_calc = compute_crc16_ccitt(&buffer[2], 3 + len, 0xFFFF);
    if (crc_recv != crc_calc) {
        return false;
    }

    frame->length = len;
    frame->seq = buffer[3];
    frame->msg_id = buffer[4];
    if (len > 0) {
        memcpy(frame->payload, &buffer[5], len);
    }

    if (consumed_bytes) {
        *consumed_bytes = total_len;
    }
    return true;
}

