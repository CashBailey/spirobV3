#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <functional>

// ---------- IDs (matches spec) ----------
enum : uint8_t {
  CMD_PING           = 0x00,
  CMD_SET_TARGET_U   = 0x01,
  CMD_SET_TARGET_V   = 0x02,
  CMD_SET_TARGET_W   = 0x03,
  CMD_SET_SPEED_U    = 0x04,
  CMD_SET_SPEED_V    = 0x05,
  CMD_SET_SPEED_W    = 0x06,
  CMD_HOLD_ENABLE_U  = 0x07,
  CMD_HOLD_ENABLE_V  = 0x08,
  CMD_HOLD_ENABLE_W  = 0x09,
  CMD_ZERO_HERE_U    = 0x0A,
  CMD_ZERO_HERE_V    = 0x0B,
  CMD_ZERO_HERE_W    = 0x0C,
  CMD_SET_GAINS_U    = 0x0D,
  CMD_SET_GAINS_V    = 0x0E,
  CMD_SET_GAINS_W    = 0x0F,

  TX_STATE_U         = 0x10,
  TX_STATE_V         = 0x11,
  TX_STATE_W         = 0x12,
  TX_ACK             = 0x13,
  TX_ERR             = 0x14
};

// Error codes
enum : uint16_t {
  ERR_CRC_FAIL = 1,
  ERR_UNKNOWN_ID = 2,
  ERR_TIMEOUT = 3,
  ERR_LIMIT = 4
};

namespace Proto {

// Handler signature: (id, data, crc_ok)
using Handler = std::function<void(uint8_t, int16_t, bool)>;

void init(Handler h);
void poll();

// Send helpers
void send_frame(uint8_t id, int16_t data);
void send_state_u(int16_t angle_x10);
void send_state_v(int16_t angle_x10);
void send_state_w(int16_t angle_x10);
void send_ack(uint8_t last_cmd_id);
void send_err(uint16_t code);

// CRC16 (Modbus/IBM, poly 0xA001, init 0xFFFF)
uint16_t crc16(const uint8_t* data, size_t len);

} // namespace Proto
