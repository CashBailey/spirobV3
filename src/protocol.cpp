#include "protocol.hpp"

namespace {
  constexpr uint8_t SOF1 = 0xAA;
  constexpr uint8_t SOF2 = 0x55;

  Proto::Handler g_handler;

  enum ParserState : uint8_t { WAIT1, WAIT2, ID, D0, D1, C0, C1 };
  ParserState st = WAIT1;
  uint8_t  pid = 0;
  uint8_t  d0  = 0, d1 = 0;
  uint8_t  c0  = 0, c1 = 0;

} // anon

namespace Proto {

void init(Handler h) { g_handler = h; }

uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; ++b) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else         crc = (crc >> 1);
    }
  }
  return crc;
}

static inline void dispatch(uint8_t id, int16_t dat, bool crc_ok) {
  if (g_handler) g_handler(id, dat, crc_ok);
}

void poll() {
  while (Serial.available() > 0) {
    uint8_t bytev = (uint8_t)Serial.read();
    switch (st) {
      case WAIT1: if (bytev == SOF1) st = WAIT2; break;
      case WAIT2: st = (bytev == SOF2) ? ID : WAIT1; break;
      case ID: pid = bytev; st = D0; break;
      case D0: d0 = bytev;  st = D1; break;
      case D1: d1 = bytev;  st = C0; break;
      case C0: c0 = bytev;  st = C1; break;
      case C1: {
        c1 = bytev;
        uint8_t payload[3] = { pid, d0, d1 };
        uint16_t want = (uint16_t)(c0 | (uint16_t(c1) << 8));
        uint16_t got  = crc16(payload, 3);
        int16_t data  = (int16_t)(d0 | (uint16_t(d1) << 8));
        dispatch(pid, data, (want == got));
        st = WAIT1;
        break;
      }
    }
  }
}

void send_frame(uint8_t id, int16_t data) {
  uint8_t p[3] = { id, (uint8_t)(data & 0xFF), (uint8_t)((data >> 8) & 0xFF) };
  uint16_t c = crc16(p, 3);
  uint8_t c0 = (uint8_t)(c & 0xFF), c1 = (uint8_t)((c >> 8) & 0xFF);
  uint8_t f[7] = { 0xAA, 0x55, p[0], p[1], p[2], c0, c1 };
  Serial.write(f, sizeof(f));
}

void send_state_u(int16_t angle_x10) { send_frame(TX_STATE_U, angle_x10); }
void send_state_v(int16_t angle_x10) { send_frame(TX_STATE_V, angle_x10); }
void send_state_w(int16_t angle_x10) { send_frame(TX_STATE_W, angle_x10); }
void send_ack(uint8_t last_id)       { send_frame(TX_ACK, last_id); }
void send_err(uint16_t code)         { send_frame(TX_ERR, (int16_t)code); }

} // namespace Proto
