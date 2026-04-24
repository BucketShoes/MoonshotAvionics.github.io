// commands.h — Command reception, HMAC authentication, and command dispatch.
// Owns the HMAC key, nonce state, and command ack state.
// Changes to command handling, authentication, or log download only touch
// commands.h + commands.cpp.

#ifndef COMMANDS_H
#define COMMANDS_H

#include <Arduino.h>
#include "config.h"

// ===================== COMMAND ACK STATE =====================

struct CommandAck {
  uint32_t nonce;
  uint8_t  result;            // CMD_OK or CMD_ERR_*
  int8_t   rssi;              // dBm of the command packet
  int8_t   snr;               // dB*4 of the command packet
  uint16_t invalidHmacCount;  // running count of HMAC failures
  uint8_t  rxPosInSlot;       // position within current slot when command RX completed, 2ms units (255 = 510ms)
  bool     pending;           // true = force 0x0A page on next TX
};

extern CommandAck lastAck;

// Track when last verified command was received (micros).
extern unsigned long lastValidCmdUs;

// ===================== HMAC / NONCE STATE =====================

extern uint8_t  hmacKey[HMAC_KEY_LEN];
extern bool     hmacKeyValid;
extern uint32_t highestNonce;  // monotonic, persisted to NVS before acting

// ===================== PUBLIC API =====================

// Verify HMAC on a received command packet (last HMAC_TRUNC_LEN bytes = MAC).
bool verifyCommandHMAC(const uint8_t* pkt, size_t pktLen);

// Parse a received LoRa packet: check type, device ID, HMAC, nonce, then dispatch.
void processReceivedPacket(const uint8_t* pkt, size_t pktLen, int8_t rssi, int8_t snr);

// Execute a validated command. Sets lastAck for telemetry ack page.
void executeCommand(uint8_t cmdId, uint32_t nonce, const uint8_t* params, size_t paramsLen);

// Read a little-endian uint32 from a byte buffer.
uint32_t readU32(const uint8_t* buf);

#endif // COMMANDS_H
