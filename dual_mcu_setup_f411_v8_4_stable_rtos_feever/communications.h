// communications.h
#pragma once
#include "comm_protocol.h"


HardwareSerial commSerial(PA3, PA2);
// Each main sketch must define `commSerial` before including this file.
// Example (F4):  #define commSerial Serial2
// Example (S3):  #define commSerial Serial1

// ---- XOR checksum ----
inline uint8_t computeChecksum(const uint8_t* data, uint16_t len) {
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++) sum ^= data[i];
    return sum;
}

// ---- Send a 1-byte-payload packet: [0xAA][TYPE][LEN=1][PAYLOAD][XOR] ----
inline void sendPacket(uint8_t type, uint8_t payload) {
    uint8_t packet[5];
    packet[0] = PKT_START_BYTE;
    packet[1] = type;
    packet[2] = 1;
    packet[3] = payload;
    packet[4] = computeChecksum(packet, 4);
    commSerial.write(packet, 5);
}

// Convenience wrappers
inline void sendStatus(uint8_t status)  { sendPacket(PKT_TYPE_STATUS, status); }
inline void sendCommand(uint8_t command) { sendPacket(PKT_TYPE_CMD,    command); }

// ---- Non-blocking receiver ----
// Call this in a loop. Returns true for each complete, valid packet.
// Parses exactly one packet per call even if multiple are buffered.
inline bool recvPacket(uint8_t* outType, uint8_t* outPayload) {
    static commState state = WAIT_START;
    static uint8_t rxType, rxLen, rxIdx;
    static uint8_t rxBuf[16];
    static uint8_t hdr[3];

    while (commSerial.available()) {
        uint8_t b = commSerial.read();
        switch (state) {
            case WAIT_START:
                if (b == PKT_START_BYTE) { hdr[0] = b; state = WAIT_TYPE; }
                break;

            case WAIT_TYPE:
                rxType = b; hdr[1] = b; state = WAIT_LEN;
                break;

            case WAIT_LEN:
                rxLen = b; hdr[2] = b; rxIdx = 0;
                if (rxLen == 0)                  state = WAIT_CHECKSUM;
                else if (rxLen <= sizeof(rxBuf)) state = WAIT_PAYLOAD;
                else                             state = WAIT_START;   // too big
                break;

            case WAIT_PAYLOAD:
                rxBuf[rxIdx++] = b;
                if (rxIdx >= rxLen) state = WAIT_CHECKSUM;
                break;

            case WAIT_CHECKSUM: {
                uint8_t calc = computeChecksum(hdr, 3);
                for (uint8_t i = 0; i < rxLen; i++) calc ^= rxBuf[i];
                state = WAIT_START;
                if (calc == b && rxLen == 1) {
                    *outType    = rxType;
                    *outPayload = rxBuf[0];
                    return true;
                }
                break;
            }

            default:
                state = WAIT_START;
                break;
        }
    }
    return false;
}