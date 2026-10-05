// comm_protocol.h
#pragma once
#include <stdint.h>

// ==================== FRAMING ====================
#define PKT_START_BYTE   0xAA
#define PKT_TYPE_STATUS  0x01   // F4 -> S3 : wheelchair state
#define PKT_TYPE_CMD     0x02   // S3 -> F4 : motion command

// ==================== STATUS CODES (F4 -> S3) ====================
#define STATUS_IDLE           0x00
#define STATUS_FORWARD        0x01
#define STATUS_BACKWARD       0x02
#define STATUS_TURN_LEFT      0x03
#define STATUS_TURN_RIGHT     0x04
#define STATUS_HALTED         0x05
#define STATUS_OBSTACLE_FL    0x06
#define STATUS_OBSTACLE_FR    0x07
#define STATUS_OBSTACLE_BOTH  0x08
#define STATUS_FAILSAFE       0x09

// ==================== COMMAND CODES (S3 -> F4) ====================
#define CMD_NONE         0x00
#define CMD_FORWARD      0x01
#define CMD_BACKWARD     0x02
#define CMD_TURN_LEFT    0x03
#define CMD_TURN_RIGHT   0x04
#define CMD_STOP         0x05

// ==================== RX STATE MACHINE ====================
enum commState {
    WAIT_START,
    WAIT_TYPE,
    WAIT_LEN,
    WAIT_PAYLOAD,
    WAIT_CHECKSUM
};