// comm_protocol.h
#pragma once
#include <stdint.h>

// ==================== FRAMING ====================
#define PKT_START_BYTE   0xAA
#define PKT_TYPE_STATUS  0x01   // F4 -> S3 : motion state flags
#define PKT_TYPE_CMD     0x02   // S3 -> F4 : motion command

// ==================== MOTION STATE FLAGS (payload byte) ====================
// Multiple bits can be set at once. S3 tests with & masks.
#define ST_FORWARD      (1 << 0)
#define ST_BACKWARD     (1 << 1)
#define ST_TURN_LEFT    (1 << 2)
#define ST_TURN_RIGHT   (1 << 3)
#define ST_OBSTACLE_L   (1 << 4)    // obstacle on left side
#define ST_OBSTACLE_R   (1 << 5)    // obstacle on right side
#define ST_OBSTACLE_F   (1 << 6)    // obstacle at front (continuous)
#define ST_HALTED       (1 << 7)    // fully stopped / emergency

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