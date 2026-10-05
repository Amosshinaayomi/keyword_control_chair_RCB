#pragma once
#include <Arduino.h>

// // --- 1. Configuration ---
#define ENCODER_PIN_1 PB2  // Front left
#define ENCODER_PIN_2 PC15 // Front right
#define ENCODER_PIN_3 PA15 // Back left
#define ENCODER_PIN_4 PB5 // Back right

#define BATTERY_PIN PA0
#define MOTOR_FREQ 20000

#define TOTAL_ENCODERS 4
#define MOTOR_STBY_PIN PB12

#define SDA_PIN PB7
#define SCL_PIN PB6

#define UART_TX PA2
#define UART_RX PA3

#define LED_PIN PC13

#define hapticMotorA PB8
#define hapticMotorB PB9
#define hapticMotorC  PB14
#define hapticMotorD PB15

#define JOYSTICK_Y PA1
#define JOYSTICK_X PA4
#define SWITCH PA5