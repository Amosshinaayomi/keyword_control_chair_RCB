// #pragma once

// #include "PID.h"
// #include "motor_control.h"
// #include <cmath>

// // --- Feedforward data extracted from logs ---
// static const int FF_POINTS = 11;
// static const float ff_voltages[FF_POINTS] = {2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f, 11.0f, 11.5f};
// static const float ff_left_speed_pos[FF_POINTS]  = {  2.0f,  21.0f,  31.0f,  42.0f,  54.0f,  68.0f,  79.0f,  92.0f,  92.0f,  92.0f,  92.0f };
// static const float ff_right_speed_pos[FF_POINTS] = {  0.0f,  14.0f,  28.0f,  39.0f,  51.0f,  62.0f,  72.0f,  83.0f,  83.0f,  82.0f,  82.0f };
// static const float ff_yaw_deg[FF_POINTS] = {  0.56f,  8.70f, 17.87f, 25.25f, 32.15f, 38.54f, 44.48f, 50.25f, 50.33f, 50.08f, 50.20f };
// static const float ff_vdiff[FF_POINTS]   = {  4.0f,   6.0f,   8.0f,  10.0f,  12.0f,  14.0f,  16.0f,  18.0f,  20.0f,  22.0f,  23.0f };

// static inline float lerp(float a, float b, float t) { return a + t * (b - a); }

// static float interpFromTable(const float *xArr, const float *yArr, int n, float x) {
//     if (n <= 0) return 0.0f;
//     if (x <= xArr[0]) return yArr[0];
//     if (x >= xArr[n-1]) return yArr[n-1];
//     for (int i = 0; i < n-1; ++i) {
//         if (x >= xArr[i] && x <= xArr[i+1]) {
//             float t = (x - xArr[i]) / (xArr[i+1] - xArr[i]);
//             return lerp(yArr[i], yArr[i+1], t);
//         }
//     }
//     return yArr[n-1];
// }

// float getFeedforwardVoltageLeft(float desiredSpeed_ticks) {
//     int sign = (desiredSpeed_ticks >= 0.0f) ? 1 : -1;
//     float absSp = fabsf(desiredSpeed_ticks);
//     float v = interpFromTable(ff_left_speed_pos, ff_voltages, FF_POINTS, absSp);
//     return v * sign;
// }

// float getFeedforwardVoltageRight(float desiredSpeed_ticks) {
//     int sign = (desiredSpeed_ticks >= 0.0f) ? 1 : -1;
//     float absSp = fabsf(desiredSpeed_ticks);
//     float v = interpFromTable(ff_right_speed_pos, ff_voltages, FF_POINTS, absSp);
//     return v * sign;
// }

// float getAngularFeedforwardVoltageDiff(float desiredOmega_degps) {
//     float sign = (desiredOmega_degps >= 0.0f) ? 1.0f : -1.0f;
//     float absOmega = fabsf(desiredOmega_degps);
//     float vdiff = interpFromTable(ff_yaw_deg, ff_vdiff, FF_POINTS, absOmega);
//     return sign * vdiff;
// }

// class MotionController {
// public:
//     MotionController()
//         : _pidLeft(KP_VEL, KI_VEL, KD_VEL, CONTROL_DT, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE),
//           _pidRight(KP_VEL, KI_VEL, KD_VEL, CONTROL_DT, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE),
//           _pidOmega(KP_OMEGA, KI_OMEGA, KD_OMEGA, CONTROL_DT, -MAX_OMEGA_RADPS, MAX_OMEGA_RADPS),
//           prevLeft(0.0f), prevRight(0.0f),
//           _lastLeftVoltage(0.0f), _lastRightVoltage(0.0f) {}

//     void setTargetVelocity(float v, float omega) {
//         _targetV = v;
//         _targetOmega = omega;
//         _straightMode = false;
//     }

//     void setStraight(float speed) {
//         _targetV = speed;
//         _targetOmega = 0.0f;
//         _straightMode = true;
//         _headingSetpointValid = false;
//     }



//     // Main control update, to be called at fixed interval (dt seconds)
//     void update(float leftTicksAvg, float rightTicksAvg, float yaw, float yawRate_radps, float dt, float *motorsVoltage) {
//         // 1. Compute wheel speeds (ticks/s)
//         Serial.printf("prevleft tick is %f\n", prevLeft);
//         Serial.printf("prev right tick is %f\n", prevRight);   

//         Serial.printf("absolute yaw is %.5f, degs: %.2f\n", yaw, yaw * RAD_TO_DEG);
//         Serial.printf("yawRate is %.5f\n", yawRate_radps);   

//         float leftTickSpeed = (leftTicksAvg - prevLeft) / dt;
//         float rightTickSpeed = (rightTicksAvg - prevRight) / dt;

//         float leftTickSpeedF = 0.0f, rightTickSpeedF = 0.0f;
//         const int FILTER_WINDOW_SIZE = 3;
//         static float leftTickSpeedBuffer[FILTER_WINDOW_SIZE] = {0};
//         static float rightTickSpeedBuffer[FILTER_WINDOW_SIZE] = {0};

//         static uint8_t idx = 0;
//         leftTickSpeedBuffer[idx] = leftTickSpeed;
//         rightTickSpeedBuffer[idx] = rightTickSpeed;
//         idx = (idx + 1) % FILTER_WINDOW_SIZE;
    

//         for(uint8_t i = 0; i < FILTER_WINDOW_SIZE; i++)
//         {
//             leftTickSpeedF += leftTickSpeedBuffer[i];
//             rightTickSpeedF += rightTickSpeedBuffer[i];
//         }

//         leftTickSpeedF /= FILTER_WINDOW_SIZE;
//         rightTickSpeedF /=FILTER_WINDOW_SIZE;
//         Serial.printf("leftTickSpeedF is %.f\nrightTickSpeedF is %.f\n", leftTickSpeedF, rightTickSpeedF); 
//         prevLeft = leftTicksAvg;
//         prevRight = rightTicksAvg;
        
//         // Outer heading loop (produces desired yaw rate)
//         // float desiredOmega = _targetOmega;
//         // 2. Heading correction if in straight mode
//         static float headingError = 0;
//         if (_straightMode) {
//             if (!_headingSetpointValid) {
//                 _headingSetpoint = yaw;
//                 _headingSetpointValid = true;
//                 Serial.printf("heading is locked at %.3f, %.2f", _headingSetpoint, _headingSetpoint * RAD_TO_DEG); 
//             }
//             headingError = yaw - _headingSetpoint;
//             Serial.printf("set point is %.5f\n", _headingSetpoint);
//             // Normalize to [-π, π] (assuming yaw in radians)
//             headingError = atan2f(sinf(headingError), cosf(headingError));
//             Serial.printf("MAX_OMEGA_RADPS is %.5f\n", MAX_OMEGA_RADPS);
//             // Adjust angular velocity command
//             _targetOmega = KP_HEADING * headingError;
//             Serial.printf("heading error %.5f\n", headingError);            
//         }
//         // Feedforward block
//         float desiredOmega_dps = -(_targetOmega * RAD_TO_DEG);   // to deg/s
//         Serial.printf("desired correction Omega in degs is %.3f\n", desiredOmega_dps);
//         float ff_angular_volt_diff = getAngularFeedforwardVoltageDiff(desiredOmega_dps); 
//         Serial.printf("ffOmegaCorrection is %.2f\n", ff_angular_volt_diff);
//         int8_t sign = (ff_angular_volt_diff >= 0) ? 1 : -1;
//         // Split voltage difference and add to motor commands (direct voltage feedforward)
//         float left_angular_ff = sign * fabs(ff_angular_volt_diff) / 2.0f;
//         float right_angular_ff = -sign * fabs(ff_angular_volt_diff) / 2.0f;
//         Serial.printf("angular_ff voltage correction for left motor is %.3f\n", left_angular_ff);
//         Serial.printf("angular_ff voltage correction for right motor is %.3f\n", right_angular_ff);


//         // Inner angular velocity loop (PID on yaw rate)
//         // Filter out noisy gyro readings
//         const float RESOLUTION_RADPS = DEG_TO_RAD / 0.5;   // 2 deg/s
//         float yawRate_corrected = yawRate_radps;
//         if ((fabs(yawRate_corrected) < RESOLUTION_RADPS))
//         {
//             Serial.println("yawrate corrected");
//             yawRate_corrected = 0.0f;
//         }
//         Serial.printf("corrected yawrate is %.2f\n", yawRate_corrected);


//         float quantised = roundf(yawRate_corrected / RESOLUTION_RADPS) * RESOLUTION_RADPS;
//         Serial.printf("quantized yawRate after clamping is %.3f\n", quantised);
//         // float omegaCorrection = _pidOmega.compute(_targetOmega, quantised);
//         float omegaCorrection = constrain((_pidOmega.compute(_targetOmega, quantised)),  -MAX_OMEGA_RADPS,  MAX_OMEGA_RADPS);

//         Serial.printf("omega correction is %.5f\n", omegaCorrection);
//         // 3. Desired side speeds from kinematics (m/s)
//         float leftDesired_mps = _targetV - omegaCorrection  * (ROBOT_TRACK_WIDTH / 2.0f);
//         float rightDesired_mps = _targetV + omegaCorrection * (ROBOT_TRACK_WIDTH / 2.0f);
//         Serial.printf("leftDesired_mps is %f\n", leftDesired_mps);
//         Serial.printf("rightDesired_mps is %f\n", rightDesired_mps);

//         // 4. Convert to desired ticks/s using encoder resolution
//         const float leftDesired = leftDesired_mps * TICKS_PER_METER;
//         const float rightDesired = rightDesired_mps * TICKS_PER_METER;
//         Serial.printf("leftDesired ticks is %f\n", leftDesired);
//         Serial.printf("rightDesired ticks is %f\n", rightDesired);

//         Serial.printf("current left speed(ticks/sec) is %f\n", leftTickSpeed);
//         Serial.printf("current right speed(ticks/sec) is %f\n", rightTickSpeed);
//         Serial.printf("leftDesired ticks_per_meter is %f\n", leftDesired);
//         Serial.printf("rightDesired ticks_per_meter is %f\n", rightDesired);

//         float leftVelocity_mps = leftTickSpeedF / TICKS_PER_METER;
//         float rightVelocity_mps = rightTickSpeedF / TICKS_PER_METER;
//         float robotVelocity_mps = (leftVelocity_mps + rightVelocity_mps) / 2.0f;
//         Serial.printf("current left velocity(m/s) is %f\n", leftVelocity_mps);
//         Serial.printf("current right velocity(m/s) is %f\n", rightVelocity_mps);
//         Serial.printf("current robot velocity(m/s) is %f\n", robotVelocity_mps);
        

//         // 5. Compute PID outputs (volts)
//         // float leftFF = getFeedforwardVoltage(leftDesired);
//         // float rightFF = getFeedforwardVoltage(rightDesired);

//         const float leftFF = getFeedforwardVoltageLeft(leftDesired);
//         const float rightFF = getFeedforwardVoltageRight(rightDesired);

//         Serial.printf("left FF voltage is %.2f\n", leftFF);
//         Serial.printf("right FF voltage is %.2f\n", rightFF);

//         float leftPIDout = _pidLeft.compute(leftDesired, leftTickSpeedF);
//         float rightPIDout = _pidRight.compute(rightDesired, rightTickSpeedF);
//         Serial.printf("leftPIDout voltage is %.2f\n", leftPIDout);
//         Serial.printf("rightPIDout voltage is %.2f\n", rightPIDout);

//         float leftCmd;
//         float rightCmd;
//         if(fabs(headingError) >= (DEG_TO_RAD * 7)){
//             leftCmd = leftFF + leftPIDout + left_angular_ff;
//             rightCmd = rightFF + rightPIDout + right_angular_ff;   
//             Serial.println("using angular feedforward table");
//         } else {
//             leftCmd = leftFF + leftPIDout;
//             rightCmd = rightFF + rightPIDout;   
//             Serial.println("using normal omega correction");         
//         }

//         Serial.printf("leftCmd before constrain is %f\n", leftCmd);
//         Serial.printf("rightCmd before constrain is %f\n", rightCmd);

//         leftCmd = constrain(leftCmd, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE);
//         rightCmd = constrain(rightCmd, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE);

//         Serial.printf("leftCmd after constrain is %f\n", leftCmd);
//         Serial.printf("rightCmd after constrain is %f\n", rightCmd);

//         if(_targetV == 0 && _targetOmega == 0) {
//             if(fabs(leftCmd) < 2.0 && fabs(rightCmd) < 2.0)
//             {
//                 leftCmd = 0;
//                 rightCmd = 0;
//             }
//         }
//         motorsVoltage[0] = leftCmd;
//         motorsVoltage[1] = rightCmd;

//         // setLeftMotorsVoltage(leftCmd);
//         // setRightMotorsVoltage(rightCmd);
//         // float leftCmd = _pidLeft.compute(leftDesired, leftTickSpeed);
//         // float rightCmd = _pidRight.compute(rightDesired, rightTickSpeed);
//         Serial.printf("leftCmd is %f\n", leftCmd);
//         Serial.printf("rightCmd is%f\n", rightCmd);

//         // // 6. Apply to motors (the motor driver will convert volts to PWM)
//         // setLeftMotorsVoltage(leftCmd);
//         // setRightMotorsVoltage(rightCmd);
//     }

//     float getLastLeftVoltage() const  { return _lastLeftVoltage; }
//     float getLastRightVoltage() const { return _lastRightVoltage; }

//     void reset() {
//         _pidLeft.reset();
//         _pidRight.reset();
//         _pidOmega.reset();
//         _targetV = 0.0f;
//         _targetOmega = 0.0f;
//         _straightMode = false;
//         _headingSetpointValid = false;
//     }

// private:
//     PID _pidLeft, _pidRight, _pidOmega;
//     float _targetV = 0.0f, _targetOmega = 0.0f;
//     bool _straightMode = false;
//     float _headingSetpoint = 0.0f;
//     bool _headingSetpointValid = false;
//     float prevLeft, prevRight;

//     // last computed voltages (exposed via getters)
//     float _lastLeftVoltage;
//     float _lastRightVoltage;
// };


// #pragma once

// #include "PID.h"
// #include "motor_control.h"
// #include <cmath>

// // --- Feedforward tables (from extracted logs) ---
// static const int FF_POINTS = 11;
// static const float ff_voltages[FF_POINTS] = {2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f, 11.0f, 11.5f};
// static const float ff_left_speed_pos[FF_POINTS]  = {  2.0f,  21.0f,  31.0f,  42.0f,  54.0f,  68.0f,  79.0f,  92.0f,  92.0f,  92.0f,  92.0f };
// static const float ff_right_speed_pos[FF_POINTS] = {  0.0f,  14.0f,  28.0f,  39.0f,  51.0f,  62.0f,  72.0f,  83.0f,  83.0f,  82.0f,  82.0f };
// static const float ff_yaw_deg[FF_POINTS] = {  0.56f,  8.70f, 17.87f, 25.25f, 32.15f, 38.54f, 44.48f, 50.25f, 50.33f, 50.08f, 50.20f };
// static const float ff_vdiff[FF_POINTS]   = {  4.0f,   6.0f,   8.0f,  10.0f,  12.0f,  14.0f,  16.0f,  18.0f,  20.0f,  22.0f,  23.0f };

// static inline float lerp(float a, float b, float t) { return a + t * (b - a); }

// static float interpFromTable(const float *xArr, const float *yArr, int n, float x) {
//     if (n <= 0) return 0.0f;
//     if (x <= xArr[0]) return yArr[0];
//     if (x >= xArr[n-1]) return yArr[n-1];
//     for (int i = 0; i < n-1; ++i) {
//         if (x >= xArr[i] && x <= xArr[i+1]) {
//             float t = (x - xArr[i]) / (xArr[i+1] - xArr[i]);
//             return lerp(yArr[i], yArr[i+1], t);
//         }
//     }
//     return yArr[n-1];
// }

// // Feedforward helpers (signed outputs)
// static inline float getFeedforwardVoltageLeft(float desiredSpeed_ticks) {
//     int sign = (desiredSpeed_ticks >= 0.0f) ? 1 : -1;
//     float absSp = fabsf(desiredSpeed_ticks);
//     float v = interpFromTable(ff_left_speed_pos, ff_voltages, FF_POINTS, absSp);
//     return v * sign;
// }

// static inline float getFeedforwardVoltageRight(float desiredSpeed_ticks) {
//     int sign = (desiredSpeed_ticks >= 0.0f) ? 1 : -1;
//     float absSp = fabsf(desiredSpeed_ticks);
//     float v = interpFromTable(ff_right_speed_pos, ff_voltages, FF_POINTS, absSp);
//     return v * sign;
// }

// static inline float getAngularFeedforwardVoltageDiff(float desiredOmega_degps) {
//     float sign = (desiredOmega_degps >= 0.0f) ? 1.0f : -1.0f;
//     float absOmega = fabsf(desiredOmega_degps);
//     float vdiff = interpFromTable(ff_yaw_deg, ff_vdiff, FF_POINTS, absOmega);
//     return sign * vdiff;
// }

// class MotionController {
// public:
//     MotionController()
//         : _pidLeft(KP_VEL, KI_VEL, KD_VEL, CONTROL_DT, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE),
//           _pidRight(KP_VEL, KI_VEL, KD_VEL, CONTROL_DT, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE),
//           _pidOmega(KP_OMEGA, KI_OMEGA, KD_OMEGA, CONTROL_DT, -MAX_OMEGA_RADPS, MAX_OMEGA_RADPS),
//           prevLeft(0.0f), prevRight(0.0f),
//           _lastLeftVoltage(0.0f), _lastRightVoltage(0.0f) {}

//     // Set target velocities (linear m/s, angular rad/s)
//     void setTargetVelocity(float v, float omega) {
//         _targetV = v;
//         _targetOmega = omega;
//         _straightMode = false;
//     }

//     // Go straight at given speed (m/s), using current heading as reference
//     void setStraight(float speed) {
//         _targetV = speed;
//         _targetOmega = 0.0f;
//         _straightMode = true;
//         _headingSetpointValid = false;
//     }

//     // Lock heading explicitly (radians)
//     void lockHeading(float yawAngle) {
//         _headingSetpoint = yawAngle;
//         _headingSetpointValid = true;
//     }

//     // Main update: computes voltages and stores them internally.
//     // Signature matches your task usage: (leftTicksAvg, rightTicksAvg, yaw_rad, yawRate_radps, dt, motorsVoltage)
//     void update(float leftTicksAvg, float rightTicksAvg, float yaw, float yawRate_radps, float dt, float *motorsVoltage) {
//         // 1) wheel speeds (ticks/s) + small moving average filter
//         float leftTickSpeed = (leftTicksAvg - prevLeft) / dt;
//         float rightTickSpeed = (rightTicksAvg - prevRight) / dt;

//         const int FILTER_WINDOW_SIZE = 3;
//         static float leftBuf[FILTER_WINDOW_SIZE] = {0};
//         static float rightBuf[FILTER_WINDOW_SIZE] = {0};
//         static uint8_t idx = 0;

//         leftBuf[idx] = leftTickSpeed;
//         rightBuf[idx] = rightTickSpeed;
//         idx = (idx + 1) % FILTER_WINDOW_SIZE;

//         float leftTickSpeedF = 0.0f, rightTickSpeedF = 0.0f;
//         for (int i = 0; i < FILTER_WINDOW_SIZE; ++i) { leftTickSpeedF += leftBuf[i]; rightTickSpeedF += rightBuf[i]; }
//         leftTickSpeedF /= FILTER_WINDOW_SIZE;
//         rightTickSpeedF /= FILTER_WINDOW_SIZE;

//         prevLeft = leftTicksAvg;
//         prevRight = rightTicksAvg;

//         // 2) Heading correction (straight mode)
//         static float headingError = 0.0f;
//         if (_straightMode) {
//             if (!_headingSetpointValid) {
//                 _headingSetpoint = yaw;
//                 _headingSetpointValid = true;
//                 Serial.printf("heading locked at %.3f rad (%.2f deg)\n", _headingSetpoint, _headingSetpoint * RAD_TO_DEG);
//             }
//             headingError = yaw - _headingSetpoint;
//             headingError = atan2f(sinf(headingError), cosf(headingError));
//             _targetOmega = KP_HEADING * headingError;
//         }

//         // 3) Angular feedforward (from yaw target)
//         float desiredOmega_dps = -(_targetOmega * RAD_TO_DEG); // convert to deg/s and sign convention
//         float ff_angular_volt_diff = getAngularFeedforwardVoltageDiff(desiredOmega_dps);
//         int8_t sign = (ff_angular_volt_diff >= 0.0f) ? 1 : -1;
//         float left_angular_ff = sign * fabsf(ff_angular_volt_diff) / 2.0f;
//         float right_angular_ff = -sign * fabsf(ff_angular_volt_diff) / 2.0f;

//         // 4) Inner angular PID on yaw rate (use filtered gyro)
//         const float RESOLUTION_RADPS = DEG_TO_RAD / 0.5f; // quantization threshold
//         float yawRate_corrected = yawRate_radps;
//         if (fabsf(yawRate_corrected) < RESOLUTION_RADPS) yawRate_corrected = 0.0f;
//         float quantised = roundf(yawRate_corrected / RESOLUTION_RADPS) * RESOLUTION_RADPS;
//         float omegaCorrection = constrain(_pidOmega.compute(_targetOmega, quantised), -MAX_OMEGA_RADPS, MAX_OMEGA_RADPS);

//         // optional slew limiting on omega command (keeps angular accel reasonable)
//         static float prev_omega = 0.0f;
//         const float MAX_SLEW_RATE = 2.5f; // rad/s per update (tune)
//         float omega_cmd = omegaCorrection;
//         float delta = omega_cmd - prev_omega;
//         if (delta > MAX_SLEW_RATE) omega_cmd = prev_omega + MAX_SLEW_RATE;
//         if (delta < -MAX_SLEW_RATE) omega_cmd = prev_omega - MAX_SLEW_RATE;
//         prev_omega = omega_cmd;

//         // 5) Kinematics -> desired wheel linear speeds (m/s)
//         float leftDesired_mps = _targetV - omega_cmd * (ROBOT_TRACK_WIDTH / 2.0f);
//         float rightDesired_mps = _targetV + omega_cmd * (ROBOT_TRACK_WIDTH / 2.0f);

//         // 6) Convert to ticks/s
//         const float leftDesired = leftDesired_mps * TICKS_PER_METER;
//         const float rightDesired = rightDesired_mps * TICKS_PER_METER;

//         // 7) Feedforward + PID
//         const float leftFF = getFeedforwardVoltageLeft(leftDesired);
//         const float rightFF = getFeedforwardVoltageRight(rightDesired);

//         float leftPIDout = _pidLeft.compute(leftDesired, leftTickSpeedF);
//         float rightPIDout = _pidRight.compute(rightDesired, rightTickSpeedF);

//         float leftCmd, rightCmd;
//         if (fabsf(headingError) >= (DEG_TO_RAD * 8.0f)) {
//             // large heading error -> rely on angular feedforward to correct turns
//             leftCmd = leftFF + left_angular_ff;
//             rightCmd = rightFF + right_angular_ff;
//         } else {
//             leftCmd = leftFF + leftPIDout;
//             rightCmd = rightFF + rightPIDout;
//         }

//         // 8) Constrain and small deadband
//         leftCmd = constrain(leftCmd, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE);
//         rightCmd = constrain(rightCmd, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE);

//         if (_targetV == 0.0f && _targetOmega == 0.0f) {
//             if (fabsf(leftCmd) < 2.0f && fabsf(rightCmd) < 2.0f) {
//                 leftCmd = 0.0f;
//                 rightCmd = 0.0f;
//             }
//         }

//         // 9) Store and optionally return via output array
//         _lastLeftVoltage  = leftCmd;
//         _lastRightVoltage = rightCmd;
//         if (motorsVoltage) {
//             motorsVoltage[0] = leftCmd;
//             motorsVoltage[1] = rightCmd;
//         }

//         // Debug prints (compact)
//         Serial.printf("MCtrl: L_des=%.1f t/s R_des=%.1f t/s L_ff=%.2f R_ff=%.2f L_pid=%.2f R_pid=%.2f L_out=%.2f R_out=%.2f yawRate=%.2f dps\n",
//                       leftDesired, rightDesired, leftFF, rightFF, leftPIDout, rightPIDout, _lastLeftVoltage, _lastRightVoltage, yawRate_radps * RAD_TO_DEG);
//     }

//     // getters
//     float getLastLeftVoltage() const  { return _lastLeftVoltage; }
//     float getLastRightVoltage() const { return _lastRightVoltage; }

//     void reset() {
//         _pidLeft.reset();
//         _pidRight.reset();
//         _pidOmega.reset();
//         _targetV = 0.0f;
//         _targetOmega = 0.0f;
//         _straightMode = false;
//         _headingSetpointValid = false;
//     }

// private:
//     PID _pidLeft, _pidRight, _pidOmega;
//     float _targetV = 0.0f, _targetOmega = 0.0f;
//     bool _straightMode = false;
//     float _headingSetpoint = 0.0f;
//     bool _headingSetpointValid = false;
//     float prevLeft, prevRight;

//     // last computed voltages
//     float _lastLeftVoltage;
//     float _lastRightVoltage;
// };



// // MotionController.h
// #pragma once

// #include "PID.h"
// #include "motor_control.h"
// #include <cmath>

// // --- Feedforward tables (from extracted logs) ---
// static const int FF_POINTS = 11;
// static const float ff_voltages[FF_POINTS] = {2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f, 11.0f, 11.5f};
// static const float ff_left_speed_pos[FF_POINTS]  = {  2.0f,  21.0f,  31.0f,  42.0f,  54.0f,  68.0f,  79.0f,  92.0f,  92.0f,  92.0f,  92.0f };
// static const float ff_right_speed_pos[FF_POINTS] = {  0.0f,  14.0f,  28.0f,  39.0f,  51.0f,  62.0f,  72.0f,  83.0f,  83.0f,  82.0f,  82.0f };
// static const float ff_yaw_deg[FF_POINTS] = {  0.56f,  8.70f, 17.87f, 25.25f, 32.15f, 38.54f, 44.48f, 50.25f, 50.33f, 50.08f, 50.20f };
// static const float ff_vdiff[FF_POINTS]   = {  4.0f,   6.0f,   8.0f,  10.0f,  12.0f,  14.0f,  16.0f,  18.0f,  20.0f,  22.0f,  23.0f };

// static inline float lerp(float a, float b, float t) { return a + t * (b - a); }

// static float interpFromTable(const float *xArr, const float *yArr, int n, float x) {
//     if (n <= 0) return 0.0f;
//     if (x <= xArr[0]) return yArr[0];
//     if (x >= xArr[n-1]) return yArr[n-1];
//     for (int i = 0; i < n-1; ++i) {
//         if (x >= xArr[i] && x <= xArr[i+1]) {
//             float t = (x - xArr[i]) / (xArr[i+1] - xArr[i]);
//             return lerp(yArr[i], yArr[i+1], t);
//         }
//     }
//     return yArr[n-1];
// }

// // Feedforward helpers (signed outputs)
// static inline float getFeedforwardVoltageLeft(float desiredSpeed_ticks) {
//     int sign = (desiredSpeed_ticks >= 0.0f) ? 1 : -1;
//     float absSp = fabsf(desiredSpeed_ticks);
//     float v = interpFromTable(ff_left_speed_pos, ff_voltages, FF_POINTS, absSp);
//     return v * sign;
// }

// static inline float getFeedforwardVoltageRight(float desiredSpeed_ticks) {
//     int sign = (desiredSpeed_ticks >= 0.0f) ? 1 : -1;
//     float absSp = fabsf(desiredSpeed_ticks);
//     float v = interpFromTable(ff_right_speed_pos, ff_voltages, FF_POINTS, absSp);
//     return v * sign;
// }

// static inline float getAngularFeedforwardVoltageDiff(float desiredOmega_degps) {
//     float sign = (desiredOmega_degps >= 0.0f) ? 1.0f : -1.0f;
//     float absOmega = fabsf(desiredOmega_degps);
//     float vdiff = interpFromTable(ff_yaw_deg, ff_vdiff, FF_POINTS, absOmega);
//     return sign * vdiff;
// }

// class MotionController {
// public:
//     MotionController()
//         : _pidLeft(KP_VEL, KI_VEL, KD_VEL, CONTROL_DT, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE),
//           _pidRight(KP_VEL, KI_VEL, KD_VEL, CONTROL_DT, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE),
//           _pidOmega(KP_OMEGA, KI_OMEGA, KD_OMEGA, CONTROL_DT, -MAX_OMEGA_RADPS, MAX_OMEGA_RADPS),
//           prevLeft(0.0f), prevRight(0.0f),
//           _lastLeftVoltage(0.0f), _lastRightVoltage(0.0f),
//           _headingSetpoint(0.0f), _headingSetpointValid(false) {}

//     // Set target velocities (linear m/s, angular rad/s)
//     void setTargetVelocity(float v, float omega) {
//         _targetV = v;
//         _targetOmega = omega;
//         _straightMode = false;
//     }

//     // Go straight at given speed (m/s), using current heading as reference
//     // Default behavior: do not clear an existing explicit heading setpoint.
//     void setStraight(float speed) {
//         _targetV = speed;
//         _targetOmega = 0.0f;
//         _straightMode = true;
//         // keep _headingSetpointValid as-is; caller can set/clear explicitly
//     }

//     // Overload: setStraight and force capture on next update (legacy behavior)
//     void setStraight(float speed, bool captureNow) {
//         _targetV = speed;
//         _targetOmega = 0.0f;
//         _straightMode = true;
//         if (captureNow) _headingSetpointValid = false; // cause update() to capture current yaw once
//     }

//     // Lock heading explicitly (radians)
//     void lockHeading(float yawAngle) {
//         _headingSetpoint = yawAngle;
//         _headingSetpointValid = true;
//     }

//     // Explicitly set heading setpoint; force=true marks it valid immediately
//     void setHeadingSetpoint(float yawAngle, bool force = false) {
//         _headingSetpoint = yawAngle;
//         if (force) _headingSetpointValid = true;
//     }

//     // Main update: computes voltages and stores them internally.
//     // Signature: (leftTicksAvg, rightTicksAvg, yaw_rad, yawRate_radps, dt, motorsVoltage)
//     void update(float leftTicksAvg, float rightTicksAvg, float yaw, float yawRate_radps, float dt, float *motorsVoltage) {
//         // 1) wheel speeds (ticks/s) + small moving average filter
//         float leftTickSpeed = (leftTicksAvg - prevLeft) / dt;
//         float rightTickSpeed = (rightTicksAvg - prevRight) / dt;

//         const int FILTER_WINDOW_SIZE = 3;
//         static float leftBuf[FILTER_WINDOW_SIZE] = {0};
//         static float rightBuf[FILTER_WINDOW_SIZE] = {0};
//         static uint8_t idx = 0;

//         leftBuf[idx] = leftTickSpeed;
//         rightBuf[idx] = rightTickSpeed;
//         idx = (idx + 1) % FILTER_WINDOW_SIZE;

//         float leftTickSpeedF = 0.0f, rightTickSpeedF = 0.0f;
//         for (int i = 0; i < FILTER_WINDOW_SIZE; ++i) { leftTickSpeedF += leftBuf[i]; rightTickSpeedF += rightBuf[i]; }
//         leftTickSpeedF /= FILTER_WINDOW_SIZE;
//         rightTickSpeedF /= FILTER_WINDOW_SIZE;

//         prevLeft = leftTicksAvg;
//         prevRight = rightTicksAvg;

//         // 2) Heading correction (straight mode)
//         static float headingError = 0.0f;
//         if (_straightMode) {
//             if (!_headingSetpointValid) {
//                 // only auto-capture when no explicit setpoint exists
//                 _headingSetpoint = yaw;
//                 _headingSetpointValid = true;
//                 Serial.printf("heading auto-locked at %.3f rad (%.2f deg)\n", _headingSetpoint, _headingSetpoint * RAD_TO_DEG);
//             }
//             headingError = yaw - _headingSetpoint;
//             headingError = atan2f(sinf(headingError), cosf(headingError));
//             _targetOmega = KP_HEADING * headingError;
//         }

//         // 3) Angular feedforward (from yaw target)
//         float desiredOmega_dps = -(_targetOmega * RAD_TO_DEG); // convert to deg/s and sign convention
//         float ff_angular_volt_diff = getAngularFeedforwardVoltageDiff(desiredOmega_dps);
//         int8_t sign = (ff_angular_volt_diff >= 0.0f) ? 1 : -1;
//         float left_angular_ff = sign * fabsf(ff_angular_volt_diff) / 2.0f;
//         float right_angular_ff = -sign * fabsf(ff_angular_volt_diff) / 2.0f;

//         // 4) Inner angular PID on yaw rate (use filtered gyro)
//         const float RESOLUTION_RADPS = DEG_TO_RAD / 0.5f; // quantization threshold
//         float yawRate_corrected = yawRate_radps;
//         if (fabsf(yawRate_corrected) < RESOLUTION_RADPS) yawRate_corrected = 0.0f;
//         float quantised = roundf(yawRate_corrected / RESOLUTION_RADPS) * RESOLUTION_RADPS;
//         float omegaCorrection = constrain(_pidOmega.compute(_targetOmega, quantised), -MAX_OMEGA_RADPS, MAX_OMEGA_RADPS);

//         // optional slew limiting on omega command (keeps angular accel reasonable)
//         static float prev_omega = 0.0f;
//         const float MAX_SLEW_RATE = 2.5f; // rad/s per update (tune)
//         float omega_cmd = omegaCorrection;
//         float delta = omega_cmd - prev_omega;
//         if (delta > MAX_SLEW_RATE) omega_cmd = prev_omega + MAX_SLEW_RATE;
//         if (delta < -MAX_SLEW_RATE) omega_cmd = prev_omega - MAX_SLEW_RATE;
//         prev_omega = omega_cmd;

//         // 5) Kinematics -> desired wheel linear speeds (m/s)
//         float leftDesired_mps = _targetV - omega_cmd * (ROBOT_TRACK_WIDTH / 2.0f);
//         float rightDesired_mps = _targetV + omega_cmd * (ROBOT_TRACK_WIDTH / 2.0f);

//         // 6) Convert to ticks/s
//         const float leftDesired = leftDesired_mps * TICKS_PER_METER;
//         const float rightDesired = rightDesired_mps * TICKS_PER_METER;

//         // 7) Feedforward + PID
//         const float leftFF = getFeedforwardVoltageLeft(leftDesired);
//         const float rightFF = getFeedforwardVoltageRight(rightDesired);

//         float leftPIDout = _pidLeft.compute(leftDesired, leftTickSpeedF);
//         float rightPIDout = _pidRight.compute(rightDesired, rightTickSpeedF);

//         float leftCmd, rightCmd;
//         if (fabsf(headingError) >= (DEG_TO_RAD * 8.0f)) {
//             // large heading error -> rely on angular feedforward to correct turns
//             leftCmd = leftFF + left_angular_ff;
//             rightCmd = rightFF + right_angular_ff;
//         } else {
//             leftCmd = leftFF + leftPIDout;
//             rightCmd = rightFF + rightPIDout;
//         }

//         // 8) Constrain and small deadband
//         leftCmd = constrain(leftCmd, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE);
//         rightCmd = constrain(rightCmd, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE);

//         if (_targetV == 0.0f && _targetOmega == 0.0f) {
//             if (fabsf(leftCmd) < 2.0f && fabsf(rightCmd) < 2.0f) {
//                 leftCmd = 0.0f;
//                 rightCmd = 0.0f;
//             }
//         }

//         // 9) Store and optionally return via output array
//         _lastLeftVoltage  = leftCmd;
//         _lastRightVoltage = rightCmd;
//         if (motorsVoltage) {
//             motorsVoltage[0] = leftCmd;
//             motorsVoltage[1] = rightCmd;
//         }

//         // Debug prints (compact)
//         Serial.printf("MCtrl: L_des=%.1f t/s R_des=%.1f t/s L_ff=%.2f R_ff=%.2f L_pid=%.2f R_pid=%.2f L_out=%.2f R_out=%.2f yawRate=%.2f dps\n",
//                       leftDesired, rightDesired, leftFF, rightFF, leftPIDout, rightPIDout, _lastLeftVoltage, _lastRightVoltage, yawRate_radps * RAD_TO_DEG);
//     }

//     // getters
//     float getLastLeftVoltage() const  { return _lastLeftVoltage; }
//     float getLastRightVoltage() const { return _lastRightVoltage; }

//     void reset() {
//         _pidLeft.reset();
//         _pidRight.reset();
//         _pidOmega.reset();
//         _targetV = 0.0f;
//         _targetOmega = 0.0f;
//         _straightMode = false;
//         _headingSetpointValid = false;
//     }

// private:
//     PID _pidLeft, _pidRight, _pidOmega;
//     float _targetV = 0.0f, _targetOmega = 0.0f;
//     bool _straightMode = false;
//     float _headingSetpoint = 0.0f;
//     bool _headingSetpointValid = false;
//     float prevLeft, prevRight;

//     // last computed voltages
//     float _lastLeftVoltage;
//     float _lastRightVoltage;
// };



#pragma once

#include "PID.h"
#include "motor_control.h"
#include <cmath>

// --- Feedforward tables (unchanged) ---
static const int FF_POINTS = 11;
static const float ff_voltages[FF_POINTS] = {2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f, 11.0f, 11.5f};
static const float ff_left_speed_pos[FF_POINTS]  = {  2.0f,  21.0f,  31.0f,  42.0f,  54.0f,  68.0f,  79.0f,  92.0f,  92.0f,  92.0f,  92.0f };
static const float ff_right_speed_pos[FF_POINTS] = {  0.0f,  14.0f,  28.0f,  39.0f,  51.0f,  62.0f,  72.0f,  83.0f,  83.0f,  82.0f,  82.0f };
static const float ff_yaw_deg[FF_POINTS] = {  0.56f,  8.70f, 17.87f, 25.25f, 32.15f, 38.54f, 44.48f, 50.25f, 50.33f, 50.08f, 50.20f };
static const float ff_vdiff[FF_POINTS]   = {  4.0f,   6.0f,   8.0f,  10.0f,  12.0f,  14.0f,  16.0f,  18.0f,  20.0f,  22.0f,  23.0f };

static inline float lerp(float a, float b, float t) { return a + t * (b - a); }
static float interpFromTable(const float *xArr, const float *yArr, int n, float x) {
    if (n <= 0) return 0.0f;
    if (x <= xArr[0]) return yArr[0];
    if (x >= xArr[n-1]) return yArr[n-1];
    for (int i = 0; i < n-1; ++i) {
        if (x >= xArr[i] && x <= xArr[i+1]) {
            float t = (x - xArr[i]) / (xArr[i+1] - xArr[i]);
            return lerp(yArr[i], yArr[i+1], t);
        }
    }
    return yArr[n-1];
}

static inline float getFeedforwardVoltageLeft(float desiredSpeed_ticks) {
    if (fabsf(desiredSpeed_ticks) < 1.0f) return 0.0f;   // <-- deadband
    int sign = (desiredSpeed_ticks >= 0.0f) ? 1 : -1;
    float absSp = fabsf(desiredSpeed_ticks);
    float v = interpFromTable(ff_left_speed_pos, ff_voltages, FF_POINTS, absSp);
    return v * sign;
}

static inline float getFeedforwardVoltageRight(float desiredSpeed_ticks) {
    if (fabsf(desiredSpeed_ticks) < 1.0f) return 0.0f;   // <-- deadband
    int sign = (desiredSpeed_ticks >= 0.0f) ? 1 : -1;
    float absSp = fabsf(desiredSpeed_ticks);
    float v = interpFromTable(ff_right_speed_pos, ff_voltages, FF_POINTS, absSp);
    return v * sign;
}


static inline float getAngularFeedforwardVoltageDiff(float desiredOmega_degps) {
    const float DEADBAND_DPS = 0.5f;                    // below this → no correction
    if (fabsf(desiredOmega_degps) < DEADBAND_DPS) return 0.0f;

    float sign     = (desiredOmega_degps >= 0.0f) ? 1.0f : -1.0f;
    float absOmega = fabsf(desiredOmega_degps);

    // Ramp smoothly from 0 V at 0 dps up to table[0] for small angles,
    // so we never apply the 4 V floor to tiny errors.
    if (absOmega < ff_yaw_deg[0]) {
        return sign * (ff_vdiff[0] * (absOmega / ff_yaw_deg[0]));
    }
    return sign * interpFromTable(ff_yaw_deg, ff_vdiff, FF_POINTS, absOmega);
}

class MotionController {
public:
    MotionController()
        : _pidOmega(KP_OMEGA, KI_OMEGA, KD_OMEGA, CONTROL_DT, -MAX_OMEGA_RADPS, MAX_OMEGA_RADPS),
          _lastLeftVoltage(0.0f), _lastRightVoltage(0.0f),
          _headingSetpoint(0.0f), _headingSetpointValid(false) {}

    void setTargetVelocity(float v, float omega) {
        _targetV = v;
        _targetOmega = omega;
        _straightMode = false;
    }

    void setStraight(float speed) {
        _targetV = speed;
        _targetOmega = 0.0f;
        _straightMode = true;
    }

    void setStraight(float speed, bool captureNow) {
        _targetV = speed;
        _targetOmega = 0.0f;
        _straightMode = true;
        if (captureNow) _headingSetpointValid = false;
    }

    void lockHeading(float yawAngle) {
        _headingSetpoint = yawAngle;
        _headingSetpointValid = true;
    }

    void setHeadingSetpoint(float yawAngle, bool force = false) {
        _headingSetpoint = yawAngle;
        if (force) _headingSetpointValid = true;
    }

    void update(float /*leftTicksAvg*/, float /*rightTicksAvg*/, float yaw, float yawRate_radps, float dt, float *motorsVoltage) {
        (void)dt;   // not used directly in this simplified version

        // 1) Heading correction (straight mode) – uses IMU yaw
        float headingError = 0.0f;
        if (_straightMode) {
            if (!_headingSetpointValid) {
                _headingSetpoint = yaw;
                _headingSetpointValid = true;
                // Optional debug print
                // Serial.printf("heading auto-locked at %.3f rad\n", _headingSetpoint);
            }
            headingError = yaw - _headingSetpoint;
            headingError = atan2f(sinf(headingError), cosf(headingError));
            _targetOmega = KP_HEADING * headingError;
        }

        // 2) Angular feedforward (from desired omega)
        float desiredOmega_dps = -(_targetOmega * RAD_TO_DEG);
        float ff_angular_volt_diff = getAngularFeedforwardVoltageDiff(desiredOmega_dps);
        int8_t sign = (ff_angular_volt_diff >= 0.0f) ? 1 : -1;
        float left_angular_ff = sign * fabsf(ff_angular_volt_diff) / 2.0f;
        float right_angular_ff = -sign * fabsf(ff_angular_volt_diff) / 2.0f;

        // 3) Inner angular PID on yaw rate (IMU)
        const float RESOLUTION_RADPS = DEG_TO_RAD / 0.5f;
        float yawRate_corrected = yawRate_radps;
        if (fabsf(yawRate_corrected) < RESOLUTION_RADPS) yawRate_corrected = 0.0f;
        float quantised = roundf(yawRate_corrected / RESOLUTION_RADPS) * RESOLUTION_RADPS;
        float omegaCorrection = constrain(_pidOmega.compute(_targetOmega, quantised), -MAX_OMEGA_RADPS, MAX_OMEGA_RADPS);

        // Optional slew limiting
        static float prev_omega = 0.0f;
        const float MAX_SLEW_RATE = 2.5f;
        float omega_cmd = omegaCorrection;
        float delta = omega_cmd - prev_omega;
        if (delta > MAX_SLEW_RATE) omega_cmd = prev_omega + MAX_SLEW_RATE;
        if (delta < -MAX_SLEW_RATE) omega_cmd = prev_omega - MAX_SLEW_RATE;
        prev_omega = omega_cmd;

        // 4) Kinematics -> desired wheel speeds (m/s)
        float leftDesired_mps = _targetV - omega_cmd * (ROBOT_TRACK_WIDTH / 2.0f);
        float rightDesired_mps = _targetV + omega_cmd * (ROBOT_TRACK_WIDTH / 2.0f);

        // 5) Convert to ticks/s for feedforward lookup
        float leftDesired = leftDesired_mps * TICKS_PER_METER;
        float rightDesired = rightDesired_mps * TICKS_PER_METER;

        // 6) Feedforward voltages + angular corrections (no wheel speed PID)
        float leftCmd = getFeedforwardVoltageLeft(leftDesired) + left_angular_ff;
        float rightCmd = getFeedforwardVoltageRight(rightDesired) + right_angular_ff;

        // 7) Constrain and deadband
        leftCmd = constrain(leftCmd, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE);
        rightCmd = constrain(rightCmd, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE);

        if (_targetV == 0.0f && _targetOmega == 0.0f) {
            if (fabsf(leftCmd) < 2.0f && fabsf(rightCmd) < 2.0f) {
                leftCmd = 0.0f;
                rightCmd = 0.0f;
            }
        }

        // 8) Store and output
        _lastLeftVoltage = leftCmd;
        _lastRightVoltage = rightCmd;
        if (motorsVoltage) {
            motorsVoltage[0] = leftCmd;
            motorsVoltage[1] = rightCmd;
        }

        // Debug print (optional)
        // Serial.printf("yaw=%.2f targetOmega=%.2f omegaCorr=%.2f L=%.2f R=%.2f\n",
        //               yaw * RAD_TO_DEG, _targetOmega, omega_cmd, leftCmd, rightCmd);
    }

    float getLastLeftVoltage() const { return _lastLeftVoltage; }
    float getLastRightVoltage() const { return _lastRightVoltage; }

    void reset() {
        _pidOmega.reset();
        _targetV = 0.0f;
        _targetOmega = 0.0f;
        _straightMode = false;
        _headingSetpointValid = false;
    }

private:
    PID _pidOmega;                    // yaw-rate PID (IMU only)
    float _targetV = 0.0f, _targetOmega = 0.0f;
    bool _straightMode = false;
    float _headingSetpoint = 0.0f;
    bool _headingSetpointValid = false;
    float _lastLeftVoltage, _lastRightVoltage;
};