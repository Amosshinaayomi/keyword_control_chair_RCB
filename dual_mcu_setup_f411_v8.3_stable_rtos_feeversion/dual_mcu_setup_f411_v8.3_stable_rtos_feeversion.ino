#include "RobotConfig.h"
#include "AHRS.h"
#include <task.h>
#include <EncoderManager.h>
#include "motor_control.h"
#include "MotionController.h"
#include "communications.h"
#include "comm_protocol.h"
#include <event_groups.h>
#include "Wire.h"
#include "PCF8574.h"
#include <Adafruit_VL53L0X.h>

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    Serial.print("STACK OVERFLOW in ");
    Serial.println(pcTaskName);
    while (1);
}

// ==================== SHARED STATE ====================
struct Pose          { float x, y, theta; uint32_t timestamp_ms; };
struct PlannerCommand{ float linear, angular, heading_deg; bool valid; uint32_t timestamp_ms; };
struct FinalCommand  { float linear, angular; uint8_t mode; uint32_t timestamp_ms; };
struct ManualCommand { float linear = 0.0f; float angular = 0.0f; bool active = false; };

struct ObstacleData {
    float frontLeft  = 9999.0f;
    float frontRight = 9999.0f;
    float backLeft   = 9999.0f;
    float backRight  = 9999.0f;
    bool  flValid    = false;              // set true only if this cycle produced a valid FL reading
    bool  frValid    = false;              // set true only if this cycle produced a valid FR reading
    uint32_t flLastValid_ms = 0;           // timestamp of last valid FL reading (persistent across cycles)
    uint32_t frLastValid_ms = 0;           // timestamp of last valid FR reading
    uint32_t timestamp_ms   = 0;
};

// ---- Robot state machine ----
enum RobotState : uint8_t {
    ROBOT_CLEAR       = 0,   // no obstacles, joystick passes through
    ROBOT_AVOID_LEFT  = 1,   // right-side obstacle, turning left in place
    ROBOT_AVOID_RIGHT = 2,   // left-side obstacle, turning right in place
    ROBOT_HALTED      = 3,   // front blocked, full stop
    ROBOT_FAILSAFE    = 4    // sensor unhealthy, bypass avoidance
};

struct SafeCommand {
    float linear       = 0.0f;
    float angular      = 0.0f;
    bool  active       = false;
    RobotState state   = ROBOT_CLEAR;
    uint32_t timestamp_ms = 0;
};

Pose           currentPose;    SemaphoreHandle_t poseMutex;
PlannerCommand plannerCmd;     SemaphoreHandle_t plannerMutex;
FinalCommand   finalCmd;       SemaphoreHandle_t finalMutex;
ManualCommand  manualCmd;      SemaphoreHandle_t manualMutex;
ObstacleData   obstacleData;   SemaphoreHandle_t obstacleMutex;
SafeCommand    safeCmd;        SemaphoreHandle_t safeCmdMutex;

// ==================== SPEED-TEST LOG ====================
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    float leftVoltage, rightVoltage, leftSpeed, rightSpeed, yawRate;
    uint8_t isForward;
} speed_test_log_t;
#define MAX_SPEED_TEST_LOGS 64
speed_test_log_t logBuffer[MAX_SPEED_TEST_LOGS];
uint8_t logBufferCount = 0;

// ==================== SYSTEM EVENTS ====================
#define EVT_RUNNING       (1 << 0)
#define EVT_CALIB_REQUEST (1 << 1)
#define EVT_CALIBRATED    (1 << 2)
EventGroupHandle_t sysEventGroup;

bool isSystemReady() {
    EventBits_t bits = xEventGroupGetBits(sysEventGroup);
    return (bits & EVT_RUNNING) && (bits & EVT_CALIBRATED) && !(bits & EVT_CALIB_REQUEST);
}

// ==================== TOF HARDWARE ====================
PCF8574 pcf8574(0x27);
int     xshutPins[4]      = {7, 6, 3, 2};
uint8_t sensorAddresses[4]= {0x30, 0x31, 0x32, 0x33};
Adafruit_VL53L0X lox[4];
bool    sensorReady[4]    = {false, false, false, false};

volatile bool tofBurstActive = false;

// ==================== GLOBAL OBJECTS ====================
EncoderManager   encoderManager;
AHRS             ahrs(SDA_PIN, SCL_PIN, 400000UL);
MotionController motionController;

motionSensorPacket_t ahrsData;  SemaphoreHandle_t ahrsMutex;
SemaphoreHandle_t sysPwrMutex;
SemaphoreHandle_t speedTestSemaphore;
SemaphoreHandle_t i2cMutex;

bool    hardwareInitialized  = false;
uint8_t hardwareSensorStatus = 0;

volatile bool speedTestMode    = false;
volatile bool speedTestRunning = false;
volatile bool speedTestDone    = false;

TaskHandle_t ahrsHandle = NULL, motionSensorHandle = NULL, printHandle = NULL,
             powerHandle = NULL, motionControllerHandle = NULL, tofHandle = NULL;

// ==================== PROTOTYPES ====================
bool initAllHardware(uint8_t* errorCode);
void calibrateMagnetometer2D(int duration);
void dumpSpeedTestLogs();
void resetSpeedTestLogs();
void printBufferLogs();

void ahrsTask(void*);
void tofTask(void*);
void obstacleWatchdogTask(void*);
void ahrsCalibrationTask(void*);
void motionSensorTask(void*);
void motionControllerTask(void*);
void manualControlsTask(void*);
void powerTask(void*);
void serialTask(void*);
void printTask(void*);

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ==================== AHRS TASK ====================
void ahrsTask(void *pvParameters) {
    const TickType_t period = pdMS_TO_TICKS(5);

    uint32_t updateCount  = 0;
    uint32_t lastPrint    = millis();
    const uint32_t PRINT_PERIOD_MS = 1000;

    for (;;) {
        if (!isSystemReady()) {
            lastPrint = millis();
            updateCount = 0;
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
            ahrs.update();
            updateCount++;
            xSemaphoreGive(i2cMutex);
        }

        uint32_t now = millis();
        if (now - lastPrint >= PRINT_PERIOD_MS) {
            Serial.printf("[AHRS-RATE] %lu updates in %lums -> %lu Hz | tasks=%u heap=%lu\n",
                          (unsigned long)updateCount,
                          (unsigned long)(now - lastPrint),
                          (unsigned long)updateCount,
                          (unsigned)uxTaskGetNumberOfTasks(),
                          (unsigned long)xPortGetFreeHeapSize());
            updateCount = 0;
            lastPrint   = now;
        }

        vTaskDelay(period);
    }
}

// ==================== TOF TASK ====================
void tofTask(void *pvParameters) {
    xEventGroupWaitBits(sysEventGroup, EVT_CALIBRATED, pdFALSE, pdTRUE, portMAX_DELAY);

    Serial.println("[TOF] Init: starting sensor bring-up");

    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        for (int i = 0; i < 4; i++) pcf8574.write(xshutPins[i], LOW);
        delay(100);

        for (int i = 0; i < 4; i++) {
            pcf8574.write(xshutPins[i], HIGH);
            delay(50);

            Serial.printf("[TOF] Init sensor %d -> 0x%02X ... ", i, sensorAddresses[i]);
            if (lox[i].begin(sensorAddresses[i])) {
                lox[i].configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED);
                sensorReady[i] = true;
                Serial.println("OK");
            } else {
                sensorReady[i] = false;
                Serial.println("FAILED");
            }
            delay(10);
        }
        xSemaphoreGive(i2cMutex);
    } else {
        Serial.println("[TOF] Could not acquire i2cMutex for init — aborting");
        vTaskDelete(NULL);
    }

    int readyCount = 0;
    for (int i = 0; i < 4; i++) if (sensorReady[i]) readyCount++;
    Serial.printf("[TOF] %d/4 sensors ready\n", readyCount);

    Wire.setClock(400000);
    Wire.setTimeout(20);

    const TickType_t period = pdMS_TO_TICKS(200);
    TickType_t lastWake = xTaskGetTickCount();

    // Persistent last-valid timestamps (survive across cycles)
    uint32_t flLastValid = 0;
    uint32_t frLastValid = 0;

    for (;;) {
        uint32_t t_start = micros();

        if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
            tofBurstActive = true;

            ObstacleData local;
            local.timestamp_ms = millis();

            uint16_t raw_mm[4] = {0, 0, 0, 0};
            uint8_t  raw_st[4] = {255, 255, 255, 255};

            for (int i = 0; i < 4; i++) {
                if (!sensorReady[i]) continue;

                VL53L0X_RangingMeasurementData_t measure;
                uint32_t t_sensor = micros();
                VL53L0X_Error err = lox[i].rangingTest(&measure, false);
                uint32_t dur_us   = micros() - t_sensor;

                raw_mm[i] = 0;
                raw_st[i] = 255;

                if (err != VL53L0X_ERROR_NONE) {
                    // Includes -7 (VL53L0X_ERROR_TIME_OUT) and other errors
                    Serial.printf("[TOF] sensor %d err=%d took %luus\n",
                                  i, (int)err, (unsigned long)dur_us);
                    continue;
                }

                uint8_t  st = measure.RangeStatus;
                uint16_t mm = measure.RangeMilliMeter;

                raw_mm[i] = mm;
                raw_st[i] = st;

                // Reject 0 mm, out-of-range, and any non-OK status
                bool valid = (st == 0 || st == 1 || st == 2 || st == 3) &&
                             (mm > 30) && (mm < 8000);
                if (!valid) continue;

                switch (i) {
                    case 2:  // physical FRONT RIGHT
                        local.frontRight = mm;
                        local.frValid = true;
                        frLastValid = millis();
                        break;
                    case 3:  // physical FRONT LEFT
                        local.frontLeft = mm;
                        local.flValid = true;
                        flLastValid = millis();
                        break;
                    case 0: local.backLeft  = mm; break;
                    case 1: local.backRight = mm; break;
                    default: break;
                }
            }

            // Carry forward last-valid timestamps
            local.flLastValid_ms = flLastValid;
            local.frLastValid_ms = frLastValid;

            xSemaphoreGive(i2cMutex);
            tofBurstActive = false;

            xSemaphoreTake(obstacleMutex, portMAX_DELAY);
            obstacleData = local;
            xSemaphoreGive(obstacleMutex);

            static uint32_t lastDiag = 0;
            if (millis() - lastDiag >= 1000) {
                lastDiag = millis();
                uint32_t burst_us = micros() - t_start;
                Serial.printf("[TOF-DIAG] burst=%luus | FL:%u(s%u,v%d) FR:%u(s%u,v%d)\n",
                              burst_us,
                              raw_mm[3], raw_st[3], local.flValid ? 1 : 0,
                              raw_mm[2], raw_st[2], local.frValid ? 1 : 0);
            }
        }

        vTaskDelayUntil(&lastWake, period);
    }
}

// ==================== OBSTACLE WATCHDOG (STATE MACHINE) ====================
void obstacleWatchdogTask(void *pvParameters) {
    const TickType_t period = pdMS_TO_TICKS(200);   // matches tofTask period
    TickType_t lastWake = xTaskGetTickCount();

    const float ENTER_MM         = 200.0f;   // transition INTO avoidance
    const float EXIT_MM          = 300.0f;   // transition OUT (hysteresis)
    const float AVOID_TURN_RATE  = 0.8f;     // rad/s while turning away
    const uint32_t SENSOR_STALE_MS = 600;    // declare sensor dead if silent this long

    RobotState state = ROBOT_CLEAR;
    uint32_t lastTransition = millis();

    for (;;) {
        if (!isSystemReady()) {
            lastWake = xTaskGetTickCount();
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        // ---- Read fresh inputs ----
        ObstacleData obs;
        xSemaphoreTake(obstacleMutex, portMAX_DELAY);
        obs = obstacleData;
        xSemaphoreGive(obstacleMutex);

        ManualCommand raw;
        xSemaphoreTake(manualMutex, portMAX_DELAY);
        raw = manualCmd;
        xSemaphoreGive(manualMutex);

        uint32_t now = millis();

        // ---- Health check ----
        // A sensor is healthy if we got a valid reading this cycle OR the
        // last valid reading is not older than SENSOR_STALE_MS.
        bool flHealthy = (obs.flLastValid_ms > 0) && (now - obs.flLastValid_ms < SENSOR_STALE_MS);
        bool frHealthy = (obs.frLastValid_ms > 0) && (now - obs.frLastValid_ms < SENSOR_STALE_MS);
        bool sensorFault = !flHealthy || !frHealthy;

        // ---- Fail-safe: bypass avoidance entirely if a sensor is unhealthy ----
        if (sensorFault) {
            if (state != ROBOT_FAILSAFE) {
                state = ROBOT_FAILSAFE;
                lastTransition = now;
                Serial.printf("[OBS] FAIL-SAFE: flH=%d frH=%d -> joystick passthrough\n",
                              flHealthy, frHealthy);
            }

            SafeCommand out;
            out.timestamp_ms = now;
            out.state   = ROBOT_FAILSAFE;
            out.linear  = raw.linear;
            out.angular = raw.angular;
            out.active  = raw.active;

            xSemaphoreTake(safeCmdMutex, portMAX_DELAY);
            safeCmd = out;
            xSemaphoreGive(safeCmdMutex);

            vTaskDelayUntil(&lastWake, period);
            continue;
        }

        // ---- Sensors are healthy: run the state machine ----
        bool flBlocked = obs.flValid && obs.frontLeft  > 0.0f && obs.frontLeft  <= ENTER_MM;
        bool frBlocked = obs.frValid && obs.frontRight > 0.0f && obs.frontRight <= ENTER_MM;
        bool flClear   = obs.flValid && obs.frontLeft  > 0.0f && obs.frontLeft  >= EXIT_MM;
        bool frClear   = obs.frValid && obs.frontRight > 0.0f && obs.frontRight >= EXIT_MM;

        RobotState prevState = state;
        switch (state) {
            case ROBOT_FAILSAFE:
                // Sensors recovered -> start clean
                state = ROBOT_CLEAR;
                break;

            case ROBOT_CLEAR:
                if (flBlocked && frBlocked)      state = ROBOT_HALTED;
                else if (flBlocked)              state = ROBOT_AVOID_RIGHT;
                else if (frBlocked)              state = ROBOT_AVOID_LEFT;
                break;

            case ROBOT_AVOID_LEFT:
                if (flBlocked && frBlocked)      state = ROBOT_HALTED;
                else if (flBlocked)              state = ROBOT_AVOID_RIGHT;
                else if (frClear)                state = ROBOT_CLEAR;
                break;

            case ROBOT_AVOID_RIGHT:
                if (flBlocked && frBlocked)      state = ROBOT_HALTED;
                else if (frBlocked)              state = ROBOT_AVOID_LEFT;
                else if (flClear)                state = ROBOT_CLEAR;
                break;

            case ROBOT_HALTED:
                if (flClear && frClear)          state = ROBOT_CLEAR;
                break;
        }

        if (state != prevState) {
            lastTransition = now;
            Serial.printf("[OBS] state %d -> %d (FL=%.0f FR=%.0f)\n",
                          (int)prevState, (int)state, obs.frontLeft, obs.frontRight);
        }

        // ---- Produce SafeCommand ----
        SafeCommand out;
        out.timestamp_ms = now;
        out.state = state;

        switch (state) {
            case ROBOT_CLEAR:
                out.linear  = raw.linear;
                out.angular = raw.angular;
                out.active  = raw.active;
                break;

            case ROBOT_AVOID_LEFT:
                out.linear  = 0.0f;
                out.angular = +AVOID_TURN_RATE;
                out.active  = true;
                break;

            case ROBOT_AVOID_RIGHT:
                out.linear  = 0.0f;
                out.angular = -AVOID_TURN_RATE;
                out.active  = true;
                break;

            case ROBOT_HALTED:
            case ROBOT_FAILSAFE:
            default:
                out.linear  = 0.0f;
                out.angular = 0.0f;
                out.active  = false;
                break;
        }

        xSemaphoreTake(safeCmdMutex, portMAX_DELAY);
        safeCmd = out;
        xSemaphoreGive(safeCmdMutex);

        vTaskDelayUntil(&lastWake, period);
    }
}

// ==================== MANUAL CONTROLS ====================
void manualControlsTask(void *parameters) {
    pinMode(JOYSTICK_X, INPUT_ANALOG);
    pinMode(JOYSTICK_Y, INPUT_ANALOG);
    pinMode(SWITCH, INPUT_PULLUP);
    analogReadResolution(10);

    const int DEADZONE = 30;

    // Fixed velocities per tilt direction
    const float LIN_SPEED = 0.3f;   // m/s   (forward / backward)
    const float ANG_SPEED = 0.4f;   // rad/s (left / right)

    // -------- Auto-calibrate joystick center --------
    const int CAL_SAMPLES = 30;
    long sumX = 0, sumY = 0;
    for (int i = 0; i < CAL_SAMPLES; i++) {
        sumX += analogRead(JOYSTICK_X);
        sumY += analogRead(JOYSTICK_Y);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    const int CENTER_X = (int)(sumX / CAL_SAMPLES);
    const int CENTER_Y = (int)(sumY / CAL_SAMPLES);

    Serial.printf("[MANUAL] Joystick center calibrated: X=%d Y=%d\n",
                  CENTER_X, CENTER_Y);

    uint32_t lastPrint = 0;
    const uint32_t PRINT_PERIOD_MS = 1500;

    for (;;) {
        int xRaw = 0, yRaw = 0;
        for (int i = 0; i < 5; i++) {
            xRaw += analogRead(JOYSTICK_X);
            yRaw += analogRead(JOYSTICK_Y);
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        int x = xRaw / 5;
        int y = yRaw / 5;

        if (abs(x - CENTER_X) < DEADZONE) x = CENTER_X;
        if (abs(y - CENTER_Y) < DEADZONE) y = CENTER_Y;

        int dx  = x - CENTER_X;
        int dy  = y - CENTER_Y;
        int adx = abs(dx);
        int ady = abs(dy);

        float linear  = 0.0f;
        float angular = 0.0f;

        if (adx >= DEADZONE || ady >= DEADZONE) {
            if (adx > ady) {
                // ---- Horizontal tilt: constant angular velocity ----
                angular = (dx > 0) ? -ANG_SPEED : +ANG_SPEED;
            } else {
                // ---- Vertical tilt: constant linear velocity ----
                linear  = (dy > 0) ? +LIN_SPEED : -LIN_SPEED;
            }
        }

        xSemaphoreTake(manualMutex, portMAX_DELAY);
        manualCmd.linear  = linear;
        manualCmd.angular = angular;
        manualCmd.active  = (linear != 0.0f) || (angular != 0.0f);
        xSemaphoreGive(manualMutex);

        uint32_t now = millis();
        if (now - lastPrint >= PRINT_PERIOD_MS) {
            lastPrint = now;
            Serial.printf("[MANUAL] x=%4d y=%4d (adx=%3d ady=%3d) | lin=%+5.2f ang=%+5.2f | active=%d\n",
                          x, y, adx, ady, linear, angular, manualCmd.active ? 1 : 0);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ==================== MOTION CONTROLLER ====================
void motionControllerTask(void *pvParameters) {
    const uint8_t task_period = 50;
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(task_period);
    CONTROL_DT = task_period / 1000.0f;
    static uint32_t prevTimestamp_ms = 0;
    static float last_yawRate = 0.0f;
    const float HYST_IN  = DEG_TO_RAD * 0.5f;
    const float HYST_OUT = DEG_TO_RAD * 1.0f;
    static bool headingInitialized = false, lastCmdWasAngular = false;

    for (;;) {
        if (!isSystemReady()) {
            lastWake = xTaskGetTickCount();
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        if (tofBurstActive) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        motionSensorPacket_t localData;
        xSemaphoreTake(ahrsMutex, portMAX_DELAY);
        localData = ahrsData;
        xSemaphoreGive(ahrsMutex);

        float yawRad = localData.yaw * DEG_TO_RAD;

        if (!headingInitialized) {
            motionController.setHeadingSetpoint(yawRad, true);
            motionController.setStraight(0);
            headingInitialized = true;
            lastCmdWasAngular = false;
            Serial.println("[MCTRL] Heading locked to current yaw after calibration.");
        }

        float dt = CONTROL_DT;
        if (prevTimestamp_ms != 0) {
            uint32_t now = localData.timestamp_ms;
            uint32_t delta = (now >= prevTimestamp_ms)
                           ? (now - prevTimestamp_ms)
                           : (UINT32_MAX - prevTimestamp_ms + now + 1);
            float dtc = delta / 1000.0f;
            if (dtc > 0.0f && dtc < 0.5f) dt = dtc;
        }
        prevTimestamp_ms = localData.timestamp_ms;

        float yawRateRad = localData.yawRateFiltered * DEG_TO_RAD;
        if (fabsf(yawRateRad) < HYST_IN) yawRateRad = 0.0f;
        else if (fabsf(yawRateRad) < HYST_OUT && fabsf(last_yawRate) < HYST_IN) yawRateRad = 0.0f;
        last_yawRate = yawRateRad;

        SafeCommand cmd;
        xSemaphoreTake(safeCmdMutex, portMAX_DELAY);
        cmd = safeCmd;
        xSemaphoreGive(safeCmdMutex);

        if (fabsf(cmd.angular) > 0.01f) {
            lastCmdWasAngular = true;
            motionController.setTargetVelocity(0.0f, cmd.angular);
        } else {
            if (lastCmdWasAngular) {
                motionController.setHeadingSetpoint(yawRad, true);
                lastCmdWasAngular = false;
            }
            motionController.setStraight(cmd.linear);
        }

        float motorsVolt[2] = {0.0f, 0.0f};
        motionController.update(0.0f, 0.0f, yawRad, yawRateRad, dt, motorsVolt);

        setLeftMotorsVoltage(motorsVolt[0]);
        setRightMotorsVoltage(motorsVolt[1]);

        vTaskDelayUntil(&lastWake, period);
    }
}

// ==================== MOTION SENSOR ====================
void motionSensorTask(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(5);
    for (;;) {
        if (!isSystemReady()) {
            lastWake = xTaskGetTickCount();
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        motionSensorPacket_t localData;
        localData.timestamp_ms = millis();
        localData.roll    = ahrs.getRoll();
        localData.pitch   = ahrs.getPitch();
        localData.yaw     = ahrs.getYaw();
        localData.yawRate = ahrs.getYawRate();

        static float filteredYawRate = 0.0f;
        static bool  first = true;
        const float  alpha = 0.15f;
        if (first) { filteredYawRate = localData.yawRate; first = false; }
        else       filteredYawRate = alpha * localData.yawRate + (1.0f - alpha) * filteredYawRate;
        localData.yawRateFiltered = filteredYawRate;

        localData.encoder_ticks[0] = localData.encoder_ticks[1] =
        localData.encoder_ticks[2] = localData.encoder_ticks[3] = 0;

        ahrs.getAccel(localData.accel_g);
        ahrs.getGyro(localData.gyro_dps);

        xSemaphoreTake(ahrsMutex, portMAX_DELAY);
        ahrsData = localData;
        xSemaphoreGive(ahrsMutex);

        vTaskDelayUntil(&lastWake, period);
    }
}

// ==================== POWER ====================
void powerTask(void *parameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);
    for (;;) {
        analogReadResolution(10);
        uint32_t adcSum = 0;
        for (uint8_t i = 0; i < 5; i++) adcSum += analogRead(BATTERY_PIN);
        BATTERY_VOLTAGE = ((float)adcSum / 5.0f / 930.0f) * BATTERY_FULL_VOLT;
        vTaskDelayUntil(&lastWake, period);
    }
}

// ==================== CALIBRATION ====================
void ahrsCalibrationTask(void *pvParameters) {
    xEventGroupWaitBits(sysEventGroup, EVT_RUNNING, pdFALSE, pdFALSE, portMAX_DELAY);
    Serial.println("[CAL] Started.");
    xEventGroupSetBits(sysEventGroup, EVT_CALIB_REQUEST);
    xEventGroupClearBits(sysEventGroup, EVT_CALIBRATED);

    while (BATTERY_VOLTAGE < 9.0f) {
        vTaskDelay(pdMS_TO_TICKS(100));
        Serial.printf("[CAL] Waiting for battery > 9V (%.2f V)\n", BATTERY_VOLTAGE);
    }

    Serial.println("[CAL] IMU calibration (keep still)...");
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        ahrs.calibrateIMU();
        xSemaphoreGive(i2cMutex);
    }

    Serial.println("[CAL] 2D magnetometer calibration (spin 360 deg on ground)...");
    calibrateMagnetometer2D(20000);

    Serial.println("[CAL] Waiting for AHRS to settle...");
    const float THRESHOLD = 0.5f;
    const int   REQUIRED  = 20;
    int stable = 0;
    float lastYaw = ahrs.getYaw();
    while (stable < REQUIRED) {
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            ahrs.update();
            float yaw = ahrs.getYaw();
            xSemaphoreGive(i2cMutex);
            if (fabsf(yaw - lastYaw) < THRESHOLD) stable++; else stable = 0;
            lastYaw = yaw;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    xEventGroupClearBits(sysEventGroup, EVT_CALIB_REQUEST);
    xEventGroupSetBits(sysEventGroup, EVT_CALIBRATED);
    Serial.println("[CAL] Done. System ready.");
    vTaskSuspend(NULL);
}

// ==================== PRINT ====================
void printTask(void *pvParameters) {
    for (;;) {
        if (isSystemReady()) {
            motionSensorPacket_t local;
            xSemaphoreTake(ahrsMutex, portMAX_DELAY);
            local = ahrsData;
            xSemaphoreGive(ahrsMutex);
            printAHRSPacket(local);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ==================== SERIAL ====================
void serialTask(void *pvParameters) {
    Serial.println("Serial task started.");
    for (;;) {
        if (Serial.available()) {
            String cmd = Serial.readStringUntil('\n');
            cmd.trim();
            if (cmd == "dumplog" || cmd == "printlog") dumpSpeedTestLogs();
            else if (cmd == "resetlog")                resetSpeedTestLogs();
            else if (cmd == "showbuffer")              printBufferLogs();
            else if (cmd == "speedtest" && !speedTestRunning) {
                speedTestMode = true;
                xSemaphoreGive(speedTestSemaphore);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ==================== LOG BUFFER ====================
void resetSpeedTestLogs() { logBufferCount = 0; Serial.println("Speed test logs reset."); }

void printBufferLogs() {
    Serial.println("==== RAM Buffer Logs ====");
    if (logBufferCount == 0) { Serial.println("No logs."); return; }
    const char* typeStr[] = {"BACK","FWD","L_TURN","R_TURN"};
    for (uint8_t i = 0; i < logBufferCount; i++) {
        speed_test_log_t &log = logBuffer[i];
        uint8_t type = log.isForward & 0x03;
        Serial.printf("Log %d: %s L_V=%.2f R_V=%.2f L_S=%.2f R_S=%.2f YR=%.2f\n",
                      i, typeStr[type], log.leftVoltage, log.rightVoltage,
                      log.leftSpeed, log.rightSpeed, log.yawRate);
    }
    Serial.println("===========================");
}

void dumpSpeedTestLogs() { printBufferLogs(); }

// ==================== SETUP ====================
void setup() {
    Serial.begin(115200);
    delay(2000);

    ahrsMutex          = xSemaphoreCreateMutex();
    sysPwrMutex        = xSemaphoreCreateMutex();
    speedTestSemaphore = xSemaphoreCreateBinary();
    sysEventGroup      = xEventGroupCreate();
    poseMutex          = xSemaphoreCreateMutex();
    plannerMutex       = xSemaphoreCreateMutex();
    finalMutex         = xSemaphoreCreateMutex();
    manualMutex        = xSemaphoreCreateMutex();
    obstacleMutex      = xSemaphoreCreateMutex();
    safeCmdMutex       = xSemaphoreCreateMutex();
    i2cMutex           = xSemaphoreCreateMutex();

    if (!ahrsMutex || !sysPwrMutex || !speedTestSemaphore || !sysEventGroup ||
        !poseMutex || !plannerMutex || !finalMutex || !manualMutex ||
        !obstacleMutex || !safeCmdMutex || !i2cMutex) {
        Serial.println("Mutex/EventGroup creation failed!");
        while (1);
    }

    if (!initAllHardware(&hardwareSensorStatus)) {
        Serial.printf("HW init failed (%d)\n", hardwareSensorStatus);
        while (1);
    }
    hardwareInitialized = true;
    xEventGroupSetBits(sysEventGroup, EVT_RUNNING);

    xTaskCreate(ahrsCalibrationTask,  "Calibration",   2048, NULL, 8, NULL);
    xTaskCreate(tofTask,              "TOF",           4096, NULL, 7, &tofHandle);
    xTaskCreate(ahrsTask,             "AHRS",          2048, NULL, 6, &ahrsHandle);
    xTaskCreate(motionSensorTask,     "MotionSensor",  1024, NULL, 5, &motionSensorHandle);
    xTaskCreate(obstacleWatchdogTask, "ObsWatch",      2048, NULL, 4, NULL);
    xTaskCreate(motionControllerTask, "MotionCtrl",    2048, NULL, 3, NULL);
    xTaskCreate(manualControlsTask,   "Manual",        2048, NULL, 3, NULL);
    xTaskCreate(powerTask,            "Power",         1024, NULL, 2, &powerHandle);

    Serial.println("All tasks created. Starting scheduler.");
    vTaskStartScheduler();
}

void loop() {}

// ==================== HARDWARE INIT ====================
bool initAllHardware(uint8_t* errorCode) {
    pinMode(LED_BUILTIN, OUTPUT);

    Wire.setSDA(SDA_PIN);
    Wire.setSCL(SCL_PIN);
    Wire.begin();
    Wire.setClock(400000);
    Wire.setTimeout(20);

    delay(100);

    if (!ahrs.begin()) {
        Serial.println("AHRS init failed!");
        *errorCode = 1; return false;
    }
    Serial.println("IMU initialized.");

    Wire.setTimeout(20);

    if (!initMotorDrivers()) {
        Serial.println("Motor driver init failed");
        *errorCode = 2; return false;
    }

    encoderManager.setDebounceBits(6);
    if (!encoderManager.addEncoder(ENCODER_PIN_1) ||
        !encoderManager.addEncoder(ENCODER_PIN_2) ||
        !encoderManager.addEncoder(ENCODER_PIN_3) ||
        !encoderManager.addEncoder(ENCODER_PIN_4)) { *errorCode = 3; return false; }
    if (!encoderManager.begin(5000, TIM4)) { *errorCode = 3; return false; }

    pinMode(JOYSTICK_Y, INPUT_ANALOG);
    pinMode(JOYSTICK_X, INPUT_ANALOG);
    pinMode(SWITCH, INPUT_PULLUP);

    pinMode(hapticMotorA, OUTPUT);
    pinMode(hapticMotorB, OUTPUT);
    pinMode(hapticMotorC, OUTPUT);
    pinMode(hapticMotorD, OUTPUT);


    analogWrite(hapticMotorB, 15);
    analogWrite(hapticMotorA, 15);
    // analogWrite(hapticMotorC, 150);
    // analogWrite(hapticMotorD, 150);
    return true;
}

// ==================== MAG CALIBRATION ====================
void calibrateMagnetometer2D(int duration) {
    Serial.println("2D Mag Cal: Spin robot 360 deg on ground...");

    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        ahrs.mag.startCalibration();
        xSemaphoreGive(i2cMutex);
    }

    const int stepDelayMs = 200;
    int steps = (duration / 2) / stepDelayMs;
    if (steps < 1) steps = 1;
    const float voltage = 8.0f;

    for (int dir = 0; dir < 2; dir++) {
        float lv = (dir == 0) ?  voltage : -voltage;
        float rv = (dir == 0) ? -voltage :  voltage;

        encoderManager.setDirection(0, lv >= 0);
        encoderManager.setDirection(1, lv >= 0);
        encoderManager.setDirection(2, rv >= 0);
        encoderManager.setDirection(3, rv >= 0);
        setLeftMotorsVoltage((int)lv);
        setRightMotorsVoltage((int)rv);

        for (int i = 0; i < steps; i++) {
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                ahrs.mag.updateCalibration();
                xSemaphoreGive(i2cMutex);
            }
            vTaskDelay(pdMS_TO_TICKS(stepDelayMs));
        }

        setLeftMotorsVoltage(0);
        setRightMotorsVoltage(0);
        encoderManager.printAllTicks();
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        ahrs.mag.endCalibration();
        xSemaphoreGive(i2cMutex);
    }

    encoderManager.resetAllTicks();
    for (int i = 0; i < 4; i++) encoderManager.setDirection(i, 1);
    Serial.println("Mag Calibration done.");
    encoderManager.printDebugInfo();
}