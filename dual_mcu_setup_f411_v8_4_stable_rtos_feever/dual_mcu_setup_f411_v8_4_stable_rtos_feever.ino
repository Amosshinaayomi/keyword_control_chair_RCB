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

// ==================== HAPTIC FEEDBACK ====================
// hapticMotorB = left side, hapticMotorA = right side
#define HAPTIC_MAX_PWM       150    // full intensity (0–255), capped per request
#define HAPTIC_NEAR_MM       150.0f // distance at which haptic saturates at MAX
#define HAPTIC_FAR_MM        500.0f // distance at which haptic starts to fade in
#define HAPTIC_MODE_PULSE    90     // gentle pulse PWM on mode change
#define HAPTIC_MODE_PULSE_MS 300    // pulse duration

// ==================== MODE SETTLE ====================
#define MODE_SETTLE_MS       2000   // idle period during mode transition

// ==================== OBSTACLE BACKOFF ====================
#define BACKOFF_SPEED        -0.20f   // m/s, reverse speed while escaping a front wall
#define BACKOFF_TIMEOUT_MS    3000    // safety: max time to reverse before hard halt
#define BACKOFF_SETTLE_MS      250    // pause after clearing before resuming control

// ==================== HEADING LOCK SETTLE ====================
#define HEADING_LOCK_DELTA_DEG  1.0f   // max yaw variation considered "stable"
#define HEADING_LOCK_SETTLE_MS  500    // stable duration required before locking

// ==================== CONTROL MODE ====================
enum ControlMode : uint8_t {
    MODE_MANUAL     = 0,
    MODE_AUTONOMOUS = 1
};
volatile ControlMode currentMode       = MODE_MANUAL;
volatile bool        modeSwitchPending = false;

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
    bool  flValid    = false;
    bool  frValid    = false;
    uint32_t flLastValid_ms = 0;
    uint32_t frLastValid_ms = 0;
    uint32_t timestamp_ms   = 0;
};

enum RobotState : uint8_t {
    ROBOT_IDLE        = 0,
    ROBOT_FORWARD     = 1,
    ROBOT_BACKWARD    = 2,
    ROBOT_TURN_LEFT   = 3,
    ROBOT_TURN_RIGHT  = 4,
    ROBOT_AVOID_LEFT  = 5,
    ROBOT_AVOID_RIGHT = 6,
    ROBOT_HALTED      = 7,
    ROBOT_FAILSAFE    = 8
};

struct SafeCommand {
    float linear       = 0.0f;
    float angular      = 0.0f;
    bool  active       = false;
    RobotState state   = ROBOT_IDLE;
    bool  useHeading   = false;
    float targetHeading_rad = 0.0f;
    uint32_t timestamp_ms = 0;
};

struct RemoteCommand {
    uint8_t  cmd          = CMD_NONE;
    bool     active       = false;
    uint32_t timestamp_ms = 0;
};

Pose           currentPose;    SemaphoreHandle_t poseMutex;
PlannerCommand plannerCmd;     SemaphoreHandle_t plannerMutex;
FinalCommand   finalCmd;       SemaphoreHandle_t finalMutex;
ManualCommand  manualCmd;      SemaphoreHandle_t manualMutex;
ObstacleData   obstacleData;   SemaphoreHandle_t obstacleMutex;
SafeCommand    safeCmd;        SemaphoreHandle_t safeCmdMutex;
RemoteCommand  remoteCmd;      SemaphoreHandle_t remoteMutex;

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
             powerHandle = NULL, motionControllerHandle = NULL, tofHandle = NULL,
             commsHandle = NULL;

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
void commsTask(void*);
void lShapePathTask(void*);

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float getYawDeg() {
    float y;
    xSemaphoreTake(ahrsMutex, portMAX_DELAY);
    y = ahrsData.yaw;
    xSemaphoreGive(ahrsMutex);
    return y;
}

float angleDiffDeg(float target, float current) {
    float d = target - current;
    while (d > 180.0f)  d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

// ---- Haptic intensity from distance ----
//   Strictly 0 while mm > HAPTIC_FAR_MM (500 mm)
//   HAPTIC_MAX_PWM at <= HAPTIC_NEAR_MM
//   Linear in between
int hapticFromDistance(float mm) {
    if (mm <= 0.0f)            return 0;
    if (mm >  HAPTIC_FAR_MM)   return 0;   // disengaged beyond 500 mm
    if (mm <= HAPTIC_NEAR_MM)  return HAPTIC_MAX_PWM;
    float t = (HAPTIC_FAR_MM - mm) / (HAPTIC_FAR_MM - HAPTIC_NEAR_MM);
    return (int)(t * HAPTIC_MAX_PWM);
}

// ==================== COMMS TASK (F4 -> S3) ====================
void commsTask(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);

    uint8_t lastFlags = 0xFF;

    for (;;) {
        if (!isSystemReady()) {
            lastWake = xTaskGetTickCount();
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        SafeCommand sc;
        xSemaphoreTake(safeCmdMutex, portMAX_DELAY);
        sc = safeCmd;
        xSemaphoreGive(safeCmdMutex);

        uint8_t flags = 0;
        switch (sc.state) {
            case ROBOT_IDLE:        flags = 0; break;
            case ROBOT_FORWARD:     flags = ST_FORWARD; break;
            case ROBOT_BACKWARD:    flags = ST_BACKWARD; break;
            case ROBOT_TURN_LEFT:   flags = ST_TURN_LEFT; break;
            case ROBOT_TURN_RIGHT:  flags = ST_TURN_RIGHT; break;
            case ROBOT_AVOID_LEFT:  flags = ST_OBSTACLE_R | ST_TURN_LEFT; break;
            case ROBOT_AVOID_RIGHT: flags = ST_OBSTACLE_L | ST_TURN_RIGHT; break;
            case ROBOT_HALTED:      flags = ST_OBSTACLE_F | ST_HALTED; break;
            case ROBOT_FAILSAFE:
            default:
                if      (sc.linear  >  0.01f) flags = ST_FORWARD;
                else if (sc.linear  < -0.01f) flags = ST_BACKWARD;
                else if (sc.angular >  0.01f) flags = ST_TURN_LEFT;
                else if (sc.angular < -0.01f) flags = ST_TURN_RIGHT;
                else                          flags = 0;
                break;
        }

        if (flags != lastFlags) {
            sendStatus(flags);
            lastFlags = flags;
            Serial.printf("[COMMS] tx flags=0x%02X (state=%d mode=%d)\n",
                          flags, (int)sc.state, (int)currentMode);
        }

        vTaskDelayUntil(&lastWake, period);
    }
}

// ==================== AHRS TASK ====================
void ahrsTask(void *pvParameters) {
    const TickType_t period = pdMS_TO_TICKS(5);
    uint32_t updateCount = 0, lastPrint = millis();

    for (;;) {
        if (!isSystemReady()) {
            lastPrint = millis(); updateCount = 0;
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
            ahrs.update(); updateCount++;
            xSemaphoreGive(i2cMutex);
        }
        uint32_t now = millis();
        if (now - lastPrint >= 1000) {
            Serial.printf("[AHRS-RATE] %lu updates | heap=%lu\n",
                          (unsigned long)updateCount,
                          (unsigned long)xPortGetFreeHeapSize());
            updateCount = 0; lastPrint = now;
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
                sensorReady[i] = true; Serial.println("OK");
            } else {
                sensorReady[i] = false; Serial.println("FAILED");
            }
            delay(10);
        }
        xSemaphoreGive(i2cMutex);
    }

    int readyCount = 0;
    for (int i = 0; i < 4; i++) if (sensorReady[i]) readyCount++;
    Serial.printf("[TOF] %d/4 sensors ready\n", readyCount);

    Wire.setClock(400000);
    Wire.setTimeout(20);

    const TickType_t period = pdMS_TO_TICKS(200);
    TickType_t lastWake = xTaskGetTickCount();
    uint32_t flLastValid = 0, frLastValid = 0;

    for (;;) {
        uint32_t t_start = micros();
        if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
            tofBurstActive = true;
            ObstacleData local;
            local.timestamp_ms = millis();

            uint16_t raw_mm[4] = {0,0,0,0};
            uint8_t  raw_st[4] = {255,255,255,255};

            for (int i = 0; i < 4; i++) {
                if (!sensorReady[i]) continue;
                VL53L0X_RangingMeasurementData_t measure;
                uint32_t t_sensor = micros();
                VL53L0X_Error err = lox[i].rangingTest(&measure, false);
                uint32_t dur_us = micros() - t_sensor;
                raw_mm[i] = 0; raw_st[i] = 255;

                if (err != VL53L0X_ERROR_NONE) {
                    Serial.printf("[TOF] sensor %d err=%d took %luus\n",
                                  i, (int)err, (unsigned long)dur_us);
                    continue;
                }

                uint8_t st = measure.RangeStatus;
                uint16_t mm = measure.RangeMilliMeter;
                raw_mm[i] = mm; raw_st[i] = st;

                bool valid = (st == 0 || st == 1 || st == 2 || st == 3) &&
                             (mm > 30) && (mm < 8000);
                if (!valid) continue;

                switch (i) {
                    case 2: local.frontRight = mm; local.frValid = true; frLastValid = millis(); break;
                    case 3: local.frontLeft  = mm; local.flValid = true; flLastValid = millis(); break;
                    case 0: local.backLeft  = mm; break;
                    case 1: local.backRight = mm; break;
                    default: break;
                }
            }
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

// ==================== OBSTACLE WATCHDOG ====================
void obstacleWatchdogTask(void *pvParameters) {
    const TickType_t period = pdMS_TO_TICKS(200);
    TickType_t lastWake = xTaskGetTickCount();

    const float ENTER_MM         = 200.0f;
    const float EXIT_MM          = 300.0f;
    const float AVOID_STEP_DEG   = 3.0f;
    const float TURN_DETECT_DEG  = 5.0f;
    const uint32_t SENSOR_STALE_MS = 600;

    uint8_t obstState = 0;
    bool    avoidActive = false;
    float   avoidHeading_deg = 0.0f;

    uint32_t modeChangePulseUntil = 0;
    uint32_t modeIdleUntil        = 0;
    ControlMode lastModeSeen      = currentMode;

    RobotState lastPubState = (RobotState)0xFF;

    // ---- Backoff state ----
    bool     backoffActive      = false;
    uint32_t backoffStartMs     = 0;
    uint32_t backoffSettleUntil = 0;

    for (;;) {
        if (!isSystemReady()) {
            lastWake = xTaskGetTickCount();
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        // ---- Mode-switch from double-tap ----
        if (modeSwitchPending) {
            modeSwitchPending = false;
            if (currentMode == MODE_MANUAL) {
                currentMode = MODE_AUTONOMOUS;
                Serial.println("[MODE] -> AUTONOMOUS");
            } else {
                currentMode = MODE_MANUAL;
                Serial.println("[MODE] -> MANUAL");
                xSemaphoreTake(plannerMutex, portMAX_DELAY);
                plannerCmd.linear  = 0.0f;
                plannerCmd.angular = 0.0f;
                plannerCmd.valid   = false;
                xSemaphoreGive(plannerMutex);
            }
        }

        // ---- Detect ANY mode change (double-tap OR programmatic) ----
        if (currentMode != lastModeSeen) {
            lastModeSeen = currentMode;
            modeChangePulseUntil = millis() + HAPTIC_MODE_PULSE_MS;
            modeIdleUntil        = millis() + MODE_SETTLE_MS;
            Serial.printf("[MODE] change -> %d, settling for %lums\n",
                          (int)currentMode, (unsigned long)MODE_SETTLE_MS);
        }

        ObstacleData obs;
        xSemaphoreTake(obstacleMutex, portMAX_DELAY);
        obs = obstacleData;
        xSemaphoreGive(obstacleMutex);

        ManualCommand  raw_manual;
        PlannerCommand raw_planner;
        xSemaphoreTake(manualMutex, portMAX_DELAY);
        raw_manual = manualCmd;
        xSemaphoreGive(manualMutex);
        xSemaphoreTake(plannerMutex, portMAX_DELAY);
        raw_planner = plannerCmd;
        xSemaphoreGive(plannerMutex);

        uint32_t now = millis();
        float currentYawDeg = getYawDeg();

        bool flHealthy = (obs.flLastValid_ms > 0) && (now - obs.flLastValid_ms < SENSOR_STALE_MS);
        bool frHealthy = (obs.frLastValid_ms > 0) && (now - obs.frLastValid_ms < SENSOR_STALE_MS);
        bool sensorFault = !flHealthy || !frHealthy;

        RobotState newState = ROBOT_IDLE;
        SafeCommand out;
        out.timestamp_ms = now;

        if (sensorFault) {
            newState = ROBOT_FAILSAFE;
            out.useHeading = false;
            if (currentMode == MODE_MANUAL) {
                out.linear  = raw_manual.linear;
                out.angular = raw_manual.angular;
                out.active  = raw_manual.active;
            } else {
                out.linear  = 0;
                out.angular = 0;
                out.active  = false;
            }
            avoidActive = false;
            obstState = 0;

            // A sensor fault cancels any in-progress backoff
            if (backoffActive) {
                backoffActive      = false;
                backoffSettleUntil = 0;
            }
        } else {
            bool flBlocked = obs.flValid && obs.frontLeft  > 0.0f && obs.frontLeft  <= ENTER_MM;
            bool frBlocked = obs.frValid && obs.frontRight > 0.0f && obs.frontRight <= ENTER_MM;
            bool flClear   = obs.flValid && obs.frontLeft  > 0.0f && obs.frontLeft  >= EXIT_MM;
            bool frClear   = obs.frValid && obs.frontRight > 0.0f && obs.frontRight >= EXIT_MM;

            switch (obstState) {
                case 0:
                    if (flBlocked && frBlocked) obstState = 3;
                    else if (flBlocked)         obstState = 1;
                    else if (frBlocked)         obstState = 2;
                    break;
                case 1:
                    if (frBlocked)              obstState = 3;
                    else if (flClear)           obstState = 0;
                    break;
                case 2:
                    if (flBlocked)              obstState = 3;
                    else if (frClear)           obstState = 0;
                    break;
                case 3:
                    if (flClear && frClear)     obstState = 0;
                    break;
            }

            if (obstState == 3) {
                // First entry into the blocked state
                if (!backoffActive) {
                    backoffActive  = true;
                    backoffStartMs = now;
                    Serial.println("[OBS] Both front sensors blocked — BACKOFF");
                }

                bool backoffTimedOut = (now - backoffStartMs) >= BACKOFF_TIMEOUT_MS;

                // Hold heading; drive backward (or halt if timed out)
                out.useHeading        = true;
                out.targetHeading_rad = currentYawDeg * DEG_TO_RAD;
                out.angular           = 0.0f;
                out.active            = true;

                if (backoffTimedOut) {
                    newState   = ROBOT_HALTED;
                    out.linear = 0.0f;
                    Serial.println("[OBS] Backoff timeout — HALT");
                } else {
                    newState   = ROBOT_BACKWARD;
                    out.linear = BACKOFF_SPEED;
                }

                avoidActive = false;

                // Auto mode aborts on a wall (unchanged)
                if (currentMode == MODE_AUTONOMOUS) {
                    currentMode = MODE_MANUAL;
                    Serial.println("[OBS] WALL — auto aborted, switching to MANUAL");
                    xSemaphoreTake(plannerMutex, portMAX_DELAY);
                    plannerCmd.linear  = 0.0f;
                    plannerCmd.angular = 0.0f;
                    plannerCmd.valid   = false;
                    xSemaphoreGive(plannerMutex);
                }

                // Manual override: only allow the user to reverse FASTER,
                // never to cancel the backoff or move forward.
                if (currentMode == MODE_MANUAL && raw_manual.linear < BACKOFF_SPEED) {
                    out.linear = raw_manual.linear;
                }
            }
            else if (obstState == 1 || obstState == 2) {
                // A partial-avoid takes over: cancel any backoff
                if (backoffActive) {
                    backoffActive      = false;
                    backoffSettleUntil = 0;
                }

                if (!avoidActive) {
                    avoidHeading_deg = currentYawDeg;
                    avoidActive = true;
                }

                // ---- FIX: invert steering so the robot turns AWAY from the
                //      blocked side.  obstState 1 = front-LEFT blocked -> turn
                //      RIGHT (positive step); obstState 2 = front-RIGHT blocked
                //      -> turn LEFT (negative step). ----
                float step = (obstState == 1) ? +AVOID_STEP_DEG : -AVOID_STEP_DEG;
                avoidHeading_deg += step;
                if (avoidHeading_deg >  180.0f) avoidHeading_deg -= 360.0f;
                if (avoidHeading_deg < -180.0f) avoidHeading_deg += 360.0f;

                // Labels describe the direction of travel (matching the fix above)
                newState = (obstState == 1) ? ROBOT_AVOID_RIGHT : ROBOT_AVOID_LEFT;
                out.useHeading = true;
                out.targetHeading_rad = avoidHeading_deg * DEG_TO_RAD;
                out.linear = 0.0f;
                out.angular = 0.0f;
                out.active = true;
            }
            else {
                if (backoffActive) {
                    backoffActive      = false;
                    backoffSettleUntil = now + BACKOFF_SETTLE_MS;
                    Serial.println("[OBS] Backoff clear — halting to settle");
                }
                avoidActive = false;

                if (currentMode == MODE_AUTONOMOUS) {
                    out.useHeading = raw_planner.valid;
                    out.targetHeading_rad = raw_planner.heading_deg * DEG_TO_RAD;
                    out.linear  = raw_planner.linear;
                    out.angular = 0.0f;
                    out.active  = raw_planner.valid;

                    float hErr = angleDiffDeg(raw_planner.heading_deg, currentYawDeg);
                    if (raw_planner.valid && fabsf(hErr) > TURN_DETECT_DEG) {
                        newState = (hErr > 0) ? ROBOT_TURN_LEFT : ROBOT_TURN_RIGHT;
                    } else if (raw_planner.linear > 0.01f) {
                        newState = ROBOT_FORWARD;
                    } else if (raw_planner.linear < -0.01f) {
                        newState = ROBOT_BACKWARD;
                    } else {
                        newState = ROBOT_IDLE;
                    }
                } else {
                    out.useHeading = false;
                    out.linear  = raw_manual.linear;
                    out.angular = raw_manual.angular;
                    out.active  = raw_manual.active;

                    if      (raw_manual.linear  >  0.01f) newState = ROBOT_FORWARD;
                    else if (raw_manual.linear  < -0.01f) newState = ROBOT_BACKWARD;
                    else if (raw_manual.angular >  0.01f) newState = ROBOT_TURN_LEFT;
                    else if (raw_manual.angular < -0.01f) newState = ROBOT_TURN_RIGHT;
                    else                                   newState = ROBOT_IDLE;
                }
            }
        }

        out.state = newState;

        // ---- Force idle while mode or backoff is settling ----
        if (now < modeIdleUntil || now < backoffSettleUntil) {
            newState = ROBOT_IDLE;
            out.useHeading = false;
            out.linear  = 0.0f;
            out.angular = 0.0f;
            out.active  = false;
            out.state   = ROBOT_IDLE;
        }

        // ================= HAPTIC FEEDBACK =================
        // Strictly disengaged while the nearest obstacle on that side is
        // farther than HAPTIC_FAR_MM (500 mm).  Mode pulse is also gated on
        // an obstacle being within range, per request.
        {
            bool nearLeft  = (obs.frontLeft  > 0.0f) && (obs.frontLeft  <= HAPTIC_FAR_MM);
            bool nearRight = (obs.frontRight > 0.0f) && (obs.frontRight <= HAPTIC_FAR_MM);

            int leftVib  = nearLeft  ? hapticFromDistance(obs.frontLeft)  : 0;
            int rightVib = nearRight ? hapticFromDistance(obs.frontRight) : 0;

            if (obstState == 3) {
                leftVib  = HAPTIC_MAX_PWM;
                rightVib = HAPTIC_MAX_PWM;
            }

            if (now < modeChangePulseUntil && (nearLeft || nearRight)) {
                leftVib  = HAPTIC_MODE_PULSE;
                rightVib = HAPTIC_MODE_PULSE;
            }

            analogWrite(hapticMotorB, leftVib);
            analogWrite(hapticMotorA, rightVib);
        }
        // ===================================================

        xSemaphoreTake(safeCmdMutex, portMAX_DELAY);
        safeCmd = out;
        xSemaphoreGive(safeCmdMutex);

        if (newState != lastPubState) {
            Serial.printf("[OBS] %d -> %d (mode=%d | FL=%.0f FR=%.0f | tgtH=%.1f)\n",
                          (int)lastPubState, (int)newState,
                          (int)currentMode,
                          obs.frontLeft, obs.frontRight,
                          out.targetHeading_rad * RAD_TO_DEG);
            lastPubState = newState;
        }

        vTaskDelayUntil(&lastWake, period);
    }
}

// ==================== MANUAL CONTROLS ====================
void manualControlsTask(void *parameters) {
    pinMode(JOYSTICK_X, INPUT_ANALOG);
    pinMode(JOYSTICK_Y, INPUT_ANALOG);
    pinMode(SWITCH, INPUT_PULLUP);
    analogReadResolution(10);

    const int DEADZONE = 17;
    const float LIN_SPEED = 0.3f;
    const float ANG_SPEED = 0.4f;


    const int CAL_SAMPLES = 50;
    long sumX = 0, sumY = 0;

    while(!isSystemReady()) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    for (int i = 0; i < CAL_SAMPLES; i++) {
        sumX += analogRead(JOYSTICK_X);
        sumY += analogRead(JOYSTICK_Y);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    const int CENTER_X = (int)(sumX / CAL_SAMPLES);
    const int CENTER_Y = (int)(sumY / CAL_SAMPLES);
    Serial.printf("[MANUAL] Joystick center calibrated: X=%d Y=%d\n", CENTER_X, CENTER_Y);

    const uint32_t DBL_MIN_GAP_MS = 750;
    const uint32_t DBL_MAX_GAP_MS = 2000;
    const uint32_t SW_DEBOUNCE_MS = 50;

    bool     sw_raw = false, sw_stable = false;
    uint32_t sw_lastChange = 0;
    bool     firstPress = false;
    uint32_t firstPressMs = 0;

    uint32_t lastPrint = 0;

    for (;;) {
        int xRaw = 0, yRaw = 0;
        for (int i = 0; i < 3; i++) {
            xRaw += analogRead(JOYSTICK_X);
            yRaw += analogRead(JOYSTICK_Y);
            vTaskDelay(pdMS_TO_TICKS(2));
        }

        int x = xRaw / 5, y = yRaw / 5;
        if (abs(x - CENTER_X) < DEADZONE) x = CENTER_X;
        if (abs(y - CENTER_Y) < DEADZONE) y = CENTER_Y;

        int dx = x - CENTER_X, dy = y - CENTER_Y;
        int adx = abs(dx), ady = abs(dy);
        float linear = 0.0f, angular = 0.0f;
        // if(adx > 0 && (ady >= CENTER_Y - DEADZONE && ady <= CENTER_Y + DEADZONE))
        // {
        //     angular = (dx > 0) ? -ANG_SPEED : +ANG_SPEED;
        //     linear = 0.0f;
        // } else if (ady > 0 && (adx >= CENTER_X - DEADZONE && adx <= CENTER_X + DEADZONE)){
        //     linear  = (dy > 0) ? +LIN_SPEED : -LIN_SPEED;
        //     angular = 0.0f;
        // } else {
        //     linear = 0.0f;
        //     angular = 0.0f;
        // }
        if (adx >= DEADZONE || ady >= DEADZONE) {
            if (adx > ady){ 
                angular = (dx > 0) ? -ANG_SPEED : +ANG_SPEED;
                linear = 0;
            }
            else {
                linear  = (dy > 0) ? +LIN_SPEED : -LIN_SPEED;
                angular = 0;
            }
        }

        xSemaphoreTake(manualMutex, portMAX_DELAY);
        manualCmd.linear  = linear;
        manualCmd.angular = angular;
        manualCmd.active  = (linear != 0.0f) || (angular != 0.0f);
        xSemaphoreGive(manualMutex);

        bool rawNow = (digitalRead(SWITCH) == LOW);
        uint32_t now = millis();
        if (rawNow != sw_raw) { sw_raw = rawNow; sw_lastChange = now; }

        if ((now - sw_lastChange) >= SW_DEBOUNCE_MS && sw_stable != sw_raw) {
            sw_stable = sw_raw;
            if (sw_stable) {
                if (!firstPress) {
                    firstPress = true;
                    firstPressMs = now;
                } else {
                    uint32_t gap = now - firstPressMs;
                    if (gap >= DBL_MIN_GAP_MS && gap <= DBL_MAX_GAP_MS) {
                        modeSwitchPending = true;
                        firstPress = false;
                        Serial.printf("[MANUAL] double-tap (gap=%lums)\n", (unsigned long)gap);
                    } else if (gap > DBL_MAX_GAP_MS) {
                        firstPressMs = now;
                    }
                }
            }
        }

        if (now - lastPrint >= 1500) {
            lastPrint = now;
            Serial.printf("[MANUAL] x=%4d y=%4d | lin=%+5.2f ang=%+5.2f | mode=%d\n",
                          x, y, linear, angular, (int)currentMode);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ==================== L-SHAPE PATH TASK ====================
void lShapePathTask(void *pvParameters) {
    while (!isSystemReady()) vTaskDelay(pdMS_TO_TICKS(100));
    Serial.println("[PATH] L-shape task ready");

    auto setPlanner = [](float lin, float heading_deg, bool valid) {
        xSemaphoreTake(plannerMutex, portMAX_DELAY);
        plannerCmd.linear      = lin;
        plannerCmd.angular     = 0.0f;
        plannerCmd.heading_deg = heading_deg;
        plannerCmd.valid       = valid;
        plannerCmd.timestamp_ms = millis();
        xSemaphoreGive(plannerMutex);
    };

    auto waitMs = [](uint32_t ms) -> bool {
        uint32_t t0 = millis();
        while (millis() - t0 < ms) {
            if (currentMode != MODE_AUTONOMOUS) return false;
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        return true;
    };

    auto waitHeading = [](float target_deg, float tol_deg, uint32_t timeout_ms) -> bool {
        uint32_t t0 = millis();
        while (millis() - t0 < timeout_ms) {
            if (currentMode != MODE_AUTONOMOUS) return false;
            float err = angleDiffDeg(target_deg, getYawDeg());
            if (fabsf(err) < tol_deg) return true;
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        return false;
    };

    for (;;) {
        if (currentMode != MODE_AUTONOMOUS) {
            setPlanner(0.0f, 0.0f, false);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // --- Wait for mode settle to finish before starting ---
        vTaskDelay(pdMS_TO_TICKS(MODE_SETTLE_MS + 200));

        if (currentMode != MODE_AUTONOMOUS) continue;

        Serial.println("[PATH] AUTO — L-shape start");
        vTaskDelay(pdMS_TO_TICKS(500));
        if (currentMode != MODE_AUTONOMOUS) continue;

        float startYaw = getYawDeg();
        Serial.printf("[PATH] startYaw = %.1f deg\n", startYaw);

        // ---- Segment 1: forward on start heading ----
        setPlanner(0.30f, startYaw, true);
        if (!waitMs(3000)) { setPlanner(0.0f, startYaw, false); continue; }

        setPlanner(0.0f, startYaw, false);
        if (!waitMs(300)) continue;

        // ---- Turn left 90 deg via heading setpoint ----
        float turnTarget = startYaw + 90.0f;
        if (turnTarget >  180.0f) turnTarget -= 360.0f;
        if (turnTarget < -180.0f) turnTarget += 360.0f;

        Serial.printf("[PATH] turn to %.1f deg\n", turnTarget);
        setPlanner(0.0f, turnTarget, true);
        if (!waitHeading(turnTarget, 3.0f, 5000)) {
            Serial.println("[PATH] turn timeout");
            setPlanner(0.0f, turnTarget, false);
            continue;
        }
        Serial.println("[PATH] turn complete");

        if (!waitMs(300)) continue;

        // ---- Segment 2: forward on new heading ----
        setPlanner(0.30f, turnTarget, true);
        if (!waitMs(3000)) { setPlanner(0.0f, turnTarget, false); continue; }

        // ---- Done: exit autonomous mode ----
        setPlanner(0.0f, turnTarget, false);
        Serial.println("[PATH] L-shape complete — returning to MANUAL");

        currentMode = MODE_MANUAL;

        xSemaphoreTake(plannerMutex, portMAX_DELAY);
        plannerCmd.linear  = 0.0f;
        plannerCmd.angular = 0.0f;
        plannerCmd.valid   = false;
        xSemaphoreGive(plannerMutex);

        vTaskDelay(pdMS_TO_TICKS(500));
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

    // ---- Deferred heading-lock state (waits for yaw to settle post-calibration) ----
    static bool     yawLockSampleInit = false;
    static float    yawLockSample     = 0.0f;
    static uint32_t yawLockStableSince = 0;

    for (;;) {
        if (!isSystemReady()) {
            lastWake = xTaskGetTickCount();
            // Reset lock state when the system goes "not ready" so that the
            // wait-for-settle process restarts cleanly after (re)calibration.
            headingInitialized = false;
            yawLockSampleInit  = false;
            yawLockStableSince = 0;
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

        // ---- Wait for yaw to settle BEFORE locking the heading setpoint ----
        if (!headingInitialized) {
            uint32_t nowMs = millis();

            if (!yawLockSampleInit) {
                yawLockSample     = localData.yaw;
                yawLockSampleInit = true;
                yawLockStableSince = nowMs;   // start the stability window
            }

            float dyaw = fabsf(angleDiffDeg(localData.yaw, yawLockSample));
            if (dyaw >= HEADING_LOCK_DELTA_DEG) {
                // Yaw moved — restart stability window with the new sample
                yawLockSample      = localData.yaw;
                yawLockStableSince = nowMs;
            }

            bool stable = (nowMs - yawLockStableSince) >= HEADING_LOCK_SETTLE_MS;

            if (!stable) {
                // Hold motors at 0V while waiting for the yaw to settle so the
                // chair does not creep / turn toward a transient heading.
                setLeftMotorsVoltage(0.0f);
                setRightMotorsVoltage(0.0f);
                vTaskDelayUntil(&lastWake, period);
                continue;
            }

            // Yaw is stable — capture it as the initial heading setpoint.
            // Use the freshest yaw (not the historical sample).
            motionController.setHeadingSetpoint(yawRad, true);
            motionController.setStraight(0);
            headingInitialized = true;
            lastCmdWasAngular  = false;
            Serial.printf("[MCTRL] Heading locked to yaw %.1f deg (stable %.0f ms).\n",
                          localData.yaw, (double)HEADING_LOCK_SETTLE_MS);
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

        if (cmd.useHeading) {
            motionController.setHeadingSetpoint(cmd.targetHeading_rad, true);
            motionController.setStraight(cmd.linear);
            lastCmdWasAngular = false;
        } else if (fabsf(cmd.angular) > 0.01f) {
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
        static bool first = true;
        const float alpha = 0.15f;
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
    const int REQUIRED = 20;
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
    remoteMutex        = xSemaphoreCreateMutex();
    i2cMutex           = xSemaphoreCreateMutex();

    if (!ahrsMutex || !sysPwrMutex || !speedTestSemaphore || !sysEventGroup ||
        !poseMutex || !plannerMutex || !finalMutex || !manualMutex ||
        !obstacleMutex || !safeCmdMutex || !remoteMutex || !i2cMutex) {
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
    xTaskCreate(commsTask,            "Comms",         2048, NULL, 5, &commsHandle);
    xTaskCreate(motionSensorTask,     "MotionSensor",  1024, NULL, 5, &motionSensorHandle);
    xTaskCreate(obstacleWatchdogTask, "ObsWatch",      2048, NULL, 4, NULL);
    xTaskCreate(motionControllerTask, "MotionCtrl",    2048, NULL, 3, NULL);
    xTaskCreate(manualControlsTask,   "Manual",        2048, NULL, 3, NULL);
    xTaskCreate(lShapePathTask,       "LShapePath",    2048, NULL, 2, NULL);
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

    commSerial.begin(115200);
    Serial.println("[COMMS] UART to S3: TX=PA2 RX=PA3 @115200");

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
    analogWrite(hapticMotorA, 0);
    analogWrite(hapticMotorB, 0);

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