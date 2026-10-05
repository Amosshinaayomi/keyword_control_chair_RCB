#pragma once

#include <Arduino.h>
#include <pins.h>
#include "BMI160.h"
#include "QMC5883.h"
#include <Fusion.h>
#include <FreeRTOS.h>
#include <semphr.h>   // for SemaphoreHandle_t
#include <EEPROM.h>

typedef struct {
    unsigned long timestamp_ms;  // Millisecond timestamp
    float accel_g[3];
    float gyro_dps[3];
    // Orientation (Degrees)
    float roll;
    float pitch;
    float yaw;
    float yawRate;
    float yawRateFiltered;
    // Derived Data
    long encoder_ticks[4]; //encoder data
} motionSensorPacket_t; // ahrs data packet





typedef struct  {
    float gyroBias[3];      // from AHRS (deg/s)
    float accelBias[3];     // from AHRS (m/s^2 or g)
    float hardIronOffset[3];    // hard iron offsets (uT)
    int16_t magXMin, magXMax;  // raw counts
    int16_t magYMin, magYMax;
    int16_t magZMin, magZMax;
    float softIronmatrix[3][3];   // soft iron matrix (3x3)
    uint16_t magic;         // e.g., 0x5A5A to indicate valid data
    uint16_t checksum;      // simple XOR of all previous bytes
} CalibrationData_t;

void printAHRSPacket(motionSensorPacket_t data);
// void printPose(const pose_packet_t &pose);
void sendVisualizationData(motionSensorPacket_t data);


class AHRS {
    public:
        AHRS(uint8_t sdaPin = SDA_PIN, uint8_t sclPin = SCL_PIN, uint32_t i2cSpeed = 400000UL);
        BMI160 imu; // Using default I2C address 0x69
        QMC5883 mag;
        // Initialize sensors, calibrate, set up filter – call in setup()
        bool begin();

        // Must be called as often as possible (e.g., every loop iteration)
        void update();

        // Getters for orientation (angles in degrees)
        float getRoll() const;     // degrees
        float getPitch() const;    // degrees
        float getYaw() const;      // degrees (0‑360, heading)
        float getYawRate() const;  // degrees per second
        void getAccel(float accelData[3]) const;
        void getGyro(float gyroData[3]) const;

        // Linear acceleration (body frame, g's)
        void getLinearAcceleration(float &x, float &y, float &z) const;

        // World‑frame linear acceleration (useful for dead reckoning)
        void getWorldLinearAcceleration(float &x, float &y, float &z) const;

        // Temperature in °C
        float getTemperature() const;

        void calibrateIMU();

        // Set magnetic declination (in degrees)
        void setMagneticDeclination(float declinationDeg);
        // (Optional) Manually set calibration biases – useful after EEPROM restore
        void setGyroBias(float x, float y, float z);
        void setAccelBias(float x, float y, float z);



    private:
        // Sensor and Fusion Objects

        FusionAhrs ahrsFilter;
        FusionOffset offsetFilter; // For gyroscope run-time offset correction

        FusionConvention convention = FusionConventionNwu;

        // Calibration data
        float gyroBias[3];
        float accelBias[3];
        float originalGyroBias[3];      // for temperature compensation
        float currentTemperature;
        float referenceTemperature;
        bool temperatureCompEnabled;

        // Last update time for delta calculation
        unsigned long lastUpdateMicros;
        unsigned long lastTempReadMillis;

        
        // Cached outputs
        float roll, pitch, yaw;


        // Cached calibrated sensor values (for getAccel/getGyro)
        float accelFiltered[3];   // ax_g, ay_g, az_g
        float gyroFiltered[3];    // gx_dps, gy_dps, gz_dps
        float yawRate;
        float linearAccBody[3];

        // Magnetic declination (radians)
        float declinationRad;

        // I2C settings
        uint8_t sda, scl;
        uint32_t i2cSpeed;

        // FreeRTOS mutex handle (instead of osMutexId)
        mutable SemaphoreHandle_t dataMutex;


        // Private helpers

        void readTemperature();
        void applyTemperatureCompensation(float currentTemp, float gyroScale);



 };


