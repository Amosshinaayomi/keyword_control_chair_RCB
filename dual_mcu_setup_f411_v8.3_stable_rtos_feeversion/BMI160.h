/* BMI160 ODR VALUES (Output Data Rate)
 * ====================================
 * ACCELEROMETER:
 * 0x08 = 100Hz    | 0x09 = 200Hz
 * 0x0A = 400Hz    | 0x0B = 800Hz  
 * 0x0C = 1600Hz   (Default)
 *
 * GYROSCOPE:
 * 0x08 = 100Hz    | 0x09 = 200Hz
 * 0x0A = 400Hz    | 0x0B = 800Hz
 * 0x0C = 1600Hz   | 0x0D = 3200Hz (Default)
 *
 * Example usage:
 * imu.setAccelRate(0x0B); // 800Hz
 * imu.setGyroRate(0x0A);  // 400Hz
 */

#ifndef BMI160_H
#define BMI160_H

#include <Wire.h>
class BMI160 {
public:
    // Constructor
    BMI160(uint8_t address = 0x69);
    
    // Initialization
    bool begin(uint8_t sdaPin = PB7, uint8_t sclPin = PB6, uint32_t i2cSpeed = 1000000UL);
    bool isConnected();
    
    // Configuration
    void setAccelRange(uint8_t range); // 2, 4, 8, 16g
    void setGyroRange(uint8_t range);  // 250, 500, 1000, 2000 dps
    void setAccelRate(uint8_t rate);   // ODR: 0x08-0x0C (100-1600Hz)
    void setGyroRate(uint8_t rate);    // ODR: 0x08-0x0D (100-3200Hz)
    
    // Data reading
    bool getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
    bool getAccelData(int16_t* ax, int16_t* ay, int16_t* az);
    bool getGyroData(int16_t* gx, int16_t* gy, int16_t* gz);
    
    // Calibration
    void calibrateGyro(uint16_t samples = 500, float* biasX = nullptr, float* biasY = nullptr, float* biasZ = nullptr);
    void calibrateAccel(uint16_t samples = 300, float* biasX = nullptr, float* biasY = nullptr, float* biasZ = nullptr);
    
    // Utility
    void softReset();
    uint8_t getChipID();
    
    // Performance monitoring
    uint32_t getLastReadTime() { return _lastReadTime; }

    float getAccelScaleG() const;  // Returns g's per LSB (e.g., 4.0/32768)
    float getGyroScaleDps() const; // Returns dps per LSB (e.g., 500.0/32768)
    bool getTemperature(float* temperature);
    bool isTemperatureAvailable();
    bool isGyroInNormalMode();
    // Temperature compensation helpers
    float getGyroTempCoefficient() { return 0.05f; } // °/s/K (from datasheet)
    float getAccelTempCoeffcient() { return 0.001f; } // g/K (1.0 mg/K from datasheet)

private:
    uint8_t _address;
    uint32_t _lastReadTime;
    uint8_t _accelRange; // Store the current accelerometer range setting (2, 4, 8, 16)
    uint8_t _gyroRange;  // Store the current gyroscope range setting (250, 500, 1000, 2000)
    // Register addresses
    static const uint8_t BMI160_CHIP_ID       = 0x00;
    static const uint8_t BMI160_GYRO_DATA     = 0x0C;
    static const uint8_t BMI160_ACCEL_DATA    = 0x12;
    static const uint8_t BMI160_CMD           = 0x7E;
    static const uint8_t BMI160_ACCEL_CONFIG  = 0x40;
    static const uint8_t BMI160_ACCEL_RANGE   = 0x41;
    static const uint8_t BMI160_GYRO_CONFIG   = 0x42;
    static const uint8_t BMI160_GYRO_RANGE    = 0x43;
    static const uint8_t BMI160_PMU_STATUS    = 0x03;
    static const uint8_t BMI160_TEMPERATURE_0 = 0X20;
    static const uint8_t BMI160_TEMPERATURE_1 = 0X21;
    // Commands
    static const uint8_t ACCEL_NORMAL_MODE    = 0x11;
    static const uint8_t GYRO_NORMAL_MODE     = 0x15;
    static const uint8_t SOFT_RESET_CMD       = 0xB6;
    

    float _lastTemperature;
    // I2C communication
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t reg, uint8_t* data, uint8_t length);
    
    // Configuration helpers
    bool configureSensor();

    float getAccelLsbPerG() const; // Helper to get LSB per g for current range

};

#endif