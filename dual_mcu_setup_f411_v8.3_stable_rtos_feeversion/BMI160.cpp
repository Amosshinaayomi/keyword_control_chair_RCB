#include "BMI160.h"

BMI160::BMI160(uint8_t address) : _address(address),_accelRange(4), _gyroRange(500), _lastReadTime(0) {
}

bool BMI160::begin(uint8_t sdaPin, uint8_t sclPin, uint32_t i2cSpeed) {
    // Wire.setSDA(sdaPin);
    // Wire.setSCL(sclPin);
    // Wire.setClock(i2cSpeed);
    // Wire.begin();
    delay(10);
    // Serial.println("got here 1");
    // Soft reset
    if (!writeRegister(BMI160_CMD, SOFT_RESET_CMD)) {
        return false;
    }
    // Serial.println("got here 2");
    delay(50);
    
    // Verify chip ID
    uint8_t chipId;
    if (!readRegisters(BMI160_CHIP_ID, &chipId, 1) || chipId != 0xD1) {
        return false;
    }
    // Serial.println("got here 3");
    // Configure sensor with default settings
    return configureSensor();
}

bool BMI160::isConnected() {
    uint8_t chipId;
    return readRegisters(BMI160_CHIP_ID, &chipId, 1) && chipId == 0xD1;
}

bool BMI160::configureSensor() {
    // Default configuration: 1600Hz accel, 3200Hz gyro, ±2g, ±250dps
    
    // Accelerometer: 1600Hz (0x0C), normal mode
    if (!writeRegister(BMI160_ACCEL_CONFIG, 0x2C)) return false;
    if (!writeRegister(BMI160_ACCEL_RANGE, 0x03)) return false; // ±2g
    
    // Gyroscope: 3200Hz (0x0D), normal mode  
    if (!writeRegister(BMI160_GYRO_CONFIG, 0x2D)) return false;
    if (!writeRegister(BMI160_GYRO_RANGE, 0x03)) return false; // ±250dps
    
    // Power on sensors
    if (!writeRegister(BMI160_CMD, ACCEL_NORMAL_MODE)) return false;
    delay(5);
    if (!writeRegister(BMI160_CMD, GYRO_NORMAL_MODE)) return false;
    delay(5);
    
    return true;
}

// =============================================
// RATE CONFIGURATION FUNCTIONS
// =============================================


float BMI160::getAccelLsbPerG() const {
    switch (_accelRange) {
        case 2:  return 16384.0f; // ±2g  -> 16384 LSB/g
        case 4:  return 8192.0f;  // ±4g  -> 8192 LSB/g
        case 8:  return 4096.0f;  // ±8g  -> 4096 LSB/g
        case 16: return 2048.0f;  // ±16g -> 2048 LSB/g
        default: return 8192.0f;  // Default to ±4g if unknown
    }
}
void BMI160::setAccelRate(uint8_t rate) {
    // Validate input (0x08-0x0C per datasheet)
    if (rate < 0x08 || rate > 0x0C) {
        rate = 0x0C; // Default to 1600Hz
    }
    
    // Read current ACCEL_CONFIG register
    uint8_t current;
    if (readRegisters(BMI160_ACCEL_CONFIG, &current, 1)) {
        // Keep the upper 4 bits (filter setting) and set lower 4 bits (ODR)
        uint8_t newValue = (current & 0xF0) | (rate & 0x0F);
        writeRegister(BMI160_ACCEL_CONFIG, newValue);
    }
}

void BMI160::setGyroRate(uint8_t rate) {
    // Validate input (0x08-0x0D per datasheet)
    if (rate < 0x08 || rate > 0x0D) {
        rate = 0x0D; // Default to 3200Hz
    }
    
    // Read current GYRO_CONFIG register
    uint8_t current;
    if (readRegisters(BMI160_GYRO_CONFIG, &current, 1)) {
        // Keep the upper 4 bits (filter setting) and set lower 4 bits (ODR)
        uint8_t newValue = (current & 0xF0) | (rate & 0x0F);
        writeRegister(BMI160_GYRO_CONFIG, newValue);
    }
}

// =============================================
// ACCELEROMETER RANGE HELPER
// =============================================
void BMI160::setAccelRange(uint8_t range) {
    uint8_t value = 0x03; // Default ±2g
    
    switch (range) {
        case 2: value = 0x03; break;
        case 4: value = 0x05; break;
        case 8: value = 0x08; break;
        case 16: value = 0x0C; break;
    }
    
    writeRegister(BMI160_ACCEL_RANGE, value);
    _accelRange = range;
}

// =============================================
// GYROSCOPE RANGE HELPER
// =============================================
void BMI160::setGyroRange(uint8_t range) {
    uint8_t value = 0x03; // Default ±250dps
    
    switch (range) {
        case 250: value = 0x03; break;
        case 500: value = 0x02; break;
        case 1000: value = 0x01; break;
        case 2000: value = 0x00; break;
    }
    
    writeRegister(BMI160_GYRO_RANGE, value);
    _gyroRange = range;
}

// =============================================
// DATA READING FUNCTIONS
// =============================================
bool BMI160::getMotion6(int16_t* ax, int16_t* ay, int16_t* az, 
                       int16_t* gx, int16_t* gy, int16_t* gz) {
    uint8_t data[12];
    unsigned long startTime = micros();
    
    // Read both accelerometer and gyroscope in one transaction
    if (!readRegisters(BMI160_GYRO_DATA, data, 12)) {
        return false;
    }
    
    _lastReadTime = micros() - startTime;
    
    // Extract gyroscope data (first 6 bytes) - little endian
    *gx = (int16_t)((data[1] << 8) | data[0]);
    *gy = (int16_t)((data[3] << 8) | data[2]);
    *gz = (int16_t)((data[5] << 8) | data[4]);
    
    // Extract accelerometer data (next 6 bytes) - little endian
    *ax = (int16_t)((data[7] << 8) | data[6]);
    *ay = (int16_t)((data[9] << 8) | data[8]);
    *az = (int16_t)((data[11] << 8) | data[10]);
    
    return true;
}

bool BMI160::getAccelData(int16_t* ax, int16_t* ay, int16_t* az) {
    uint8_t data[6];
    
    if (!readRegisters(BMI160_ACCEL_DATA, data, 6)) {
        return false;
    }
    
    *ax = (int16_t)((data[1] << 8) | data[0]);
    *ay = (int16_t)((data[3] << 8) | data[2]);
    *az = (int16_t)((data[5] << 8) | data[4]);
    
    return true;
}

bool BMI160::getGyroData(int16_t* gx, int16_t* gy, int16_t* gz) {
    uint8_t data[6];
    
    if (!readRegisters(BMI160_GYRO_DATA, data, 6)) {
        return false;
    }
    
    *gx = (int16_t)((data[1] << 8) | data[0]);
    *gy = (int16_t)((data[3] << 8) | data[2]);
    *gz = (int16_t)((data[5] << 8) | data[4]);
    
    return true;
}

// =============================================
// CALIBRATION FUNCTIONS
// =============================================
void BMI160::calibrateGyro(uint16_t samples, float* biasX, float* biasY, float* biasZ) {
    int32_t sumX = 0, sumY = 0, sumZ = 0;
    int16_t gx, gy, gz;
    uint16_t valid_sample_count = 0;
    for (uint16_t i = 0; i < samples; i++) {
        if (getGyroData(&gx, &gy, &gz)) {
            sumX += gx;
            sumY += gy;
            sumZ += gz;
            valid_sample_count++;
        }
        delay(1);
    }
    // Serial.print("sample count:"); Serial.println(samples);
    // Serial.print("valid sample count:"); Serial.println(valid_sample_count);
    float biasX_calc = (float)sumX / valid_sample_count;
    float biasY_calc = (float)sumY / valid_sample_count;
    float biasZ_calc = (float)sumZ / valid_sample_count;
    
    // Store biases if pointers provided
    if (biasX) *biasX = biasX_calc;
    if (biasY) *biasY = biasY_calc;
    if (biasZ) *biasZ = biasZ_calc;
}

void BMI160::calibrateAccel(uint16_t samples, float* biasX, float* biasY, float* biasZ) {
    int32_t sumX = 0, sumY = 0, sumZ = 0;
    int16_t ax, ay, az;
    uint16_t valid_sample_count = 0;
    for (uint16_t i = 0; i < samples; i++) {
        if (getAccelData(&ax, &ay, &az)) {
            sumX += ax;
            sumY += ay;
            sumZ += az;
            valid_sample_count++;
        }
        delay(1);
    }
    // Serial.print("sample count:"); Serial.println(samples);
    // Serial.print("valid sample count:"); Serial.println(valid_sample_count);
    // Use the dynamic LSB/g value
    float lsbPerG = getAccelLsbPerG(); // e.g., 8192 for ±4g
    
    float biasX_calc = (float)sumX / valid_sample_count;
    float biasY_calc = (float)sumY / valid_sample_count;
    // Dynamically subtract 1g (lsbPerG) from the Z-axis
    float biasZ_calc = ((float)sumZ / valid_sample_count) - lsbPerG;
    
    if (biasX) *biasX = biasX_calc;
    if (biasY) *biasY = biasY_calc;
    if (biasZ) *biasZ = biasZ_calc;
    
    // Optional: Print debug info
    Serial.print("Accel Cal: Range="); Serial.print(_accelRange);
    Serial.print("g, LSB/g="); Serial.println(lsbPerG);
}


// =============================================
// TEMPERATURE SENSOR FUNCTIONS
// =============================================


bool BMI160::getTemperature(float* temperature) {
    uint8_t data[2];
    
    // Read temperature registers (0x20-0x21)
    if (!readRegisters(BMI160_TEMPERATURE_0, data, 2)) {
        return false;
    }
    
    // Combine 16-bit value (little endian)
    int16_t tempRaw = (int16_t)((data[1] << 8) | data[0]);
    
    // Check for invalid temperature (0x8000)
    if (tempRaw == (int16_t)0x8000) {
        return false;
    }
    
    // Convert to Celsius
    // According to datasheet: 0x0000 = 23°C, 1 LSB = 0.5K when gyro active
    // For 16-bit signed: T = 23 + (temp_raw * 0.5) / 32768
    *temperature = 23.0f + (tempRaw * 0.5f) / 32768.0f;
    
    return true;
}

bool BMI160::isGyroInNormalMode() {
    uint8_t pmuStatus;
    if (!readRegisters(BMI160_PMU_STATUS, &pmuStatus, 1)) {
        return false;
    }
    // Bits 3:2 = gyro power mode (00=suspend, 01=normal, 11=fast start-up)
    uint8_t gyroMode = (pmuStatus >> 2) & 0x03;
    return (gyroMode == 0x01); // 0x01 = normal mode
}



bool BMI160::isTemperatureAvailable() {
    // Check if gyro is in normal mode (temperature sensor updates at 100Hz in this mode)
    uint8_t pmuStatus;
    if(!readRegisters(BMI160_PMU_STATUS, &pmuStatus, 1)) {
        return false;
    }
    uint8_t gyroMode = (pmuStatus >> 2) & 0x03;
    return (gyroMode == 0x01); // 0x01 = normal mode
    
}



// Implement the getter methods
float BMI160::getAccelScaleG() const {
    return _accelRange / 32768.0f;
}

float BMI160::getGyroScaleDps() const {
    return _gyroRange / 32768.0f;
}
// =============================================
// UTILITY FUNCTIONS
// =============================================
void BMI160::softReset() {
    writeRegister(BMI160_CMD, SOFT_RESET_CMD);
    delay(100);
}

uint8_t BMI160::getChipID() {
    uint8_t chipId;
    readRegisters(BMI160_CHIP_ID, &chipId, 1);
    return chipId;
}

// =============================================
// I2C COMMUNICATION
// =============================================
bool BMI160::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

bool BMI160::readRegisters(uint8_t reg, uint8_t* data, uint8_t length) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    
    Wire.requestFrom(_address, length);
    for (uint8_t i = 0; i < length && Wire.available(); i++) {
        data[i] = Wire.read();
    }
    return true;
}