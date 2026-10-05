#include "QMC5883.h"
#include <math.h>
QMC5883::QMC5883(uint8_t address) : _address(address), _sdaPin(255), _sclPin(255) {}

bool QMC5883::begin(uint8_t sdaPin, uint8_t sclPin, uint16_t i2c_speed) {
    _sdaPin = sdaPin;
    _sclPin = sclPin;
    
    // Initialize I2C with specified pins
    // Wire.setSDA(_sdaPin);
    // Wire.setSCL(_sclPin);
    // Wire.setClock(i2c_speed);
    // Wire.begin();
    // Verify chip ID
    if (!checkID()) {
        return false;
    }
    
    // Default configuration
    Config config = {
        .mode = MODE_CONTINUOUS,
        .odr = ODR_200Hz,
        .range = RNG_8G,
        .osr = OSR_512,
        .setReset = SET_RESET_ON
    };
    
    return configure(config);
}

bool QMC5883::begin(uint8_t sdaPin, uint8_t sclPin, const Config& config) {
    _sdaPin = sdaPin;
    _sclPin = sclPin;
    
    // Initialize I2C with specified pins
    Wire.setSDA(_sdaPin);
    Wire.setSCL(_sclPin);
    Wire.begin();
    
    if (!checkID()) {
        return false;
    }
    
    _lastMode = config.mode; // Remember the mode for wakeup
    return configure(config);
}

bool QMC5883::configure(const Config& config) {
    // Control Register 1 (0x0A)
    // [OSR2][OSR1][ODR][MODE]
    uint8_t cr1 = config.osr | config.odr | config.mode;
    if (!writeRegister(REG_CONTROL_1, cr1)) {
        return false;
    }
    
    // Control Register 2 (0x0B)
    // [SOFT_RST][SELF_TEST][-][-][RNG][SET/RESET]
    uint8_t cr2 = (config.range << 2) | config.setReset;
    if (!writeRegister(REG_CONTROL_2, cr2)) {
        return false;
    }
    
    // Set sign configuration (default positive for all axes)
    if (!writeRegister(REG_SIGN, 0x06)) {
        return false;
    }
    
    _initialized = true;
    _asleep = (config.mode == MODE_STANDBY);
    return true;
}

bool QMC5883::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

uint8_t QMC5883::readRegister(uint8_t reg) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.endTransmission();
    
    Wire.requestFrom(_address, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0;
}

bool QMC5883::isDataReady() {
    if (_asleep) return false; // No data when asleep
    
    uint8_t status = readRegister(REG_STATUS);
    return (status & 0x01) != 0;
}

QMC5883::Status QMC5883::getStatus() {
    if (_asleep) {
        return {false, false}; // No data ready or overflow when asleep
    }
    
    uint8_t status = readRegister(REG_STATUS);
    return {
        .dataReady = (status & 0x01) != 0,
        .overflow = (status & 0x02) != 0
    };
}

bool QMC5883::softReset() {
    bool result = writeRegister(REG_CONTROL_2, 0x80);
    if (result) {
        delay(10); // Give time to reset
        _asleep = true; // Sensor goes to standby after reset
        _lastMode = MODE_CONTINUOUS; // Reset to default
    }
    return result;
}

bool QMC5883::selfTest() {
    if (_asleep) {
        if (!wakeup()) return false;
    }
    
    // Save current configuration
    uint8_t cr1 = readRegister(REG_CONTROL_1);
    uint8_t cr2 = readRegister(REG_CONTROL_2);
    
    // Set to continuous mode for self-test
    writeRegister(REG_CONTROL_1, (cr1 & 0xFC) | MODE_CONTINUOUS);
    
    // Read initial values
    RawData before;
    if (!readRaw(before)) {
        return false;
    }
    
    // Enable self-test
    writeRegister(REG_CONTROL_2, 0x40);
    delay(5); // Wait for measurement
    
    // Read values with self-test field
    RawData after;
    if (!readRaw(after)) {
        return false;
    }
    
    // Calculate differences
    int16_t deltaX = abs(after.x - before.x);
    int16_t deltaY = abs(after.y - before.y);
    int16_t deltaZ = abs(after.z - before.z);
    
    // Restore configuration
    writeRegister(REG_CONTROL_1, cr1);
    writeRegister(REG_CONTROL_2, cr2);
    
    // Simple threshold check
    const int16_t threshold = 100;
    return (deltaX > threshold) && (deltaY > threshold) && (deltaZ > threshold);
}

bool QMC5883::setMode(Mode mode) {
    uint8_t cr1 = readRegister(REG_CONTROL_1);
    cr1 = (cr1 & 0xFC) | mode;
    
    bool result = writeRegister(REG_CONTROL_1, cr1);
    if (result) {
        _lastMode = mode;
        _asleep = (mode == MODE_STANDBY);
    }
    return result;
}

bool QMC5883::setOutputDataRate(OutputDataRate odr) {
    uint8_t cr1 = readRegister(REG_CONTROL_1);
    cr1 = (cr1 & 0xF3) | odr;
    return writeRegister(REG_CONTROL_1, cr1);
}

bool QMC5883::setRange(Range range) {
    uint8_t cr2 = readRegister(REG_CONTROL_2);
    cr2 = (cr2 & 0xF3) | (range << 2);
    return writeRegister(REG_CONTROL_2, cr2);
}

bool QMC5883::checkID() {
    uint8_t id = readRegister(REG_CHIP_ID);
    return (id == 0x80);
}

uint8_t QMC5883::getChipID() {
    return readRegister(REG_CHIP_ID);
}

bool QMC5883::isInitialized() const {
    return _initialized;
}

float QMC5883::getSensitivity(Range range) {
    switch (range) {
        case RNG_2G:  return 15000.0;
        case RNG_8G:  return 3750.0;
        case RNG_12G: return 2500.0;
        case RNG_30G: return 1000.0;
        default:      return 3750.0;
    }
}

float QMC5883::calculateHeading(float x, float y) {
    return atan2(y, x);
}

float QMC5883::calculateHeadingDegrees(float x, float y) {
    float heading = atan2(y, x) * 180.0 / PI;
    if (heading < 0) heading += 360.0;
    return heading;
}

// Power management functions
bool QMC5883::sleep() {
    bool result = setMode(MODE_STANDBY);
    if (result) {
        _asleep = true;
    }
    return result;
}

bool QMC5883::wakeup() {
    bool result = setMode(_lastMode);
    if (result) {
        _asleep = false;
        delay(5); // Small delay for sensor to wake up
    }
    return result;
}

bool QMC5883::isAsleep() const {
    return _asleep;
}

// Calibration methods
void QMC5883::startCalibration() {
    _calibration = {32767, -32768, 32767, -32768, 32767, -32768, false};
    // Reset soft iron matrix to identity
    for (uint8_t i = 0; i < 3; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            _calibration.soft_iron[i][j] = (i == j) ? 1.0 : 0.0;
        }
        
    }
    
}

void QMC5883::updateCalibration() {
    RawData raw;
    if (readRaw(raw)) {
        _calibration.x_min = min(_calibration.x_min, raw.x);
        _calibration.x_max = max(_calibration.x_max, raw.x);
        _calibration.y_min = min(_calibration.y_min, raw.y);
        _calibration.y_max = max(_calibration.y_max, raw.y);
        _calibration.z_min = min(_calibration.z_min, raw.z);
        _calibration.z_max = max(_calibration.z_max, raw.z);
        _calibration.calibrated = false;
    }
}

void QMC5883::endCalibration() {
    // Calculate hard iron offsets
    _hardIronOffset.x = (_calibration.x_min + _calibration.x_max) / 2.0;
    _hardIronOffset.y = (_calibration.y_min + _calibration.y_max) / 2.0;
    _hardIronOffset.z = (_calibration.z_min + _calibration.z_max) / 2.0;
    
    // Calculate ranges (diameters)
    float range_x = (_calibration.x_max - _calibration.x_min) / 2.0;
    float range_y = (_calibration.y_max - _calibration.y_min) / 2.0;
    float range_z = (_calibration.z_max - _calibration.z_min) / 2.0;
    
    // Calculate average radius (for soft iron scaling)
    float avg_radius = (range_x + range_y + range_z) / 3.0;
    
    // Create soft iron calibration matrix (simplified - assumes axes are orthogonal)
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            _calibration.soft_iron[i][j] = 0.0;
        }
    }
    
    // Scale factors to normalize all axes to same radius
    _calibration.soft_iron[0][0] = avg_radius / range_x;
    _calibration.soft_iron[1][1] = avg_radius / range_y;
    _calibration.soft_iron[2][2] = avg_radius / range_z;
    
    _calibration.calibrated = true;
}


QMC5883::RawData QMC5883::applyHardIronCalibration(const RawData& raw) {
    if (!_calibration.calibrated) return raw;
    
    // 1. Apply hard iron calibration (offsets)
    float x = raw.x - _hardIronOffset.x;
    float y = raw.y - _hardIronOffset.y;
    float z = raw.z - _hardIronOffset.z;
    
    // 2. Apply soft iron calibration (matrix multiplication)
    float calibrated_x = _calibration.soft_iron[0][0] * x + 
                         _calibration.soft_iron[0][1] * y + 
                         _calibration.soft_iron[0][2] * z;
    float calibrated_y = _calibration.soft_iron[1][0] * x + 
                         _calibration.soft_iron[1][1] * y + 
                         _calibration.soft_iron[1][2] * z;
    float calibrated_z = _calibration.soft_iron[2][0] * x + 
                         _calibration.soft_iron[2][1] * y + 
                         _calibration.soft_iron[2][2] * z;
    
    RawData calibrated;
    calibrated.x = (int16_t)calibrated_x;
    calibrated.y = (int16_t)calibrated_y;
    calibrated.z = (int16_t)calibrated_z;
    
    return calibrated;
}



void QMC5883::updateHardIronOffset(float offX, float offY, float offZ) {
    if (!_calibration.calibrated) return;
    float halfRangeX = (_calibration.x_max - _calibration.x_min) / 2.0f;
    float halfRangeY = (_calibration.y_max - _calibration.y_min) / 2.0f;
    float halfRangeZ = (_calibration.z_max - _calibration.z_min) / 2.0f;
    _calibration.x_min = offX - halfRangeX;
    _calibration.x_max = offX + halfRangeX;
    _calibration.y_min = offY - halfRangeY;
    _calibration.y_max = offY + halfRangeY;
    _calibration.z_min = offZ - halfRangeZ;
    _calibration.z_max = offZ + halfRangeZ;
    _hardIronOffset.x = offX;
    _hardIronOffset.y = offY;
    _hardIronOffset.z = offZ;
}

bool QMC5883::isCalibrated() const {
    return _calibration.calibrated;
}

const QMC5883::CalibrationData& QMC5883::getCalibrationData() const {
    return _calibration;
}

void QMC5883::setCalibrationData(const CalibrationData& cal) {
    _calibration = cal;
}



void QMC5883::performEllipsoidCalibration(int samples) {
    // Simple ellipsoid fitting algorithm
    // This is a simplified version - for production, use a proper ellipsoid fit
    
    // Collect samples
    RawData* samplesArray = new RawData[samples];
    int validSamples = 0;
    
    for(int i = 0; i < samples; i++) {
        RawData raw;
        if(readRaw(raw)) {
            samplesArray[validSamples++] = raw;
        }
        delay(10);
    }
    
    if(validSamples < 50) {
        delete[] samplesArray;
        return; // Not enough samples
    }
    
    // Calculate means (hard iron offsets)
    float mean_x = 0, mean_y = 0, mean_z = 0;
    for(int i = 0; i < validSamples; i++) {
        mean_x += samplesArray[i].x;
        mean_y += samplesArray[i].y;
        mean_z += samplesArray[i].z;
    }
    mean_x /= validSamples;
    mean_y /= validSamples;
    mean_z /= validSamples;
    
    // Calculate radii (simplified)
    float max_radius = 0;
    for(int i = 0; i < validSamples; i++) {
        float dx = samplesArray[i].x - mean_x;
        float dy = samplesArray[i].y - mean_y;
        float dz = samplesArray[i].z - mean_z;
        float radius = sqrt(dx*dx + dy*dy + dz*dz);
        if(radius > max_radius) max_radius = radius;
    }
    
    // Scale to unit sphere
    float scale = 1.0 / max_radius;
    
    // Update calibration
    _calibration.x_min = (int16_t)(mean_x - max_radius);
    _calibration.x_max = (int16_t)(mean_x + max_radius);
    _calibration.y_min = (int16_t)(mean_y - max_radius);
    _calibration.y_max = (int16_t)(mean_y + max_radius);
    _calibration.z_min = (int16_t)(mean_z - max_radius);
    _calibration.z_max = (int16_t)(mean_z + max_radius);
    
    // Set soft iron matrix (scaling only)
    _calibration.soft_iron[0][0] = scale;
    _calibration.soft_iron[1][1] = scale;
    _calibration.soft_iron[2][2] = scale;
    
    delete[] samplesArray;
    _calibration.calibrated = true;
}


bool QMC5883::readRaw(RawData& data) {
    if (_asleep) return false;
    if (!isDataReady()) return false;
    
    Wire.beginTransmission(_address);
    Wire.write(REG_DATA_X_LSB);
    if (Wire.endTransmission() != 0) return false;
    
    Wire.requestFrom(_address, (uint8_t)6);
    if (Wire.available() != 6) return false;
    
    uint8_t x_lsb = Wire.read();
    uint8_t x_msb = Wire.read();
    uint8_t y_lsb = Wire.read();
    uint8_t y_msb = Wire.read();
    uint8_t z_lsb = Wire.read();
    uint8_t z_msb = Wire.read();
    
    data.x = (int16_t)((x_msb << 8) | x_lsb);
    data.y = (int16_t)((y_msb << 8) | y_lsb);
    data.z = (int16_t)((z_msb << 8) | z_lsb);
    
    data = applyOrientationCorrection(data);
    return true;
}

bool QMC5883::read(Data& data, Range range) {
    RawData raw;
    if (!readRaw(raw)) return false;
    
    float sensitivity = getSensitivity(range);
    Data uncorrected;
    uncorrected.x = raw.x / sensitivity;
    uncorrected.y = raw.y / sensitivity;
    uncorrected.z = raw.z / sensitivity;
    
    data = applyOrientationCorrection(uncorrected);
    return true;
}

bool QMC5883::readCalibrated(Data& data, Range range) {
    RawData raw;
    if (!readRaw(raw)) return false;
    
    RawData calibrated_raw = applyHardIronCalibration(raw);
    
    float sensitivity = getSensitivity(range);
    Data temp;
    temp.x = calibrated_raw.x / sensitivity;
    temp.y = calibrated_raw.y / sensitivity;
    temp.z = calibrated_raw.z / sensitivity;
    
    data = applyOrientationCorrection(temp);
    return true;
}



void QMC5883::setOrientation(BoardOrientation orientation) {
    switch (orientation) {
        case ORIENTATION_NORMAL:
            _orientation = {0, 1, 2, 1, 1, 1};
            break;
        case ORIENTATION_ROTATE_90:
            _orientation = {1, 0, 2, 1, -1, 1};
            break;
        case ORIENTATION_ROTATE_180:
            _orientation = {0, 1, 2, -1, -1, 1};
            break;
        case ORIENTATION_ROTATE_270:
            _orientation = {1, 0, 2, -1, 1, 1};
            break;
        case ORIENTATION_FLIP_X:
            _orientation = {0, 1, 2, -1, 1, 1};
            break;
        case ORIENTATION_FLIP_Y:
            _orientation = {0, 1, 2, 1, -1, 1};
            break;
        case ORIENTATION_CUSTOM:
            break;
    }
}

void QMC5883::setCustomOrientation(uint8_t x_src, uint8_t y_src, uint8_t z_src, 
                                 bool x_invert, bool y_invert, bool z_invert) {
    _orientation.x_map = constrain(x_src, 0, 2);
    _orientation.y_map = constrain(y_src, 0, 2);
    _orientation.z_map = constrain(z_src, 0, 2);
    _orientation.x_sign = x_invert ? -1 : 1;
    _orientation.y_sign = y_invert ? -1 : 1;
    _orientation.z_sign = z_invert ? -1 : 1;
}

QMC5883::RawData QMC5883::applyOrientationCorrection(const RawData& raw) {
    int16_t axes[3] = {raw.x, raw.y, raw.z};
    
    RawData corrected;
    corrected.x = axes[_orientation.x_map] * _orientation.x_sign;
    corrected.y = axes[_orientation.y_map] * _orientation.y_sign; 
    corrected.z = axes[_orientation.z_map] * _orientation.z_sign;
    
    return corrected;
}

QMC5883::Data QMC5883::applyOrientationCorrection(const Data& data) {
    float axes[3] = {data.x, data.y, data.z};
    
    Data corrected;
    corrected.x = axes[_orientation.x_map] * _orientation.x_sign;
    corrected.y = axes[_orientation.y_map] * _orientation.y_sign;
    corrected.z = axes[_orientation.z_map] * _orientation.z_sign;
    
    return corrected;
}

void QMC5883::setDeclination(float declination_degrees) {
    _magneticDeclination = declination_degrees;
}

float QMC5883::getDeclination() const {
    return _magneticDeclination;
}

float QMC5883::applyDeclination(float magnetic_heading) {
    float true_heading = magnetic_heading + _magneticDeclination;
    
    while (true_heading < 0) true_heading += 360;
    while (true_heading >= 360) true_heading -= 360;
    
    return true_heading;
}

bool QMC5883::getTrueHeading(float &true_heading) {
    Data data;
    if (!readCalibrated(data)) return false;
    
    float magnetic_heading = calculateHeadingDegrees(data.x, data.y);
    true_heading = applyDeclination(magnetic_heading);
    return true;
}

bool QMC5883::getHeadings(float &magnetic_heading, float &true_heading) {
    Data data;
    if (!readCalibrated(data)) return false;
    
    magnetic_heading = calculateHeadingDegrees(data.x, data.y);
    true_heading = applyDeclination(magnetic_heading);
    return true;
}

bool QMC5883::getAllData(Data& sensor_data, float& magnetic_heading, float& true_heading) {
    if (!readCalibrated(sensor_data)) return false;
    
    magnetic_heading = calculateHeadingDegrees(sensor_data.x, sensor_data.y);
    true_heading = applyDeclination(magnetic_heading);
    return true;
}