#ifndef QMC5883_H
#define QMC5883_H

#include <Arduino.h>
#include <Wire.h>

class QMC5883 {
public:
    // Operating Modes
    enum Mode {
        MODE_STANDBY     = 0x00, // Suspend mode - lowest power
        MODE_NORMAL      = 0x01, // Continuous measurement at ODR
        MODE_SINGLE      = 0x02, // Single measurement then suspend
        MODE_CONTINUOUS  = 0x03  // Continuous at max speed
    };

    // Output Data Rates
    enum OutputDataRate {
        ODR_10Hz  = 0x00,
        ODR_50Hz  = 0x04,
        ODR_100Hz = 0x08,
        ODR_200Hz = 0x0C
    };

    // Magnetic Field Ranges
    enum Range {
        RNG_2G  = 0x00,  // ±2 Gauss
        RNG_8G  = 0x01,  // ±8 Gauss  
        RNG_12G = 0x02,  // ±12 Gauss
        RNG_30G = 0x03   // ±30 Gauss
    };

    // Over Sampling Ratios
    enum OverSampleRatio {
        OSR_512 = 0x00,  // Lowest noise, highest power
        OSR_256 = 0x40,
        OSR_128 = 0x80,
        OSR_64  = 0xC0   // Highest noise, lowest power
    };

    // Set/Reset Modes
    enum SetResetMode {
        SET_RESET_ON   = 0x00, // Default - best accuracy
        SET_ONLY_ON    = 0x01,
        SET_RESET_OFF  = 0x02
    };

    // Pre-defined orientations
    enum BoardOrientation {
        ORIENTATION_NORMAL,      // X=X, Y=Y, Z=Z
        ORIENTATION_ROTATE_90,   // 90° clockwise
        ORIENTATION_ROTATE_180,  // 180°
        ORIENTATION_ROTATE_270,  // 90° counter-clockwise
        ORIENTATION_FLIP_X,      // X inverted
        ORIENTATION_FLIP_Y,      // Y inverted
        ORIENTATION_CUSTOM       // Custom mapping
    };

    // Status flags
    struct Status {
        bool dataReady;
        bool overflow;
    };

    // Raw sensor data
    struct RawData {
        int16_t x;
        int16_t y;
        int16_t z;
    };

    // Calibrated data (in Gauss)
    struct Data {
        float x;
        float y;
        float z;
    };

    // Configuration structure
    struct Config {
        Mode mode;
        OutputDataRate odr;
        Range range;
        OverSampleRatio osr;
        SetResetMode setReset;
    };

    // Calibration data
    struct CalibrationData {
        int16_t x_min, x_max;
        int16_t y_min, y_max; 
        int16_t z_min, z_max;
        // Soft iron calibration (3x3 matrix)
        float soft_iron[3][3] = {
            {1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0}
        };
        
        bool calibrated = false;
    };

    // Constructor
    QMC5883(uint8_t address = 0x2C);
    

    // _hardIronOffset.x = 0;
    // _hardIronOffset.y = 0;
    // _hardIronOffset.z = 0;

    // Initialization methods - ALWAYS require I2C pins
    bool begin(uint8_t sdaPin, uint8_t sclPin, uint16_t i2c_speed = 1000000UL);
    bool begin(uint8_t sdaPin, uint8_t sclPin, const Config& config);
    
    // Power management
    bool sleep();           // Enter low-power standby mode
    bool wakeup();          // Wake up from standby to previous mode
    bool isAsleep() const;  // Check if sensor is in standby
    
    // Data reading methods
    bool readRaw(RawData& data);
    bool read(Data& data, Range range = RNG_8G);
    bool readCalibrated(Data& data, Range range = RNG_8G);
    
    // Heading methods
    bool getTrueHeading(float &true_heading);
    bool getHeadings(float &magnetic_heading, float &true_heading);
    bool getAllData(Data& sensor_data, float& magnetic_heading, float& true_heading);
    
    // Sensor status and control
    bool isDataReady();
    Status getStatus();
    bool softReset();
    bool selfTest();
    
    // Configuration methods
    bool setMode(Mode mode);
    bool setOutputDataRate(OutputDataRate odr);
    bool setRange(Range range);
    
    // Information methods
    bool checkID();
    uint8_t getChipID();
    bool isInitialized() const;
    
    // Static utility methods
    static float getSensitivity(Range range);
    static float calculateHeading(float x, float y);
    static float calculateHeadingDegrees(float x, float y);
    
    // Calibration methods
    void startCalibration();
    void updateCalibration();
    void endCalibration();
    bool isCalibrated() const;
    
    // CalibrationData getCalibrationData() const;
    void setCalibrationData(const CalibrationData& cal);
    
    // Orientation correction
    void setOrientation(BoardOrientation orientation);
    void setCustomOrientation(uint8_t x_src, uint8_t y_src, uint8_t z_src, 
                             bool x_invert = false, bool y_invert = false, bool z_invert = false);
    
    // Magnetic declination
    void setDeclination(float declination_degrees);
    float getDeclination() const;
    float applyDeclination(float magnetic_heading);

    // helper function
    void performEllipsoidCalibration(int samples);


    // Read-only access to hard-iron offset (raw counts)
    const Data& getHardIronOffset() const { return _hardIronOffset; }

    // Read-only access to the full calibration struct (returns a copy, safe)
    const CalibrationData& getCalibrationData() const;

    void updateHardIronOffset(float offsetX_counts, float offsetY_counts, float offsetZ_counts);
    

private:
    uint8_t _address;
    uint8_t _sdaPin;
    uint8_t _sclPin;
    bool _initialized = false;
    bool _asleep = false;
    float _magneticDeclination = 0.0;
    Mode _lastMode = MODE_CONTINUOUS; // Remember last mode for wakeup

    // Register addresses
    enum Registers {
        REG_CHIP_ID      = 0x00,
        REG_DATA_X_LSB   = 0x01,
        REG_DATA_X_MSB   = 0x02,
        REG_DATA_Y_LSB   = 0x03,
        REG_DATA_Y_MSB   = 0x04,
        REG_DATA_Z_LSB   = 0x05,
        REG_DATA_Z_MSB   = 0x06,
        REG_STATUS       = 0x09,
        REG_CONTROL_1    = 0x0A,
        REG_CONTROL_2    = 0x0B,
        REG_SIGN         = 0x29
    };

    // Orientation correction
    struct Orientation {
        int8_t x_map;    // Source axis for X (0=x, 1=y, 2=z)
        int8_t y_map;    // Source axis for Y  
        int8_t z_map;    // Source axis for Z
        int8_t x_sign;   // Sign for X (+1 or -1)
        int8_t y_sign;   // Sign for Y (+1 or -1)
        int8_t z_sign;   // Sign for Z (+1 or -1)
    };
    
    Orientation _orientation = {0, 1, 2, 1, 1, 1};
    CalibrationData _calibration = {32767, -32768, 32767, -32768, 32767, -32768, false};
    struct Data _hardIronOffset;
    
    // Private methods
    bool configure(const Config& config);
    bool writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    RawData applyOrientationCorrection(const RawData& raw);
    Data applyOrientationCorrection(const Data& data);
    RawData applyHardIronCalibration(const RawData& raw);

};

#endif