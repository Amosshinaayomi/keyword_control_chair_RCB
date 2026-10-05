// PID.h
#pragma once

class PID {
public:
    PID(float kp, float ki, float kd, float dt, float out_min, float out_max)
        : _kp(kp), _ki(ki), _kd(kd), _dt(dt), _out_min(out_min), _out_max(out_max) {}

    float compute(float setpoint, float measurement) {
        float error = setpoint - measurement;

        // Proportional
        float P = _kp * error;
        
        if (_ki != 0.0f) {
            _integral += error * _dt;
            _integral = constrain(_integral, _out_min / _ki, _out_max / _ki);
        } else {
            _integral = 0;
        }

        // Derivative (on measurement to avoid derivative kick)
        float D = _kd * (measurement - _prev_measurement) / _dt;
        _prev_measurement = measurement;

        float output = P + _ki * _integral - D;  // note: D on measurement, negative sign
        output = constrain(output, _out_min, _out_max);
        return output;
    }

    void reset() {
        _integral = 0;
        _prev_measurement = 0;
    }

private:
    float _kp, _ki, _kd, _dt;
    float _out_min, _out_max;
    float _integral = 0;
    float _prev_measurement = 0;
};