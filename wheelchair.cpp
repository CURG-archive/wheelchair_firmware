#include "wheelchair.h"
#include <PWM.h>

const uint32_t PWM_FREQUENCY = 32768;
const uint32_t PWM_MAX = 65535;

WheelChair::WheelChair(const uint8_t spdpin, const uint8_t dirpin,
        HardwareSerial *hws) {
    if (hws) {
        m_serial = hws;
    } else {
        m_serial = NULL;
    }

    m_spd_pin = spdpin;
    m_dir_pin = dirpin;
}

WheelChair::~WheelChair() {
}

void WheelChair::init() {
    InitTimersSafe();
    SetPinFrequency(m_spd_pin, PWM_FREQUENCY);
    SetPinFrequency(m_dir_pin, PWM_FREQUENCY);
    SetSpeedAndDirection(0, 0);
}

uint16_t WheelChair::SetSpeed(float speed) {
    const float offset = 0.4;
    const float range = 0.8;
    m_spd = SetPin(m_spd_pin, speed, offset, range);
    return m_spd;
}

uint16_t WheelChair::SetDirection(float direction) {
    const float offset = 0.4;
    const float range = 0.8;
    m_dir = SetPin(m_dir_pin, direction, offset, range);
    return m_dir;
}

void WheelChair::SetSpeedAndDirection(float speed, float direction) {
    SetSpeed(speed);
    SetDirection(direction);
}

uint16_t WheelChair::GetRawSpeed() {
    return m_spd;
}

uint16_t WheelChair::GetRawDirection() {
    return m_dir;
}

uint16_t WheelChair::SetPin(uint8_t pin_val, float value, float offset,
        float max_range) {
    if (value > 1.0) {
        value = 1.0;
    } else if (value < -1.0) {
        value = -1.0;
    }

    value = value * max_range * 0.5 + offset;

    if (value <= 0) {
        value = 0;
    } else if (value >= 1.0) {
        value = 1.0;
    }

    uint16_t pwm_val = PWM_MAX * value;

    // if (m_serial) {
    //     m_serial->print("SetPin: ");
    //     m_serial->print(pin_val);
    //     m_serial->print(", ");
    //     m_serial->print(pwm_val);
    //     m_serial->print("\r\n");
    // }
    pwmWriteHR(pin_val, pwm_val);
    return pwm_val;
}
