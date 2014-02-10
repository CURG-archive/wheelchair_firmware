#ifndef WHEELCHAIR_H_
#define WHEELCHAIR_H_

#include <HardwareSerial.h>

class WheelChair {
    public:
        WheelChair(const uint8_t spdpin, const uint8_t dirpin, HardwareSerial *hws=NULL);
        virtual ~WheelChair();

        void init();

        uint16_t SetSpeed(float speed);
        uint16_t SetDirection(float direction);

        void SetSpeedAndDirection(float speed, float direction);

        uint16_t GetRawSpeed();
        uint16_t GetRawDirection();

    private:
        uint8_t m_spd_pin, m_dir_pin;

        uint16_t m_spd, m_dir;
        HardwareSerial * m_serial;

        uint16_t SetPin(uint8_t pin_val, float value, float offset, float max_range);
};

#endif
