#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Twist.h>
#include <PWM.h>
#include "wheelchair.h"

WheelChair wc(11, 12);

const uint8_t led_pin = 13;

ros::NodeHandle nh;

void stopCb(const std_msgs::Empty& stop_msg) {
    wc.SetSpeedAndDirection(0, 0);
}

void cmdVelCb(const geometry_msgs::Twist& cmd_vel) {
    float fwd = cmd_vel.linear.x;
    float turn = -cmd_vel.angular.z;
    wc.SetSpeedAndDirection(fwd, turn);
}

ros::Subscriber<std_msgs::Empty> stop_sub("stop", &stopCb);
ros::Subscriber<geometry_msgs::Twist> twist_sub("cmd_vel", &cmdVelCb);

std_msgs::UInt16 speed, dir;
ros::Publisher speed_pub("speed_int16", &speed);
ros::Publisher dir_pub("dir_int16", &dir);

uint32_t last_time = 0;

void setup() {
    wc.init();
    pinMode(led_pin, OUTPUT);

    nh.initNode();

    nh.advertise(speed_pub);
    nh.advertise(dir_pub);

    nh.subscribe(stop_sub);
    nh.subscribe(twist_sub);

    last_time = millis();
}

void loop() {
    nh.spinOnce();

    if (millis() - last_time > 50) {

        speed.data = wc.GetRawSpeed();
        dir.data = wc.GetRawDirection();

        speed_pub.publish(&speed);
        dir_pub.publish(&dir);

        last_time = millis();

        digitalWrite(led_pin, HIGH - digitalRead(led_pin));
    }
}
