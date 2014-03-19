#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <PWM.h>
#include <NewPing.h>
#include "wheelchair.h"

WheelChair wc(11, 12);

static const uint8_t led_pin = 13;
static const uint8_t trig_right = 50;
static const uint8_t echo_right = 48;
static const uint8_t trig_left = 46;
static const uint8_t echo_left = 44;
static const uint8_t trig_center = 42;
static const uint8_t echo_center = 40;

static const uint8_t MAX_DISTANCE = 200;

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


sensor_msgs::Range left_sonar_msg;
sensor_msgs::Range center_sonar_msg;
sensor_msgs::Range right_sonar_msg;
char *left_frame = "sonar_left";
char *center_frame = "sonar_center";
char *right_frame = "sonar_right";

ros::Publisher left_sonar_pub(left_frame, &left_sonar_msg);
ros::Publisher center_sonar_pub(center_frame, &center_sonar_msg);
ros::Publisher right_sonar_pub(right_frame, &right_sonar_msg);

NewPing left_sonar(trig_left, echo_left, MAX_DISTANCE);
NewPing center_sonar(trig_center, echo_center, MAX_DISTANCE);
NewPing right_sonar(trig_right, echo_right, MAX_DISTANCE);

uint32_t last_time = 0;

uint32_t seq = 0;

void initializeSonarMsg(sensor_msgs::Range& msg) {
    msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    msg.field_of_view = 0.261; // approx 15 deg
    msg.min_range = 0.01; // 1 cm
    msg.max_range = 0.01 * MAX_DISTANCE; // 250 cm
}

void setup() {
    wc.init();
    pinMode(led_pin, OUTPUT);

    nh.initNode();

    nh.advertise(speed_pub);
    nh.advertise(dir_pub);
    nh.advertise(left_sonar_pub);
    nh.advertise(center_sonar_pub);
    nh.advertise(right_sonar_pub);

    nh.subscribe(stop_sub);
    nh.subscribe(twist_sub);

    last_time = millis();

    initializeSonarMsg(left_sonar_msg);
    initializeSonarMsg(center_sonar_msg);
    initializeSonarMsg(right_sonar_msg);

    left_sonar_msg.header.frame_id = left_frame;
    center_sonar_msg.header.frame_id = center_frame;
    right_sonar_msg.header.frame_id = right_frame;
}

void loop() {
    nh.spinOnce();

    if (millis() - last_time > 50) {
        ros::Time now = nh.now();

        // speed and direction
        speed.data = wc.GetRawSpeed();
        dir.data = wc.GetRawDirection();

        speed_pub.publish(&speed);
        dir_pub.publish(&dir);

        // sonar
        left_sonar_msg.header.seq = seq;
        center_sonar_msg.header.seq = seq;
        right_sonar_msg.header.seq = seq;

        left_sonar_msg.header.stamp = now;
        center_sonar_msg.header.stamp = now;
        right_sonar_msg.header.stamp = now;

        left_sonar_msg.range = left_sonar.ping_cm() * 0.01;
        center_sonar_msg.range = center_sonar.ping_cm() * 0.01;
        right_sonar_msg.range = right_sonar.ping_cm() * 0.01;

        left_sonar_pub.publish(&left_sonar_msg);
        center_sonar_pub.publish(&center_sonar_msg);
        right_sonar_pub.publish(&right_sonar_msg);

        // loop data
        last_time = millis();
        seq++;

        // heartbeat
        digitalWrite(led_pin, (seq % 2 == 1) ? HIGH : LOW);
    }
}
