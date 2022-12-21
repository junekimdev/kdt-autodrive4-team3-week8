#ifndef CONTROL_STATE_H
#define CONTROL_STATE_H

#include "ros/ros.h"
#include "xycar_msgs/xycar_motor.h"

// Const
const std::string CONTROLLER_NAME = "controller";

constexpr int ANGLE_CENTER = 8;
constexpr int MAX_ANGLE = 50;
constexpr int MIN_ANGLE = -50;
constexpr int MAX_SPEED = 30;

enum struct DRIVE_MODE { STOP, GO_SLOW, GO_FAST, TURN_SLOW };

struct ControlState {
  ros::Publisher& pub;
  DRIVE_MODE mode;
  int angle;
  int speed;
  bool isStarted;

  ControlState()
      : pub(nullptr),
        mode(DRIVE.STOP),
        angle(ANGLE_CENTER),
        speed(0),
        isStarted(false) {}
  ControlState(ros::Publisher& pub)
      : pub(pub),
        mode(DRIVE.STOP),
        angle(ANGLE_CENTER),
        speed(0),
        isStarted(false) {}

  xycar_msgs::xycar_motor createMsg() {
    xycar_msgs::xycar_motor msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = CONTROLLER_NAME;
    msg.angle = this->angle;
    msg.speed = this->speed;
    return msg;
  }

  void drive(ros::Publisher& pub) const { pub.publish(this->createMsg()); }

  void reduce(DRIVE_MODE mode, int angle, int speed) {
    this->mode = mode;
    this->angle = angle;
    this->speed = speed;
    this->drive();
  }
};

#endif