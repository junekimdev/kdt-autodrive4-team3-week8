#ifndef CONTROLDRIVER_H
#define CONTROLDRIVER_H

#include "ros/ros.h"
#include "xycar_msgs/xycar_motor.h"

// Const
const std::string CONTROLLER_NAME = "controller";

constexpr int ANGLE_CENTER = 8;
constexpr int MAX_SPEED = 30;

class ControlDriver {
protected:
  int angle;
  int speed;

public:
  ControlDriver() : angle(ANGLE_CENTER), speed(0) {}
  ControlDriver(int angle, int speed) : angle(angle), speed(speed) {}
  virtual void drive(ros::Publisher& pub) const = 0;
};

class DriverStop : public ControlDriver {
public:
  DriverStop() : ControlDriver(ANGLE_CENTER, 0) {}
  void drive(ros::Publisher& pub) const {
    xycar_msgs::xycar_motor msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = CONTROLLER_NAME;
    msg.angle = this->angle;
    msg.speed = this->speed;
    pub.publish(msg);
  }
};

class DriverGoSlow : public ControlDriver {
public:
  DriverGoSlow() : ControlDriver(ANGLE_CENTER, 10) {}
  void drive(ros::Publisher& pub) const {
    xycar_msgs::xycar_motor msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = CONTROLLER_NAME;
    msg.angle = this->angle;
    msg.speed = this->speed;
    pub.publish(msg);
  }
};

class DriverGoFast : public ControlDriver {
public:
  DriverGoFast() : ControlDriver(ANGLE_CENTER, MAX_SPEED) {}
  void drive(ros::Publisher& pub) const {
    xycar_msgs::xycar_motor msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = CONTROLLER_NAME;
    msg.angle = this->angle;
    msg.speed = this->speed;
    pub.publish(msg);
  }
};

class DriverTurnSlow : public ControlDriver {
public:
  DriverTurnSlow(int angle) : ControlDriver(angle, 10) {}
  void drive(ros::Publisher& pub) const {
    xycar_msgs::xycar_motor msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = CONTROLLER_NAME;
    msg.angle = this->angle;
    msg.speed = this->speed;
    pub.publish(msg);
  }
};

#endif