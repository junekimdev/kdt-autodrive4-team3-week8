#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// Include ROS
#include "ros/console.h"
#include "ros/ros.h"
#include "sensor_cam/cam_msg.h"
#include "xycar_msgs/xycar_motor.h"

// Include States
#include "team3-controller/ControlState.h"

// Const
const std::string NODE_NAME = "controller";
const std::string SUB_TOPIC_CAM = "cam_data";
const std::string PUB_TOPIC = "xycar_motor";
constexpr int FREQ = 140;  // Hz
constexpr int MAX_SPEED = 15;
constexpr float ANGLE_DIV = 2.f;
constexpr float HOUGH_ANGLE_PX_DIFF = 20;

class Controller {
  ros::NodeHandle node;
  ros::Subscriber sub_cam;
  ros::Publisher pub;
  SensorState sensorState;
  ControlState controlState;

  int correctAngle(int angle) {
    angle += ANGLE_CENTER;
    if (angle > MAX_ANGLE) angle = MAX_ANGLE;
    if (angle < MIN_ANGLE) angle = MIN_ANGLE;
    return angle;
  }

public:
  Controller() {
    this->sub_cam =
        this->node.subscribe(SUB_TOPIC_CAM, 1, &Controller::callbackCam, this);
    this->pub = this->node.advertise<xycar_msgs::xycar_motor>(PUB_TOPIC, 1);
  }

  xycar_msgs::xycar_motor createMsg() {
    xycar_msgs::xycar_motor msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = CONTROLLER_NAME;
    msg.angle = this->controlState.angle;
    msg.speed = this->controlState.speed;
    return msg;
  }

  void callbackCam(const sensor_cam::cam_msg::ConstPtr& msg);
  void control();
  void start();
};

void Controller::callbackCam(const sensor_cam::cam_msg::ConstPtr& msg) {
  this->sensorState.cam.reduce(msg);
}

void Controller::control() {
  // Cam
  float cView = this->sensorState.cam.width / 2.f;
  // float cposCam =
  //     (this->sensorState.cam.lpos + this->sensorState.cam.rpos) / 2.f;
  float cLane =
      (this->sensorState.cam.lposSMA + this->sensorState.cam.rposSMA) / 2.f;

  int angle, speed;
  angle = (int)((cLane - cView) / ANGLE_DIV + .5f);
  if (!this->sensorState.cam.isLeftDetected) angle -= 5;
  if (!this->sensorState.cam.isRightDetected) angle += 5;
  angle = this->correctAngle(angle);
  speed = 10;
  DRIVE_MODE mode = DRIVE_MODE::GO;

  if (this->controlState.isStarted) {
    this->controlState.reduce(mode, angle, speed);
    this->pub.publish(this->createMsg());
  }
}

void Controller::start() { this->controlState.isStarted = true; }

int main(int argc, char** argv) {
  // Init
  ros::init(argc, argv, NODE_NAME);
  Controller controller;
  ROS_INFO("%s is ONLINE", NODE_NAME.c_str());

  controller.start();
  while (ros::ok()) {
    ros::spinOnce();
    controller.control();
  }

  return 0;
}