// #include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// Include ROS
#include "ros/console.h"
#include "ros/ros.h"
#include "sensor_cam/cam_msg.h"
#include "sensor_cam_hough/cam_msg.h"
// #include "sensor_lidar/lidar_msg.h"
// #include "sensor_sonic/sonic_msg.h"
// #include "sensor_imu/imu_msg.h"
#include "xycar_msgs/xycar_motor.h"

// Include States
#include "team3-controller/ControlState.h"

// Const
const std::string NODE_NAME = "controller";
const std::string SUB_TOPIC_CAM = "cam_data";
// const std::string SUB_TOPIC_LIDAR = "lidar_data";
// const std::string SUB_TOPIC_SONIC = "SONIC_data";
// const std::string SUB_TOPIC_IMU = "imu_data";
const std::string PUB_TOPIC = "xycar_motor";
constexpr int FREQ = 140;  // Hz
constexpr float ANGLE_DIV = 2.f;

// std::chrono::system_clock::time_point t1, t2;

class Controller {
  ros::NodeHandle node;
  ros::Subscriber sub_cam;
  // ros::Subscriber sub_lidar;
  // ros::Subscriber sub_sonic;
  // ros::Subscriber sub_imu;
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
    // this->sub_lidar = node.subscribe(SUB_TOPIC_LIDAR, 1,
    // &Controller::callbackLidar, this);
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
  void callbackCamHough(const sensor_cam_hough::cam_msg::ConstPtr& msg);
  // void callbackLidar(const sensor_lidar::lidar_msg::ConstPtr& msg);
  void control();
  void start();
};

void Controller::callbackCam(const sensor_cam::cam_msg::ConstPtr& msg) {
  this->sensorState.cam.reduce(msg);
  this->control();
  // t2 = std::chrono::system_clock::now();
  // std::chrono::nanoseconds dt = t2 - t1;
  // std::cout << "time: " << dt.count() << '\n';
  // std::swap(t1, t2);
}

void Controller::callbackCamHough(
    const sensor_cam_hough::cam_msg::ConstPtr& msg) {
  this->sensorState.hough.reduce(msg);
  this->control();
  // t2 = std::chrono::system_clock::now();
  // std::chrono::nanoseconds dt = t2 - t1;
  // std::cout << "time: " << dt.count() << '\n';
  // std::swap(t1, t2);
}

// void Controller::callbackLidar(const sensor_lidar::lidar_msg::ConstPtr& msg)
// {
//   ROS_INFO("CON:: %d", msg->size)
// }

// TODO:
void Controller::control() {
#if 0
  // Decide angle
  float viewCenter = this->sensorState.cam.width / 2.f;
  float laneCenter =
      (this->sensorState.cam.lposSMA + this->sensorState.cam.rposSMA) / 2.f;
  // ROS_INFO("laneCenter: %.3f", laneCenter);
  int angle =
      (int)((laneCenter - viewCenter) / ANGLE_DIV + .5f);  // Round half up
  angle = this->correctAngle(angle);
  DRIVE_MODE mode =
      (std::abs(angle) < 5) ? DRIVE_MODE::GO_SLOW : DRIVE_MODE::TURN_SLOW;
  int speed =
      mode == DRIVE_MODE::GO_SLOW || mode == DRIVE_MODE::TURN_SLOW ? 5 : 30;

#else
  // Decide angle
  float viewCenter = this->sensorState.hough.width / 2.f;
  float laneCenter =
      (this->sensorState.hough.lposSMA + this->sensorState.hough.rposSMA) / 2.f;
  // ROS_INFO("laneCenter: %.3f", laneCenter);
  int angle =
      (int)((laneCenter - viewCenter) / ANGLE_DIV + .5f);  // Round half up
  angle = this->correctAngle(angle);
  DRIVE_MODE mode =
      (std::abs(angle) < 5) ? DRIVE_MODE::GO_SLOW : DRIVE_MODE::TURN_SLOW;
  int speed =
      mode == DRIVE_MODE::GO_SLOW || mode == DRIVE_MODE::TURN_SLOW ? 5 : 30;

#endif

  if (this->controlState.isStarted) {
    // ROS_INFO("Angle: %d | Speed: %d", angle, speed);
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

  // Set repeat freq
  // ros::Rate rate(FREQ);

  controller.start();
  // t1 = std::chrono::system_clock::now();
  while (ros::ok()) {
    ros::spinOnce();
    // controller.control();
    // rate.sleep();
  }

  return 0;
}