#include <memory>
#include <string>
#include <vector>

// Include ROS
#include "ros/console.h"
#include "ros/ros.h"
#include "sensor_cam/cam_msg.h"
// #include "sensor_lidar/lidar_msg.h"
// #include "sensor_sonic/sonic_msg.h"
// #include "sensor_imu/imu_msg.h"
#include "xycar_msgs/xycar_motor.h"

// Include Drivers
#include "../include/team3-controller/ControlDriver.h"

// Const
const std::string NODE_NAME = "controller";
const std::string SUB_TOPIC_CAM = "cam_data";
// const std::string SUB_TOPIC_LIDAR = "lidar_data";
// const std::string SUB_TOPIC_SONIC = "SONIC_data";
// const std::string SUB_TOPIC_IMU = "imu_data";
const std::string PUB_TOPIC = "xycar_motor";
constexpr int FREQ = 2;  // 2Hz

class Controller {
  ros::NodeHandle node;
  ros::Subscriber sub_cam;
  // ros::Subscriber sub_lidar;
  // ros::Subscriber sub_sonic;
  // ros::Subscriber sub_imu;
  ros::Publisher pub;
  int width;
  int lpos;
  int rpos;

public:
  Controller() {
    sub_cam = node.subscribe(SUB_TOPIC_CAM, 1, &Controller::callbackCam, this);
    // sub_lidar = node.subscribe(SUB_TOPIC_LIDAR, 1,
    // &Controller::callbackLidar, this);
    pub = node.advertise<xycar_msgs::xycar_motor>(PUB_TOPIC, 1);
  }

  void callbackCam(const sensor_cam::cam_msg::ConstPtr& msg);
  // void callbackLidar(const sensor_lidar::lidar_msg::ConstPtr& msg);
  void control();
};

void Controller::callbackCam(const sensor_cam::cam_msg::ConstPtr& msg) {
  this->width = msg->width;
  this->lpos = msg->lpos;
  this->rpos = msg->rpos;
  this->control();
}

// void Controller::callbackLidar(const sensor_lidar::lidar_msg::ConstPtr& msg)
// {
//   ROS_INFO("CON:: %d", msg->size)
// }

void Controller::control() {
  // TODO:
  int viewCenter = this->width / 2;
  int laneCenter = (this->rpos + this->lpos) / 2;
  int ang = laneCenter - viewCenter;
  std::unique_ptr<ControlDriver> driver;
  if (ang < 5) {
    driver = std::make_unique<DriverGoSlow>();
  } else if (ang > 50) {
    driver = std::make_unique<DriverTurnSlow>(50);
  } else if (ang < -50) {
    driver = std::make_unique<DriverTurnSlow>(-50);
  } else {
    driver = std::make_unique<DriverTurnSlow>(ang);
  }
  driver->drive(this->pub);
}

int main(int argc, char** argv) {
  // Init
  ros::init(argc, argv, NODE_NAME);
  Controller controller;
  ROS_INFO("%s is ONLINE", NODE_NAME.c_str());

  // Set repeat freq
  ros::Rate rate(FREQ);

  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}