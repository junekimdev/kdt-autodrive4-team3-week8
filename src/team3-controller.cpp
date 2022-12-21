#include <cmath>
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

// Include States
#include "team3-controller/ControlState.h"
#include "team3-controller/SensorState.h"

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
  SensorState& sensorState;
  ControlState& controlState;

  void correctAngle() {
    this->controlState.angle += ANGLE_CENTER;
    if (this->controlState.angle > MAX_ANGLE)
      this->controlState.angle = MAX_ANGLE;
    if (this->controlState.angle < MIN_ANGLE)
      this->controlState.angle = MIN_ANGLE;
  }

public:
  Controller() : sensorState(SensorState()) {
    this->sub_cam =
        this->node.subscribe(SUB_TOPIC_CAM, 1, &Controller::callbackCam, this);
    // this->sub_lidar = node.subscribe(SUB_TOPIC_LIDAR, 1,
    // &Controller::callbackLidar, this);
    this->controlState = ControlState(
        this->node.advertise<xycar_msgs::xycar_motor>(PUB_TOPIC, 1));
  }

  void callbackCam(const sensor_cam::cam_msg::ConstPtr& msg);
  // void callbackLidar(const sensor_lidar::lidar_msg::ConstPtr& msg);
  void control();
  void start();
};

void Controller::callbackCam(const sensor_cam::cam_msg::ConstPtr& msg) {
  this->sensorState.reduceCamState(msg);
  this->control();
}

// void Controller::callbackLidar(const sensor_lidar::lidar_msg::ConstPtr& msg)
// {
//   ROS_INFO("CON:: %d", msg->size)
// }

// TODO:
void Controller::control() {
  // Decide angle
  float viewCenter = sensorState.width / 2.f;
  float laneCenter = (sensorState.lposSMA + sensorState.rposSMA) / 2.f;
  int angle = (int)((laneCenter - viewCenter) / 3.f + .5f);  // Round half up
  angle = correctAngle(angle);
  DRIVE_MODE mode = (std::abs(angle) < 5) DRIVE.GO_SLOW : DRIVE.TURN_SLOW;
  int speed = mode == DRIVE.GO_SLOW || mode == DRIVE.TURN_SLOW ? 10 : 30;

  if (this->controlState.isStarted)
    this->controlState.reduce(mode, angle, speed);
}

void Controller::start() { this->controlState.isStarted = true; }

int main(int argc, char** argv) {
  // Init
  ros::init(argc, argv, NODE_NAME);
  Controller controller;
  ROS_INFO("%s is ONLINE", NODE_NAME.c_str());

  // Set repeat freq
  ros::Rate rate(FREQ);

  controller.start();
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}