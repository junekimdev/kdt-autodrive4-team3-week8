#include <string>

// Include ROS
#include "ros/console.h"
#include "ros/ros.h"
#include "xycar_msgs/xycar_motor.h"

// Const
constexpr std::string NODE_NAME = "test-driver";
constexpr std::string XYCAR_MOTOR_TOPIC = "xycar_motor";
constexpr int ANGLE = 0;
constexpr int SPEED = 10;

class Driver {
  ros::NodeHandle node;
  ros::Publisher pub;

public:
  Driver() {
    pub = node.advertise<xycar_msgs::xycar_motor>(XYCAR_MOTOR_TOPIC, 1);
  }

  void drive(int angle);
}

void Driver::drive(int speed) {
  xycar_msgs::xycar_motor msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = XYCAR_MOTOR_TOPIC;
  msg.angle = ANGLE;
  msg.speed = speed;
  this->pub.publish(msg);
  ROS_INFO("angle: %d, speed: %d", msg.angle, msg.speed);
}

int main(int argc, char** argv) {
  // Init
  ros::init(argc, argv, NODE_NAME);
  Driver driver;
  ROS_INFO("System is ONLINE");

  // Freq
  ros::Rate rate(2);  // 2Hz

  // Driving for 10s
  for (int i = 0; i < 20; i++) {
    ros::spinOnce();
    driver.drive(SPEED);
    rate.sleep();
  }

  // Stop
  while (ros::ok()) {
    ros::spinOnce();
    driver.drive(0);
    rate.sleep();
  }

  return 0;
}