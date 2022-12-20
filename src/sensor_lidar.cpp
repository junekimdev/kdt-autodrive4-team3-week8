#include <string>
#include <vector>

// Include ROS
#include "ros/console.h"
#include "ros/ros.h"
#include "sensor_cam/lidar_msg.h"
#include "sensor_msgs/LaserScan.h"

// Const
const std::string NODE_NAME = "sensor_lidar";
const std::string SUB_TOPIC = "scan";
const std::string PUB_TOPIC = "sensor_data";

class Sensor {
  ros::NodeHandle node;
  ros::Subscriber sub;
  ros::Publisher pub;
  vector<float> objects;

public:
  Sensor() {
    sub = node.subscribe(SUB_TOPIC, 1, &Sensor::callback, this);
    pub = node.advertise<sensor_cam::lidar_msg>(PUB_TOPIC, 1);
  }

  void callback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void processData();
  void publish();
};

void Sensor::callback(const sensor_msgs::LaserScanConstPtr& msg) {
  try {
    // TODO:
    msg->ranges;
    this->processData();
  } catch (const std::exception& e) {
    ROS_ERROR("Lidar callback exception: %s", e.what());
    return;
  }
}

void Sensor::processData() {
  // TODO:
}

void Sensor::publish() {
  sensor_cam::lidar_msg msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = PUB_TOPIC;
  msg.size = this->objects.size();
  msg.object_distances = &this->objects[0];

  this->pub.publish(msg);
  ROS_INFO("Lidar detected objects: " + this->objects.size());
}

int main(int argc, char** argv) {
  // Init
  ros::init(argc, argv, NODE_NAME);
  Sensor sensor;
  ROS_INFO(NODE_NAME + " is ONLINE");

  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}