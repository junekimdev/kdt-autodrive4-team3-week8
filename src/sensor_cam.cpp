#include <string>

// Include ROS
#include "ros/console.h"
#include "ros/ros.h"
#include "sensor_cam/cam_msg.h"
#include "sensor_msgs/Image.h"

// Include OpenCV
#include "opencv2/opencv.hpp"

// Const
const std::string NODE_NAME = "sensor_cam";
const std::string WINDOW_TITLE = "Camera";
const std::string SUB_TOPIC = "usb_cam/image_raw";
const std::string PUB_TOPIC = "cam_data";
const cv::Scalar WHITE = cv::Scalar(255, 255, 255);
const cv::Scalar BLACK = cv::Scalar(0, 0, 0);
const cv::Scalar RED = cv::Scalar(0, 0, 255);
const cv::Scalar GREEN = cv::Scalar(0, 255, 0);
const cv::Scalar BLUE = cv::Scalar(255, 0, 0);
const cv::Scalar YELLOW = cv::Scalar(0, 255, 255);

constexpr int ESC_KEY = 27;
constexpr int WIDTH = 640;
constexpr int HEIGHT = 480;
constexpr int SCAN_ROW = 400;
// constexpr int ROI_TOP = 420;
// constexpr int ROI_HEIGHT = 20;

class Sensor {
  ros::NodeHandle node;
  ros::Subscriber sub;
  ros::Publisher pub;
  cv::Mat myImg;
  bool isLeftDetected;
  bool isRightDetected;
  int lpos;
  int rpos;

public:
  Sensor() : isLeftDetected(false), isRightDetected(false), lpos(0), rpos(0) {
    sub = node.subscribe(SUB_TOPIC, 1, &Sensor::imageCallback, this);
    pub = node.advertise<sensor_cam::cam_msg>(PUB_TOPIC, 1);
    cv::namedWindow(WINDOW_TITLE);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void processImg();
  void publish();
};

void Sensor::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    this->myImg = cv::Mat(HEIGHT, WIDTH, CV_8UC3,
                          const_cast<uchar*>(&msg->data), msg->step);
    this->processImg();
  } catch (const std::exception& e) {
    ROS_ERROR("imageCallback exception: %s", e.what());
    return;
  }
}

void Sensor::processImg() {
  // TODO:
  cv::line(this->myImg, cv::Point(0, SCAN_ROW), cv::Point(WIDTH, SCAN_ROW),
           BLUE, 1);
}

void Sensor::publish() {
  sensor_cam::cam_msg msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = PUB_TOPIC;

  msg.width = WIDTH;
  msg.height = HEIGHT;
  msg.scanRow = SCAN_ROW;
  msg.isLeftDetected = this->isLeftDetected;
  msg.isRightDetected = this->isRightDetected;
  msg.lpos = this->lpos;
  msg.rpos = this->rpos;

  this->pub.publish(msg);
  ROS_INFO("lpos: " + lpos + " | rpos: " + rpos);
}

int main(int argc, char** argv) {
  // Init
  ros::init(argc, argv, NODE_NAME);
  Sensor sensor;
  ROS_INFO(NODE_NAME + " is ONLINE");

  while (ros::ok()) {
    ros::spinOnce();

    // for debugging
    cv::imshow(WINDOW_TITLE, this->myImg);
    int k = cv::wait_key(1);
    if (k == ESC_KEY || k == ' ') break;
  }

  cv::destroyAllWindows();
  return 0;
}