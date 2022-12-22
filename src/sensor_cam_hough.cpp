#include <cmath>
#include <iostream>
#include <string>
#include <vector>

// Include ROS
#include "ros/console.h"
#include "ros/ros.h"
#include "sensor_cam_hough/cam_msg.h"
#include "sensor_msgs/Image.h"

// Include OpenCV
#include "opencv2/opencv.hpp"

// Const
const std::string NODE_NAME = "sensor_cam_hough";
const std::string WINDOW_TITLE = "Camera_hough";
const std::string SUB_TOPIC = "usb_cam/image_raw";
const std::string PUB_TOPIC = "cam_hough_data";
const cv::Scalar WHITE = cv::Scalar(255, 255, 255);
const cv::Scalar BLACK = cv::Scalar(0, 0, 0);
const cv::Scalar RED = cv::Scalar(0, 0, 255);
const cv::Scalar GREEN = cv::Scalar(0, 255, 0);
const cv::Scalar BLUE = cv::Scalar(255, 0, 0);
const cv::Scalar YELLOW = cv::Scalar(0, 255, 255);

constexpr int ESC_KEY = 27;
constexpr int WIDTH = 640;
constexpr int HEIGHT = 480;
constexpr double GAUSIAN_BLUR_SIGMA = 2.;
constexpr int CANNY_LOW_THRESHOLD = 60;
constexpr int CANNY_HIGH_THRESHOLD = 70;
constexpr int CANNY_KERNEL_SIZE = 3;
constexpr double HOUGH_RHO = 1.;
constexpr double HOUGH_THETA = M_PI / 180;
constexpr int HOUGH_THRESHOLD = 30;
constexpr int HOUGH_MIN_LINE_LEN = 30;
constexpr int HOUGH_MAX_LINE_GAP = 10;
constexpr int LINE_SLOPE_LOW = 0;
constexpr int LINE_SLOPE_HIGH = 10;
constexpr int CENTER_GAP = 90;
constexpr int ROI_HEIGHT = 40;
constexpr int ROI_Y = 340;

const cv::Rect ROI_RECT = cv::Rect(0, ROI_Y, WIDTH, ROI_HEIGHT);

class Sensor {
  ros::NodeHandle node;
  ros::Subscriber sub;
  ros::Publisher pub;
  cv::Mat vFrame;
  bool isLeftDetected;
  bool isRightDetected;
  int lpos;
  int rpos;
  int lposFar;
  int rposFar;

  std::vector<double> getLineParams(const std::vector<cv::Vec4i>& lines);
  std::vector<std::vector<cv::Vec4i>> divideLines(
      std::vector<cv::Vec4i> lines) const;
  std::vector<cv::Point> getPoints(double m, double b, bool isRight) const;

public:
  Sensor()
      : isLeftDetected(false),
        isRightDetected(false),
        lpos(0),
        rpos(WIDTH - 1),
        lposFar(0),
        rposFar(WIDTH - 1) {
    sub = node.subscribe(SUB_TOPIC, 1, &Sensor::callback, this);
    pub = node.advertise<sensor_cam_hough::cam_msg>(PUB_TOPIC, 1);
    cv::namedWindow(WINDOW_TITLE);
  }

  void callback(const sensor_msgs::ImageConstPtr& msg);
  void process();
  void publish();
};

std::vector<double> Sensor::getLineParams(const std::vector<cv::Vec4i>& lines) {
  double sumX = 0., sumY = 0., sumM = 0.;
  int len = lines.size();
  if (!len) return std::vector<double>{0., 0.};

  for (int i = 0; i < lines.size(); i++) {
    cv::Vec4i iLine = lines[i];
    int x0 = iLine[0], y0 = iLine[1], x1 = iLine[2], y1 = iLine[3];

    double slope = ((double)y1 - y0) / ((double)x1 - x0);
    sumX += x0 + x1;
    sumY += y0 + y1;
    sumM += slope;
  }

  double avrX = sumX / (len * 2);
  double avrY = sumY / (len * 2);
  double m = sumM / len;
  double b = avrY - m * avrX;
  return std::vector<double>{m, b};
}

std::vector<std::vector<cv::Vec4i>> Sensor::divideLines(
    std::vector<cv::Vec4i> lines) const {
  std::vector<cv::Vec4i> left;
  std::vector<cv::Vec4i> right;
  left.reserve(lines.size());
  right.reserve(lines.size());

  for (int i = 0; i < lines.size(); i++) {
    cv::Vec4i iLine = lines[i];
    int x0 = iLine[0], y0 = iLine[1], x1 = iLine[2], y1 = iLine[3];
    if (x0 == x1) continue;

    double slope = ((double)y1 - y0) / ((double)x1 - x0);
    if (std::abs(slope) > LINE_SLOPE_LOW && std::abs(slope) < LINE_SLOPE_HIGH) {
      if (slope > 0 && x1 > WIDTH / 2 + CENTER_GAP) right.emplace_back(iLine);
      if (slope < 0 && x0 < WIDTH / 2 - CENTER_GAP) left.emplace_back(iLine);
    }
  }
  return std::vector<std::vector<cv::Vec4i>>{left, right};
}

std::vector<cv::Point> Sensor::getPoints(double m, double b,
                                         bool isRight) const {
  int x0 = 0, x1 = 0, xp = 0, yp = ROI_Y + (ROI_HEIGHT >> 1);
  if (m) {
    b += ROI_Y;
    x0 = (HEIGHT - b) / m;
    x1 = (HEIGHT / 2 - b) / m;
    xp = (yp - b) / m;
  } else if (isRight) {
    xp = WIDTH;
  }
  cv::Point p0(x0, HEIGHT);
  cv::Point p1(x1, HEIGHT / 2);
  cv::Point p2(xp, yp);
  return std::vector<cv::Point>{p0, p1, p2};
}

void Sensor::callback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    this->vFrame = cv::Mat(HEIGHT, WIDTH, CV_8UC3,
                           const_cast<uchar*>(&msg->data[0]), msg->step);
    // cv::TickMeter tm;
    // tm.start();
    this->process();
    // tm.stop();
    // std::cout << "Elapsed time: " << tm.getTimeMilli() << "ms." << '\n';
  } catch (const std::exception& e) {
    ROS_ERROR("callback exception: %s", e.what());
    return;
  }
}

void Sensor::process() {
  // Convert to Gray
  cv::Mat grayFrame, blurred, edged;
  cv::cvtColor(this->vFrame, grayFrame, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(grayFrame, blurred, cv::Size(), GAUSIAN_BLUR_SIGMA);
  cv::Canny(blurred, edged, CANNY_LOW_THRESHOLD, CANNY_HIGH_THRESHOLD,
            CANNY_KERNEL_SIZE);

  // ROI
  cv::Mat triangleMask = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
  std::vector<cv::Point> roiPts{
      cv::Point(WIDTH >> 1, HEIGHT >> 1), cv::Point(0, ROI_Y),
      cv::Point(0, ROI_Y + ROI_HEIGHT), cv::Point(WIDTH, ROI_Y + ROI_HEIGHT),
      cv::Point(WIDTH, ROI_Y)};
  cv::fillPoly(triangleMask, roiPts, WHITE, cv::LINE_AA);
  cv::Mat roi = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
  edged.copyTo(roi, triangleMask);

  // Hough TF
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(roi, lines, HOUGH_RHO, HOUGH_THETA, HOUGH_THRESHOLD,
                  HOUGH_MIN_LINE_LEN, HOUGH_MAX_LINE_GAP);
  const auto& sidedLines = this->divideLines(lines);
  const auto& lParam = this->getLineParams(sidedLines[0]);
  const auto& rParam = this->getLineParams(sidedLines[1]);
  const auto& lp = this->getPoints(lParam[0], lParam[1], false);
  const auto& rp = this->getPoints(rParam[0], rParam[1], true);

  this->isLeftDetected = true;
  this->isRightDetected = true;
  this->lpos = lp[2].x;
  this->rpos = rp[2].x;
  this->lposFar = lp[1].x;
  this->rposFar = rp[1].x;

  // for debugging
  // Lane extensions
  cv::line(this->vFrame, lp[0], lp[1], BLUE, 1, cv::LINE_AA);
  cv::line(this->vFrame, rp[0], rp[1], BLUE, 1, cv::LINE_AA);
  cv::polylines(this->vFrame, roiPts, true, RED, 1, cv::LINE_AA);
  cv::imshow(WINDOW_TITLE, this->vFrame);
  this->publish();
}

void Sensor::publish() {
  sensor_cam_hough::cam_msg msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = PUB_TOPIC;

  msg.width = WIDTH;
  msg.height = HEIGHT;
  msg.isLeftDetected = this->isLeftDetected;
  msg.isRightDetected = this->isRightDetected;
  msg.lpos = this->lpos;
  msg.rpos = this->rpos;
  msg.lposFar = this->lposFar;
  msg.rposFar = this->rposFar;

  this->pub.publish(msg);
  // ROS_INFO("lpos: %d | rpos: %d", lpos, rpos);
}

int main(int argc, char** argv) {
  // Init
  ros::init(argc, argv, NODE_NAME);
  Sensor sensor;
  ROS_INFO("%s is ONLINE", NODE_NAME.c_str());

  while (ros::ok()) {
    ros::spinOnce();

    // for debugging
    int k = cv::waitKey(1);
    if (k == ESC_KEY || k == ' ') break;
  }

  cv::destroyAllWindows();
  return 0;
}