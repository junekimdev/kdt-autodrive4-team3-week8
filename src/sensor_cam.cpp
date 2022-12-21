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
constexpr int SCAN_ROW = 380;
constexpr double GAUSIAN_BLUR_SIGMA = 2.;
constexpr int ROI_HEIGHT = 30;
constexpr int ROI_Y = SCAN_ROW - (ROI_HEIGHT / 2);
constexpr int ROI_GAP = 8;

inline std::vector<int> filterX(const std::vector<cv::Point>& pts,
                                const int minV, const int maxV,
                                const bool isLeft = true) {
  int lp = pts[0].x, rp = pts[1].x;

  // Offset rightside
  if (!isLeft) {
    lp += minV;
    rp += minV;
  }

  int distance = rp - lp;
  if (distance < 1) {
    lp = isLeft ? minV : maxV;
    rp = isLeft ? minV : maxV;
  }

  return {lp, rp};
}

inline std::vector<cv::Point> findEdges(const cv::Mat& img,
                                        const bool isLeft = true) {
  cv::Mat img32, blr, dx, dy, dst;
  img.convertTo(img32, CV_32F);
  cv::GaussianBlur(img32, blr, cv::Size(), GAUSIAN_BLUR_SIGMA);
  cv::Sobel(blr, dx, CV_32F, 1, 0);
  cv::Sobel(blr, dy, CV_32F, 0, 1);
  dst = dx.mul(dx) + dy.mul(dy);
  cv::sqrt(dst, dst);

  // for debugging
  cv::imshow("ROI", dst);

  double leftsideV1, rightsideV1, leftsideV2, rightsideV2, leftsideV3,
      rightsideV3;
  cv::Point leftsidePt1, rightsidePt1, leftsidePt2, rightsidePt2, leftsidePt3,
      rightsidePt3;

  int centerY = ROI_HEIGHT / 2;               // horizontal center line
  cv::Mat roi1 = dst.row(centerY);            // Line scanning
  cv::Mat roi2 = dst.row(centerY + ROI_GAP);  // Line scanning
  cv::Mat roi3 = dst.row(centerY - ROI_GAP);  // Line scanning
  cv::minMaxLoc(roi1, &leftsideV1, &rightsideV1, &leftsidePt1, &rightsidePt1);
  cv::minMaxLoc(roi2, &leftsideV2, &rightsideV2, &leftsidePt2, &rightsidePt2);
  cv::minMaxLoc(roi3, &leftsideV3, &rightsideV3, &leftsidePt3, &rightsidePt3);

  cv::Point leftsidePt, rightsidePt;
  leftsidePt.x =
      (int)((leftsidePt1.x + leftsidePt2.x + leftsidePt3.x) / 3.f + .5f);
  rightsidePt.x =
      (int)((rightsidePt1.x + rightsidePt2.x + rightsidePt3.x) / 3.f + .5f);

  return {leftsidePt, rightsidePt};
}

class Sensor {
  ros::NodeHandle node;
  ros::Subscriber sub;
  ros::Publisher pub;
  cv::Mat vFrame;
  bool isLeftDetected;
  bool isRightDetected;
  int lpos;
  int rpos;

public:
  Sensor() : isLeftDetected(false), isRightDetected(false), lpos(0), rpos(0) {
    sub = node.subscribe(SUB_TOPIC, 1, &Sensor::callback, this);
    pub = node.advertise<sensor_cam::cam_msg>(PUB_TOPIC, 1);
    cv::namedWindow(WINDOW_TITLE);
  }

  void callback(const sensor_msgs::ImageConstPtr& msg);
  void processImg();
  void publish();
};

void Sensor::callback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    this->vFrame = cv::Mat(HEIGHT, WIDTH, CV_8UC3,
                           const_cast<uchar*>(&msg->data[0]), msg->step);
    // cv::TickMeter tm;
    // tm.start();
    this->processImg();
    // tm.stop();
    // std::cout << "Elapsed time: " << tm.getTimeMilli() << "ms." << '\n';
  } catch (const std::exception& e) {
    ROS_ERROR("callback exception: %s", e.what());
    return;
  }
}

void Sensor::processImg() {
  // TODO:
  int roi_width = this->vFrame.cols / 2;

  // Convert to Gray
  cv::Mat grayFrame;
  cv::cvtColor(this->vFrame, grayFrame, cv::COLOR_BGR2GRAY);

  // Find lanes
  cv::Rect roiRectL(0, ROI_Y, roi_width - 1, ROI_HEIGHT);          // left half
  cv::Rect roiRectR(roi_width, ROI_Y, roi_width - 1, ROI_HEIGHT);  // right half
  cv::Mat roiL = grayFrame(roiRectL);
  cv::Mat roiR = grayFrame(roiRectR);
  std::vector<cv::Point> ptsL = findEdges(roiL);
  std::vector<cv::Point> ptsR = findEdges(roiR, false);
  std::vector<int> pxL = filterX(ptsL, 0, roi_width);
  std::vector<int> pxR = filterX(ptsR, roi_width, this->vFrame.cols - 1, false);
  int left = cvRound((pxL[0] + pxL[1]) / 2.f);
  int right = cvRound((pxR[0] + pxR[1]) / 2.f);

  // When undetected, lpos & rpos will be kept as previous values
  if (pxL[0] != 0 && pxL[1] != 0) {
    this->isLeftDetected = true;
    this->lpos = left;
  }
  if (pxR[0] != this->vFrame.cols - 1 && pxR[1] != this->vFrame.cols - 1) {
    this->isRightDetected = true;
    this->rpos = right;
  }

  // for debugging
  // cv::drawMarker(this->vFrame, cv::Point(pxL[0], SCAN_ROW), YELLOW,
  //                cv::MARKER_TILTED_CROSS, 10, 2, cv::LINE_AA);
  // cv::drawMarker(this->vFrame, cv::Point(pxL[1], SCAN_ROW), BLUE,
  //                cv::MARKER_TILTED_CROSS, 10, 2, cv::LINE_AA);
  // cv::drawMarker(this->vFrame, cv::Point(pxR[0], SCAN_ROW), YELLOW,
  //                cv::MARKER_TILTED_CROSS, 10, 2, cv::LINE_AA);
  // cv::drawMarker(this->vFrame, cv::Point(pxR[1], SCAN_ROW), BLUE,
  //                cv::MARKER_TILTED_CROSS, 10, 2, cv::LINE_AA);
  cv::line(this->vFrame, cv::Point(0, SCAN_ROW), cv::Point(WIDTH, SCAN_ROW),
           BLUE, 1);
  cv::line(this->vFrame, cv::Point(0, SCAN_ROW + ROI_GAP),
           cv::Point(WIDTH, SCAN_ROW + ROI_GAP), BLUE, 1);
  cv::line(this->vFrame, cv::Point(0, SCAN_ROW - ROI_GAP),
           cv::Point(WIDTH, SCAN_ROW - ROI_GAP), BLUE, 1);
  cv::imshow(WINDOW_TITLE, this->vFrame);
  this->publish();
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