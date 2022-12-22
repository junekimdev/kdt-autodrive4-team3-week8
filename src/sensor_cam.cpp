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
constexpr int SCAN_ROW2 = 320;
constexpr double GAUSIAN_BLUR_SIGMA = 2.;
constexpr int ROI_HEIGHT = 30;
constexpr int ROI_Y = SCAN_ROW - (ROI_HEIGHT >> 1);
constexpr int ROI_Y2 = SCAN_ROW2 - (ROI_HEIGHT >> 1);
constexpr int ROI_GAP = 8;

const cv::Size ROI_SIZE_FULL = cv::Size(WIDTH, ROI_HEIGHT);
const cv::Size ROI_SIZE_WIDE = cv::Size(WIDTH >> 1, ROI_HEIGHT);
const cv::Size ROI_SIZE_NORM = cv::Size(WIDTH >> 2, ROI_HEIGHT);
const cv::Rect ROI_FULL = cv::Rect(0, ROI_Y, WIDTH, ROI_HEIGHT);
const cv::Rect ROI_L_NULL = cv::Rect(0, ROI_Y, 1, ROI_HEIGHT);
const cv::Rect ROI_R_NULL = cv::Rect(WIDTH - 1, ROI_Y, 1, ROI_HEIGHT);
const cv::Rect ROI_L_INIT = cv::Rect(cv::Point(0, ROI_Y), ROI_SIZE_WIDE);
const cv::Rect ROI_R_INIT =
    cv::Rect(cv::Point(WIDTH >> 1, ROI_Y), ROI_SIZE_WIDE);
const cv::Rect ROI_FULL2 = cv::Rect(0, ROI_Y2, WIDTH, ROI_HEIGHT);
const cv::Rect ROI_L_NULL2 = cv::Rect(0, ROI_Y2, 1, ROI_HEIGHT);
const cv::Rect ROI_R_NULL2 = cv::Rect(WIDTH - 1, ROI_Y2, 1, ROI_HEIGHT);
const cv::Rect ROI_L_INIT2 = cv::Rect(cv::Point(0, ROI_Y2), ROI_SIZE_WIDE);
const cv::Rect ROI_R_INIT2 =
    cv::Rect(cv::Point(WIDTH >> 1, ROI_Y2), ROI_SIZE_WIDE);

inline std::vector<int> filterX(const std::vector<cv::Point>& pts,
                                const int topLeftX, const int resetV) {
  int lp = pts[0].x, rp = pts[1].x;

  // Offset rightside
  lp += topLeftX;
  rp += topLeftX;

  int distance = rp - lp;
  if (distance < 1) {
    lp = resetV;
    rp = resetV;
  }

  return {lp, rp};
}

inline std::vector<cv::Point> findEdges(const cv::Mat& img) {
  cv::Mat img32, blr, dx;
  img.convertTo(img32, CV_32F);
  cv::GaussianBlur(img32, blr, cv::Size(), GAUSIAN_BLUR_SIGMA);
  cv::Sobel(blr, dx, CV_32F, 1, 0);

  double leftsideV1, rightsideV1, leftsideV2, rightsideV2, leftsideV3,
      rightsideV3;
  cv::Point leftsidePt1, rightsidePt1, leftsidePt2, rightsidePt2, leftsidePt3,
      rightsidePt3;

  int centerY = ROI_HEIGHT / 2;              // horizontal center line
  cv::Mat roi1 = dx.row(centerY);            // Line scanning
  cv::Mat roi2 = dx.row(centerY + ROI_GAP);  // Line scanning
  cv::Mat roi3 = dx.row(centerY - ROI_GAP);  // Line scanning
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

inline cv::Rect getRoiRectL(int lx, int w, int scanRow, int rightCut = WIDTH) {
  // Prep args
  if (lx < 0) lx = 0;
  if (rightCut < 1) rightCut = 1;
  if (lx == rightCut) lx--;

  int rx = lx + w;
  if (rx > rightCut) rx = rightCut;
  return cv::Rect(cv::Point(lx, scanRow), cv::Point(rx, scanRow + ROI_HEIGHT));
}

inline cv::Rect getRoiRectR(int rx, int w, int scanRow, int leftCut = 0) {
  // Prep args
  if (rx > WIDTH) rx = WIDTH;
  if (leftCut > WIDTH - 1) leftCut = WIDTH - 1;
  if (rx == leftCut) rx++;

  int lx = rx - w;
  if (lx < leftCut) lx = leftCut;
  return cv::Rect(cv::Point(lx, scanRow), cv::Point(rx, scanRow + ROI_HEIGHT));
}

class Sensor {
  ros::NodeHandle node;
  ros::Subscriber sub;
  ros::Publisher pub;
  cv::Mat vFrame;
  cv::Rect roiRectL;
  cv::Rect roiRectR;
  bool isLeftDetected;
  bool isRightDetected;
  int lpos;
  int rpos;
  cv::Rect roiRectL2;
  cv::Rect roiRectR2;
  bool isLeftDetected2;
  bool isRightDetected2;
  int lpos2;
  int rpos2;

public:
  Sensor()
      : roiRectL(ROI_L_INIT),
        roiRectR(ROI_R_INIT),
        isLeftDetected(false),
        isRightDetected(false),
        lpos(0),
        rpos(WIDTH - 1),
        roiRectL2(ROI_L_INIT2),
        roiRectR2(ROI_R_INIT2),
        isLeftDetected2(false),
        isRightDetected2(false),
        lpos2(0),
        rpos2(WIDTH - 1) {
    sub = node.subscribe(SUB_TOPIC, 1, &Sensor::callback, this);
    pub = node.advertise<sensor_cam::cam_msg>(PUB_TOPIC, 1);
    cv::namedWindow(WINDOW_TITLE);
  }

  void callback(const sensor_msgs::ImageConstPtr& msg);
  void process();
  void publish();
  void publish2();
};

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
  // TODO:
  int roi_width = this->vFrame.cols / 2;

  // Convert to Gray
  cv::Mat grayFrame;
  cv::cvtColor(this->vFrame, grayFrame, cv::COLOR_BGR2GRAY);

  // Find lines
  cv::Mat roiL = grayFrame(this->roiRectL);
  cv::Mat roiR = grayFrame(this->roiRectR);
  std::vector<cv::Point> ptsL = findEdges(roiL);
  std::vector<cv::Point> ptsR = findEdges(roiR);
  std::vector<int> pxL = filterX(ptsL, this->roiRectL.x, 0);
  std::vector<int> pxR = filterX(ptsR, this->roiRectR.x, WIDTH - 1);

  int left = cvRound((pxL[0] + pxL[1]) / 2.f);
  int right = cvRound((pxR[0] + pxR[1]) / 2.f);

  // Update roi for next
  // When undetected, lpos & rpos will be kept as previous values
  bool goodL = pxL[0] != pxL[1];
  bool goodR = pxR[0] != pxR[1];
  if (goodL && goodR) {
    // None lost
    this->lpos = left;
    this->rpos = right;

    int lx = left - (ROI_SIZE_NORM.width >> 1);
    int rx = right + (ROI_SIZE_NORM.width >> 1);
    int mid = (int)((left + right) / 2.f + .5f);
    this->roiRectL = getRoiRectL(lx, ROI_SIZE_NORM.width, SCAN_ROW, mid);
    this->roiRectR = getRoiRectR(rx, ROI_SIZE_NORM.width, SCAN_ROW, mid);

  } else if (goodL) {
    // Right line lost
    this->lpos = left;

    int lx = left - (ROI_SIZE_NORM.width >> 1);
    int rx = right + (ROI_SIZE_WIDE.width >> 1);
    this->roiRectL = getRoiRectL(lx, ROI_SIZE_NORM.width,
                                 SCAN_ROW);  // the order is important
    this->roiRectR =
        getRoiRectR(rx, ROI_SIZE_WIDE.width, SCAN_ROW, this->roiRectL.br().x);

  } else if (goodR) {
    // Left line lost
    this->rpos = right;

    int lx = left - (ROI_SIZE_WIDE.width >> 1);
    int rx = right + (ROI_SIZE_NORM.width >> 1);
    this->roiRectR = getRoiRectR(rx, ROI_SIZE_NORM.width,
                                 SCAN_ROW);  // the order is important
    this->roiRectL =
        getRoiRectL(lx, ROI_SIZE_WIDE.width, SCAN_ROW, this->roiRectR.tl().x);

  } else {
    // All lost
    if (this->isLeftDetected) {
      // Stand by to find L line
      this->roiRectL = ROI_FULL;
      this->roiRectR = ROI_R_NULL;
    } else if (this->isRightDetected) {
      // Stand by to find R line
      this->roiRectL = ROI_L_NULL;
      this->roiRectR = ROI_FULL;
    }
  }
  this->isLeftDetected = goodL;
  this->isRightDetected = goodR;

  this->publish();

  // 22222222222

  cv::Mat roiL2 = grayFrame(this->roiRectL2);
  cv::Mat roiR2 = grayFrame(this->roiRectR2);
  std::vector<cv::Point> ptsL2 = findEdges(roiL2);
  std::vector<cv::Point> ptsR2 = findEdges(roiR2);
  std::vector<int> pxL2 = filterX(ptsL2, this->roiRectL2.x, 0);
  std::vector<int> pxR2 = filterX(ptsR2, this->roiRectR2.x, WIDTH - 1);

  int left2 = cvRound((pxL2[0] + pxL2[1]) / 2.f);
  int right2 = cvRound((pxR2[0] + pxR2[1]) / 2.f);

  // Update roi for next
  // When undetected, lpos & rpos will be kept as previous values
  bool goodL2 = pxL2[0] != pxL2[1];
  bool goodR2 = pxR2[0] != pxR2[1];
  if (goodL2 && goodR2) {
    // None lost
    this->lpos2 = left2;
    this->rpos2 = right2;

    int lx2 = left2 - (ROI_SIZE_NORM.width >> 1);
    int rx2 = right2 + (ROI_SIZE_NORM.width >> 1);
    int mid2 = (int)((left2 + right2) / 2.f + .5f);
    this->roiRectL2 = getRoiRectL(lx2, ROI_SIZE_NORM.width, SCAN_ROW2, mid2);
    this->roiRectR2 = getRoiRectR(rx2, ROI_SIZE_NORM.width, SCAN_ROW2, mid2);

  } else if (goodL2) {
    // Right line lost
    this->lpos2 = left2;

    int lx2 = left2 - (ROI_SIZE_NORM.width >> 1);
    int rx2 = right2 + (ROI_SIZE_WIDE.width >> 1);
    this->roiRectL2 = getRoiRectL(lx2, ROI_SIZE_NORM.width,
                                  SCAN_ROW2);  // the order is important
    this->roiRectR2 = getRoiRectR(rx2, ROI_SIZE_WIDE.width, SCAN_ROW2,
                                  this->roiRectL2.br().x);

  } else if (goodR2) {
    // Left line lost
    this->rpos2 = right2;

    int lx2 = left2 - (ROI_SIZE_WIDE.width >> 1);
    int rx2 = right2 + (ROI_SIZE_NORM.width >> 1);
    this->roiRectR2 = getRoiRectR(rx2, ROI_SIZE_NORM.width,
                                  SCAN_ROW2);  // the order is important
    this->roiRectL2 = getRoiRectL(lx2, ROI_SIZE_WIDE.width, SCAN_ROW2,
                                  this->roiRectR2.tl().x);

  } else {
    // All lost
    if (this->isLeftDetected2) {
      // Stand by to find L line
      this->roiRectL2 = ROI_FULL2;
      this->roiRectR2 = ROI_R_NULL2;
    } else if (this->isRightDetected2) {
      // Stand by to find R line
      this->roiRectL2 = ROI_L_NULL2;
      this->roiRectR2 = ROI_FULL2;
    }
  }
  this->isLeftDetected2 = goodL2;
  this->isRightDetected2 = goodR2;

  // for debugging
  cv::rectangle(this->vFrame, this->roiRectL, BLACK, 2);
  cv::rectangle(this->vFrame, this->roiRectR, BLACK, 2);
  // cv::drawMarker(this->vFrame, cv::Point(pxL[0], SCAN_ROW), YELLOW,
  //                cv::MARKER_TILTED_CROSS, 10, 2, cv::LINE_AA);
  // cv::drawMarker(this->vFrame, cv::Point(pxL[1], SCAN_ROW), BLUE,
  //                cv::MARKER_TILTED_CROSS, 10, 2, cv::LINE_AA);
  // cv::drawMarker(this->vFrame, cv::Point(pxR[0], SCAN_ROW), YELLOW,
  //                cv::MARKER_TILTED_CROSS, 10, 2, cv::LINE_AA);
  // cv::drawMarker(this->vFrame, cv::Point(pxR[1], SCAN_ROW), BLUE,
  //                cv::MARKER_TILTED_CROSS, 10, 2, cv::LINE_AA);
  cv::drawMarker(this->vFrame, cv::Point(this->lpos, SCAN_ROW), YELLOW,
                 cv::MARKER_TILTED_CROSS, 10, 2, cv::LINE_AA);
  cv::drawMarker(this->vFrame, cv::Point(this->rpos, SCAN_ROW), BLUE,
                 cv::MARKER_TILTED_CROSS, 10, 2, cv::LINE_AA);
  cv::line(this->vFrame, cv::Point(0, SCAN_ROW), cv::Point(WIDTH, SCAN_ROW),
           BLUE, 1);
  cv::line(this->vFrame, cv::Point(0, SCAN_ROW + ROI_GAP),
           cv::Point(WIDTH, SCAN_ROW + ROI_GAP), BLUE, 1);
  cv::line(this->vFrame, cv::Point(0, SCAN_ROW - ROI_GAP),
           cv::Point(WIDTH, SCAN_ROW - ROI_GAP), BLUE, 1);

  cv::rectangle(this->vFrame, this->roiRectL2, BLACK, 2);
  cv::rectangle(this->vFrame, this->roiRectR2, BLACK, 2);
  cv::drawMarker(this->vFrame, cv::Point(this->lpos2, SCAN_ROW2), YELLOW,
                 cv::MARKER_TILTED_CROSS, 10, 2, cv::LINE_AA);
  cv::drawMarker(this->vFrame, cv::Point(this->rpos2, SCAN_ROW2), BLUE,
                 cv::MARKER_TILTED_CROSS, 10, 2, cv::LINE_AA);
  cv::line(this->vFrame, cv::Point(0, SCAN_ROW2), cv::Point(WIDTH, SCAN_ROW2),
           BLUE, 1);
  cv::line(this->vFrame, cv::Point(0, SCAN_ROW2 + ROI_GAP),
           cv::Point(WIDTH, SCAN_ROW2 + ROI_GAP), BLUE, 1);
  cv::line(this->vFrame, cv::Point(0, SCAN_ROW2 - ROI_GAP),
           cv::Point(WIDTH, SCAN_ROW2 - ROI_GAP), BLUE, 1);
  cv::imshow(WINDOW_TITLE, this->vFrame);
  this->publish2();
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
}

void Sensor::publish2() {
  sensor_cam::cam_msg msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = PUB_TOPIC;

  msg.width = WIDTH;
  msg.height = HEIGHT;
  msg.scanRow = SCAN_ROW2;
  msg.isLeftDetected = this->isLeftDetected2;
  msg.isRightDetected = this->isRightDetected2;
  msg.lpos = this->lpos2;
  msg.rpos = this->rpos2;

  this->pub.publish(msg);
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