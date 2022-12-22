#ifndef CONTROL_STATE_H
#define CONTROL_STATE_H

#include <cmath>
#include <numeric>
#include <vector>

#include "ros/console.h"
#include "ros/ros.h"
#include "sensor_cam/cam_msg.h"
#include "sensor_cam_hough/cam_msg.h"
#include "xycar_msgs/xycar_motor.h"

// Const
const std::string CONTROLLER_NAME = "controller";

constexpr int ANGLE_CENTER = 8;
constexpr int MAX_ANGLE = 50;
constexpr int MIN_ANGLE = -50;
constexpr int WIDTH = 640;
constexpr int SMA_NUM = 10;

enum struct DRIVE_MODE { STOP, GO };

struct Kalman1D {
  float Q;
  float R;
  float P;
  float S;
  float K;
  float Xest;
  float Xmeasured;
  float Phat;
  float PhatSqrt;

  Kalman1D(float Q, float R, float Xest, float Phat)
      : Q(Q),
        R(R),
        P(.5f),
        S(.5f),
        K(.5f),
        Xest(Xest),
        Xmeasured(.0f),
        Phat(Phat),
        PhatSqrt(std::sqrt(Phat)) {}

  void estimate(int Xmeasured) {
    // Get Kalman gain
    this->P = this->Phat + this->Q;
    this->S = this->P + this->R;
    this->K = this->P / this->S;

    // Estimate X
    this->Xmeasured = Xmeasured;
    this->Xest = this->Xest + this->K * (this->Xmeasured * this->Xest);

    // Update error
    this->Phat = (1 - this->K) * this->P;
    this->PhatSqrt = std::sqrt(this->Phat);
  }
};

struct ControlState {
  DRIVE_MODE mode;
  Kalman1D kalmanCam;
  Kalman1D kalmanHough;
  int angle;
  int speed;
  bool isStarted;

  ControlState()
      : mode(DRIVE_MODE::STOP),
        kalmanCam(.5, .1, 1, 100),
        kalmanHough(.5, .1, 1, 100),
        angle(ANGLE_CENTER),
        speed(0),
        isStarted(false) {}

  void reduce(DRIVE_MODE mode, int angle, int speed) {
    this->mode = mode;
    this->angle = angle;
    this->speed = speed;
  }
};

struct SensorCamState {
  int width;
  int lpos;
  int rpos;
  std::vector<int> lposMemo;
  std::vector<int> rposMemo;
  int lposSMA;
  int rposSMA;

  SensorCamState()
      : width(WIDTH),
        lpos(0),
        rpos(WIDTH - 1),
        lposMemo(std::vector<int>(SMA_NUM, 0)),
        rposMemo(std::vector<int>(SMA_NUM, WIDTH - 1)),
        lposSMA(0),
        rposSMA(WIDTH - 1) {}

  void reduce(const sensor_cam::cam_msg::ConstPtr& msg);
  void update(const sensor_cam::cam_msg::ConstPtr& msg);
  void filter();
};

void SensorCamState::reduce(const sensor_cam::cam_msg::ConstPtr& msg) {
  this->update(msg);
  this->filter();
}
void SensorCamState::update(const sensor_cam::cam_msg::ConstPtr& msg) {
  this->width = msg->width;
  if (msg->isLeftDetected) this->lpos = msg->lpos;
  if (msg->isRightDetected) this->rpos = msg->rpos;
}
void SensorCamState::filter() {
  lposMemo.erase(lposMemo.begin());
  rposMemo.erase(rposMemo.begin());
  lposMemo.emplace_back(this->lpos);
  rposMemo.emplace_back(this->rpos);
  this->lposSMA = (int)(std::accumulate(lposMemo.begin(), lposMemo.end(), 0) /
                            (float)SMA_NUM +
                        .5f);
  this->rposSMA = (int)(std::accumulate(rposMemo.begin(), rposMemo.end(), 0) /
                            (float)SMA_NUM +
                        .5f);
}
struct SensorCamHoughState {
  int width;
  int lpos;
  int rpos;
  int lposFar;
  int rposFar;
  std::vector<int> lposMemo;
  std::vector<int> rposMemo;
  int lposSMA;
  int rposSMA;
  std::vector<int> lposFarMemo;
  std::vector<int> rposFarMemo;
  int lposFarSMA;
  int rposFarSMA;

  SensorCamHoughState()
      : width(WIDTH),
        lpos(0),
        rpos(WIDTH - 1),
        lposFar(0),
        rposFar(WIDTH - 1),
        lposMemo(std::vector<int>(SMA_NUM, 0)),
        rposMemo(std::vector<int>(SMA_NUM, WIDTH - 1)),
        lposSMA(0),
        rposSMA(WIDTH - 1),
        lposFarMemo(std::vector<int>(SMA_NUM, 0)),
        rposFarMemo(std::vector<int>(SMA_NUM, WIDTH - 1)),
        lposFarSMA(0),
        rposFarSMA(WIDTH - 1) {}

  void reduce(const sensor_cam_hough::cam_msg::ConstPtr& msg);
  void update(const sensor_cam_hough::cam_msg::ConstPtr& msg);
  void filter();
};

void SensorCamHoughState::reduce(
    const sensor_cam_hough::cam_msg::ConstPtr& msg) {
  this->update(msg);
  this->filter();
}
void SensorCamHoughState::update(
    const sensor_cam_hough::cam_msg::ConstPtr& msg) {
  this->width = msg->width;
  if (msg->isLeftDetected) {
    this->lpos = msg->lpos;
    this->lposFar = msg->lposFar;
  }
  if (msg->isRightDetected) {
    this->rpos = msg->rpos;
    this->rposFar = msg->rposFar;
  }
}
void SensorCamHoughState::filter() {
  lposMemo.erase(lposMemo.begin());
  rposMemo.erase(rposMemo.begin());
  lposMemo.emplace_back(this->lpos);
  rposMemo.emplace_back(this->rpos);
  this->lposSMA = (int)(std::accumulate(lposMemo.begin(), lposMemo.end(), 0) /
                            (float)SMA_NUM +
                        .5f);
  this->rposSMA = (int)(std::accumulate(rposMemo.begin(), rposMemo.end(), 0) /
                            (float)SMA_NUM +
                        .5f);
  lposFarMemo.erase(lposFarMemo.begin());
  rposFarMemo.erase(rposFarMemo.begin());
  lposFarMemo.emplace_back(this->lposFar);
  rposFarMemo.emplace_back(this->rposFar);
  this->lposFarSMA =
      (int)(std::accumulate(lposFarMemo.begin(), lposFarMemo.end(), 0) /
                (float)SMA_NUM +
            .5f);
  this->rposFarSMA =
      (int)(std::accumulate(rposFarMemo.begin(), rposFarMemo.end(), 0) /
                (float)SMA_NUM +
            .5f);
}

struct SensorState {
  SensorCamState cam;
  SensorCamHoughState hough;

  SensorState() {}
};

#endif