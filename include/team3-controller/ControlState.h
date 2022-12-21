#ifndef CONTROL_STATE_H
#define CONTROL_STATE_H

#include <numeric>
#include <vector>

#include "ros/console.h"
#include "ros/ros.h"
#include "sensor_cam/cam_msg.h"
#include "xycar_msgs/xycar_motor.h"

// Const
const std::string CONTROLLER_NAME = "controller";

constexpr int ANGLE_CENTER = 8;
constexpr int MAX_ANGLE = 50;
constexpr int MIN_ANGLE = -50;
constexpr int MAX_SPEED = 30;
constexpr int WIDTH = 640;
constexpr int SMA_NUM = 10;

enum struct DRIVE_MODE { STOP, GO_SLOW, GO_FAST, TURN_SLOW };

struct ControlState {
  DRIVE_MODE mode;
  int angle;
  int speed;
  bool isStarted;

  ControlState()
      : mode(DRIVE_MODE::STOP),
        angle(ANGLE_CENTER),
        speed(0),
        isStarted(false) {}

  void reduce(DRIVE_MODE mode, int angle, int speed) {
    this->mode = mode;
    this->angle = angle;
    this->speed = speed;
  }
};

struct SensorState {
  int width;
  int lpos;
  int rpos;
  std::vector<int> lposMemo;
  std::vector<int> rposMemo;
  int lposSMA;
  int rposSMA;

  SensorState()
      : width(WIDTH),
        lpos(0),
        rpos(WIDTH - 1),
        lposMemo(std::vector<int>(SMA_NUM, 0)),
        rposMemo(std::vector<int>(SMA_NUM, WIDTH - 1)),
        lposSMA(0),
        rposSMA(WIDTH - 1) {}

  void reduceCamState(const sensor_cam::cam_msg::ConstPtr& msg);
  void updateCamState(const sensor_cam::cam_msg::ConstPtr& msg);
  void filterCamState();
};

void SensorState::reduceCamState(const sensor_cam::cam_msg::ConstPtr& msg) {
  this->updateCamState(msg);
  this->filterCamState();
}
void SensorState::updateCamState(const sensor_cam::cam_msg::ConstPtr& msg) {
  this->width = msg->width;
  if (msg->isLeftDetected) this->lpos = msg->lpos;
  if (msg->isRightDetected) this->rpos = msg->rpos;
}
void SensorState::filterCamState() {
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

#endif