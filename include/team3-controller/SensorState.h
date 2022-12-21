#ifndef SENSOR_STATE_H
#define SENSOR_STATE_H

#include <numeric>
#include <vector>

#include "sensor_cam/cam_msg.h"
// #include "sensor_lidar/lidar_msg.h"
// #include "sensor_sonic/sonic_msg.h"
// #include "sensor_imu/imu_msg.h"

constexpr int WIDTH = 640;
constexpr int SMA_NUM = 3;

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
  int lposSMA = std::accumulate(lposMemo.begin(), lposMemo.end(), 0) / SMA_NUM;
  int rposSMA = std::accumulate(rposMemo.begin(), rposMemo.end(), 0) / SMA_NUM;
}

#endif