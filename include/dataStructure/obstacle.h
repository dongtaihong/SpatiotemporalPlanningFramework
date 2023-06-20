/*
 * @Author: 董泰宏 2396203400@qq.com
 * @Date: 2023-05-30 16:56:40
 * @LastEditors: 董泰宏 2396203400@qq.com
 * @LastEditTime: 2023-06-12 21:33:08
 * @FilePath: /local_planner/include/dataStructure/obstacle.h
 * @Description: 用于测试场景是否正常
 * Copyright (c) 2023 by 董泰宏 email: 2396203400@qq.com, All Rights Reserved.
 */
#pragma once
#include <chrono>
#include <ctime>

#include "trajectoryPoint.h"

struct Obstacle {
  TrajectoryPoint obsNow_;
  std::vector<TrajectoryPoint> predictTrajectory_;  //从当前时刻起，未来的6s轨迹
  int lane_;  // 障碍物的lane, 0左1右
  // 现在相当于这个框架只适合2车道的
  int index_;  //当前障碍物出生时在全局参考线上的投影参考点index,作用是为了获取出生点的轨迹点
};