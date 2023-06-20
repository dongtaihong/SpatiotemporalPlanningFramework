/*
 * @Author: 董泰宏 2396203400@qq.com
 * @Date: 2023-05-30 17:14:15
 * @LastEditors: 董泰宏 2396203400@qq.com
 * @LastEditTime: 2023-06-12 23:57:54
 * @FilePath: /local_planner/include/dataStructure/ego.h
 * @Description:
 * Copyright (c) 2023 by 董泰宏 email: 2396203400@qq.com, All Rights Reserved.
 */
#pragma once
#include <chrono>
#include <ctime>
#include <vector>

#include "trajectoryPoint.h"

struct Ego {
  TrajectoryPoint egoNow_;                        //当前的定位
  std::vector<TrajectoryPoint> localTrajectory_;  //局部轨迹
  int globalIndex_;  //当前ego在全局参考线上的投影参考点index(需要实时的去匹配)，这里记录一下index是为了方便缩小匹配的范围
  double vTarget_ = 10.0;  //期望速度
};