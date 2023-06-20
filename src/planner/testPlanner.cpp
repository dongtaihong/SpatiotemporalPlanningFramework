/*
 * @Author: 董泰宏 2396203400@qq.com
 * @Date: 2023-06-06 22:41:16
 * @LastEditors: 董泰宏 2396203400@qq.com
 * @LastEditTime: 2023-06-12 11:24:07
 * @FilePath: /local_planner/src/planner/testPlanner.cpp
 * @Description:
 * Copyright (c) 2023 by 董泰宏 email: 2396203400@qq.com, All Rights Reserved.
 */
#include "testPlanner.h"

std::vector<TrajectoryPoint> testPlanner::plan(Simulator* simulator) {
  simulator->ego_.localTrajectory_.clear();
  for (int i = 0; i < 50; i++) {
    TrajectoryPoint temp_point;
    temp_point.x = simulator->ego_.egoNow_.x + 2 * i;
    temp_point.y = simulator->ego_.egoNow_.y;
    temp_point.s = simulator->ego_.egoNow_.s + 2 * i;
    temp_point.theta = simulator->ego_.egoNow_.theta;
    simulator->ego_.localTrajectory_.emplace_back(temp_point);
  }
  return std::vector<TrajectoryPoint>();
}