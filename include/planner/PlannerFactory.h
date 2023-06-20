
/*
 * @Author: 董泰宏 2396203400@qq.com
 * @Date: 2023-05-30 16:31:19
 * @LastEditors: 董泰宏 2396203400@qq.com
 * @LastEditTime: 2023-06-12 10:50:04
 * @FilePath: /local_planner/include/planner/PlannerFactory.h
 * @Description: 所有规划器的工厂类
 * Copyright (c) 2023 by 董泰宏 email: 2396203400@qq.com, All Rights Reserved.
 */
#pragma once
#include "SimulatorFactory.h"

class Planner {
 public:
  virtual std::vector<TrajectoryPoint> plan(Simulator* simulator) = 0;
  virtual ~Planner() {}

  //所有规划器的接口应该是统一的，局部轨迹为当前位置出发，含当前位置的轨迹序列
  std::vector<TrajectoryPoint> localTrajectory_;
  std::vector<double> warm_start_;
};

class PlannerFactory {
 public:
  virtual Planner* selectPlanner() = 0;
  virtual ~PlannerFactory() {}
};