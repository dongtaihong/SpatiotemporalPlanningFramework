/*
 * @Author: 董泰宏 2396203400@qq.com
 * @Date: 2023-05-29 17:50:17
 * @LastEditors: 董泰宏 2396203400@qq.com
 * @LastEditTime: 2023-06-12 11:23:36
 * @FilePath: /local_planner/include/planner/testPlanner.h
 * @Description:
 * Copyright (c) 2023 by 董泰宏 email: 2396203400@qq.com, All Rights Reserved.
 */
#include <cmath>
#include <limits>

#include "PlannerFactory.h"
#include "SimulatorFactory.h"

#define INF 60000

//传入simulator的场景信息，输出主车的局部轨迹
class testPlanner : public Planner {
 public:
  testPlanner() = default;
  std::vector<TrajectoryPoint> plan(Simulator* simulator) override;
  ~testPlanner() = default;
};

class testPlannerFactory : public PlannerFactory {
 public:
  Planner* selectPlanner() override { return new testPlanner(); };
  ~testPlannerFactory() = default;
};