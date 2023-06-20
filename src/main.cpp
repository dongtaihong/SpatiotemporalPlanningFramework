/*
 * @Author: 董泰宏 2396203400@qq.com
 * @Date: 2023-05-29 17:50:17
 * @LastEditors: 董泰宏 2396203400@qq.com
 * @LastEditTime: 2023-06-20 22:13:27
 * @FilePath: /local_planner/src/main.cpp
 * @Description:
 * Copyright (c) 2023 by 董泰宏 email: 2396203400@qq.com, All Rights Reserved.
 */

#include "PlannerFactory.h"
#include "SimulatorFactory.h"
#include "mpcPlanner.h"
#include "randomSimulatorFactory.h"
#include "testPlanner.h"

int main(int argc, char** argv) {
  //选择某个仿真器:1.随机场景仿真器；2.carla仿真器
  SimulatorFactory* sim_factory =
      new randomSimulatorFactory();  //使用工厂模式隔离变化。这里其实还没复杂到需要使用工厂模式，直接用策略模式更好
  Simulator* simulator = sim_factory->selectSimulator();

  //选择某个规划器:1.mpc规划器；2.test规划器
  PlannerFactory* planner_factory = new mpcPlannerFactory();
  Planner* planner = planner_factory->selectPlanner();

  while (true) {
    std::chrono::steady_clock::time_point tbegin = chrono::steady_clock::now();

    simulator->refreshScenario();  //更新当前帧的场景
    planner->plan(simulator);      //规划当前帧场景下的局部轨迹
    simulator->showScenario();     //可视化当前场景
    simulator->check10hz(tbegin);  //确保规划的周期为10hz
  }
  return 0;
}
