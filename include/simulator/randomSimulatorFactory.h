/*
 * @Author: 董泰宏 2396203400@qq.com
 * @Date: 2023-05-30 17:21:43
 * @LastEditors: 董泰宏 2396203400@qq.com
 * @LastEditTime: 2023-06-07 17:30:37
 * @FilePath: /local_planner/include/simulator/randomSimulatorFactory.h
 * @Description: 自定义的仿真器————所有的障碍物随机生成
 * Copyright (c) 2023 by 董泰宏 email: 2396203400@qq.com, All Rights Reserved.
 */
#pragma once
#include <cmath>
#include <fstream>
#include <random>
#include <sstream>
#include <string>

#include "SimulatorFactory.h"

class randomSimulator : public Simulator {
 public:
  randomSimulator();  //构造函数时，则将该仿真器的地图读取
  void refreshScenario() override;
  ~randomSimulator() = default;
  void showScenario() override;
  void check10hz(std::chrono::steady_clock::time_point tbegin) override;

 private:
  //这四个步骤都是因为当前这个仿真器没有生成随机场景的能力，所以需要自己来手动创建随机场景，如果使用的是carla等仿真器，则不需要这三个函数
  bool isOverScenario();              //判断当前场景是否结束
  Obstacle generateRandomObstacle();  //随机生成动态障碍物
  void generateNextScenario();  //将新生成的动态障碍物放入场景当中
  std::vector<Obstacle> obstacle_60s_;
  int findIndex();
};

class randomSimulatorFactory : public SimulatorFactory {
 public:
  Simulator* selectSimulator() override { return new randomSimulator(); };
  ~randomSimulatorFactory() = default;
};