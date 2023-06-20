/*
 * @Author: 董泰宏 2396203400@qq.com
 * @Date: 2023-05-30 16:42:48
 * @LastEditors: 董泰宏 2396203400@qq.com
 * @LastEditTime: 2023-06-07 17:28:46
 * @FilePath: /local_planner/include/simulator/SimulatorFactory.h
 * @Description:
 * Copyright (c) 2023 by 董泰宏 email: 2396203400@qq.com, All Rights Reserved.
 */
/*
 * @Author: 董泰宏 2396203400@qq.com
 * @Date: 2023-05-30 16:42:48
 * @LastEditors: 董泰宏 2396203400@qq.com
 * @LastEditTime: 2023-06-07 17:15:18
 * @FilePath: /local_planner/include/simulator/SimulatorFactory.h
 * @Description:仿真器的输入是全局路径的文件、当前的主车信息话题、当前的障碍物信息话题
 * Copyright (c) 2023 by 董泰宏 email: 2396203400@qq.com, All Rights Reserved.
 */
#pragma once
#include <thread>

#include "ego.h"
#include "matplotlibcpp.h"
#include "obstacle.h"
#include "trajectoryPoint.h"

namespace plt = matplotlibcpp;
using namespace std;

//输入：1.主车的信息话题（或者其他形式的信息来源）
//     2.当前的障碍物信息话题（或者其他形式的信息来源）
//输出：1.全局参考线及道路边界信息； ok
//     2.主车的定位、运动信息；
//     3.障碍物的定位、运动信息; ok
//     4.仿真开始的时间
class Simulator {
 public:
  //不同的仿真器刷新场景的方式不同：1.通过话题刷新；2.自定义刷新规则。总之都是刷新主车与障碍物的最新信息
  virtual void refreshScenario() = 0;
  virtual ~Simulator() {}
  virtual void showScenario() = 0;
  virtual void check10hz(std::chrono::steady_clock::time_point tbegin) = 0;

  // 所有仿真器的接口应该是统一的，统一提供：
  // 1.全局参考线及道路边界信息
  std::vector<TrajectoryPoint> referenceLine_;  //主车参考线
  std::vector<TrajectoryPoint> leftRoad_;       //道路左边界
  std::vector<TrajectoryPoint> rightRoad_;      //道路右边界
  std::vector<TrajectoryPoint> centerRoad_;     //道路中心线
  std::vector<TrajectoryPoint> leftLane_;       //道路左车道中心
  std::vector<TrajectoryPoint> rightLane_;      //道路右车道中心

  // 2.主车的定位、运动信息、当前局部轨迹信息
  Ego ego_;

  // 3.障碍物的定位、运动信息、未来6s预测轨迹————这其实就是场景信息（场景就是动态障碍物的集合）
  std::vector<Obstacle> obstacles_;

  // 4.仿真启动的时间
  std::chrono::time_point<std::chrono::system_clock> startTime_;
};

class SimulatorFactory {
 public:
  virtual Simulator* selectSimulator() = 0;
  virtual ~SimulatorFactory() {}
};