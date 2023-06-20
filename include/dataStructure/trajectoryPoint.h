/*
 * @Author: 董泰宏 2396203400@qq.com
 * @Date: 2023-05-30 16:43:35
 * @LastEditors: 董泰宏 2396203400@qq.com
 * @LastEditTime: 2023-06-12 21:44:34
 * @FilePath: /local_planner/include/dataStructure/trajectoryPoint.h
 * @Description:
 * Copyright (c) 2023 by 董泰宏 email: 2396203400@qq.com, All Rights Reserved.
 */
#pragma once
#include <chrono>
#include <iostream>

//时空联合规划下笛卡尔坐标系下轨迹的结构
struct TrajectoryPoint {
  double relative_time = 0.0;  //相对于起点的时间偏移量
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double theta = 0.0;
  double v = 0.0;
  double a = 0.0;
  double da = 0.0;
  double s = 0.0;  //加上这个方便判断走过的总路程
};
