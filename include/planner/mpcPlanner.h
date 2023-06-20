/*
 * @Author: 董泰宏 2396203400@qq.com
 * @Date: 2023-05-29 17:50:17
 * @LastEditors: 董泰宏 2396203400@qq.com
 * @LastEditTime: 2023-06-20 22:09:48
 * @FilePath: /local_planner/include/planner/mpcPlanner.h
 * @Description:
 * Copyright (c) 2023 by 董泰宏 email: 2396203400@qq.com, All Rights Reserved.
 */
#include <bits/stdc++.h>

#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <vector>

#include "OsqpEigen/OsqpEigen.h"
#include "PlannerFactory.h"
#include "SimulatorFactory.h"

using namespace std;

#define INF (INT_MAX % 10)

//传入simulator的场景信息，输出主车的局部轨迹
class mpcPlanner : public Planner {
 public:
  mpcPlanner() {
    for (int i = 0; i < 301; i++) {
      warm_start_.push_back(0.0);
    }
  };
  std::vector<TrajectoryPoint> plan(Simulator* simulator) override;
  ~mpcPlanner() = default;

 private:
  // step2: 构建目标函数的Q矩阵
  void CostFunction(Simulator* simulator);
  // step3: 构建不等式约束的A矩阵以及约束边界
  void ConstriantFunction(Simulator* simulator);
  // step4: 求解问题
  bool Solution(Simulator* simulator);
  // step5: 从优化结果放入LocalTrajectory_
  void ResultToTrajectory(Eigen::VectorXd& solution, Simulator* simulator);
  int predict_step_ = 50;  // mpc预测长度, 设置为50步
  int step_scale_ = 6;     // 每一步的规模：x, y, vx, vy, ax, ay， 6个
  int input_scale_ = predict_step_ * step_scale_ + 1;  // 优化变量的总规模
  int eq_constraint_number_ =
      predict_step_ * 4;  // 等式约束的规模: 运动学模型约束(4
                          //*predict_step_)

  int ineq_constraint_number_ =
      (2 + 2 + 2) * predict_step_ + 2;  // 不等式约束的规模:速度约束 +
                                        // 加速度约束 +
  // 边界约束(包含了障碍物约束) +
  //   终端速度为0

  double weight_refline_ = 10;  // 参考线惩罚权重
  double weight_R_ = 100;       // 控制量变化惩罚权重
  //设为10的整体效果也不错，在减速后也会想办法超车
  double dt_ = 0.1;  // 两点间的时间间隔为0.1s

  std::vector<double>
      warm_start_;  // 上一帧规划结果[x0, y0, vx0, vy0, ax0, ay0,...]
  std::vector<Eigen::SparseMatrix<double>> Q;  // 目标矩阵
  std::vector<Eigen::VectorXd> Derivative;
  std::vector<Eigen::VectorXd> bound;
  std::vector<Eigen::VectorXd> gradient;
  std::vector<Eigen::SparseMatrix<double>> A;  // 约束矩阵
};

class mpcPlannerFactory : public PlannerFactory {
 public:
  Planner* selectPlanner() override { return new mpcPlanner(); };
  ~mpcPlannerFactory() = default;
};