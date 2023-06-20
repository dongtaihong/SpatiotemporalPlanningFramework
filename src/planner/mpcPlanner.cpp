/*
 * @Author: 董泰宏 2396203400@qq.com
 * @Date: 2023-06-06 16:45:50
 * @LastEditors: 董泰宏 2396203400@qq.com
 * @LastEditTime: 2023-06-12 23:55:13
 * @FilePath: /local_planner/src/planner/mpcPlanner.cpp
 * @Description:
 * Copyright (c) 2023 by 董泰宏 email: 2396203400@qq.com, All Rights Reserved.
 */
#include "mpcPlanner.h"

std::vector<TrajectoryPoint> mpcPlanner::plan(Simulator* simulator) {
  CostFunction(simulator);
  ConstriantFunction(simulator);
  Solution(simulator);
  return std::vector<TrajectoryPoint>();
}

void mpcPlanner::CostFunction(Simulator* simulator) {
  Eigen::SparseMatrix<double> q(input_scale_, input_scale_);
  Eigen::VectorXd p = Eigen::VectorXd::Zero(input_scale_);
  //                  1.2全局参考线项
  for (int i = 0; i < predict_step_; i++) {
    q.coeffRef(step_scale_ * i + 0, step_scale_ * i + 0) = 2;
    q.coeffRef(step_scale_ * i + 1, step_scale_ * i + 1) = 2;
    p(step_scale_ * i + 0) =
        -2 *
        simulator->rightLane_[(int)simulator->ego_.localTrajectory_[i].s + 2].x;
    p(step_scale_ * i + 1) =
        -2 *
        simulator->rightLane_[(int)simulator->ego_.localTrajectory_[i].s + 2].y;
  }
  //                  1.3控制量惩罚项
  for (int i = 0; i < predict_step_ - 1; i++) {
    // axi ayi的平方项
    q.coeffRef(step_scale_ * i + 4, step_scale_ * i + 4) = 2 * weight_R_;
    q.coeffRef(step_scale_ * i + 5, step_scale_ * i + 5) = 2 * weight_R_;
    // axi axi+1的交叉项 ayi ayi+1的交叉项
    q.coeffRef(step_scale_ * i + 4, step_scale_ * (i + 1) + 4) = -weight_R_;
    q.coeffRef(step_scale_ * (i + 1) + 4, step_scale_ * i + 4) = -weight_R_;
    q.coeffRef(step_scale_ * i + 5, step_scale_ * (i + 1) + 5) = -weight_R_;
    q.coeffRef(step_scale_ * (i + 1) + 5, step_scale_ * i + 5) = -weight_R_;
  }
  q.coeffRef(4, 4) = weight_R_;
  q.coeffRef(5, 5) = weight_R_;
  q.coeffRef(step_scale_ * (predict_step_ - 1) + 4,
             step_scale_ * (predict_step_ - 1) + 4) = weight_R_;
  q.coeffRef(step_scale_ * (predict_step_ - 1) + 5,
             step_scale_ * (predict_step_ - 1) + 5) = weight_R_;
  gradient.clear();
  gradient.emplace_back(p);
  Q.clear();
  Q.emplace_back(q);
}

void mpcPlanner::ConstriantFunction(Simulator* simulator) {
  Eigen::SparseMatrix<double> a(eq_constraint_number_ + ineq_constraint_number_,
                                input_scale_);
  Eigen::VectorXd LowerBound(eq_constraint_number_ + ineq_constraint_number_);
  LowerBound.fill(-INF);
  Eigen::VectorXd UpperBound(eq_constraint_number_ + ineq_constraint_number_);
  UpperBound.fill(INF);
  // 1.等式约束
  //            1.1 线性模型xi+1 = Axi + Bu

  // 确保每次轨迹不管迭代到多少步，都是从当前状态开始规划的
  Eigen::Matrix4d Ad;
  Ad << 1, 0, dt_, 0, 0, 1, 0, dt_, 0, 0, 1, 0, 0, 0, 0, 1;
  Eigen::Matrix<double, 4, 2> Bd;
  Bd << dt_ * dt_ / 2, 0, 0, dt_ * dt_ / 2, dt_, 0, 0, dt_;
  a.coeffRef(0, 0) = 1;
  LowerBound(0) = simulator->ego_.egoNow_.x;
  UpperBound(0) = simulator->ego_.egoNow_.x;

  a.coeffRef(1, 1) = 1;
  LowerBound(1) = simulator->ego_.egoNow_.y;
  UpperBound(1) = simulator->ego_.egoNow_.y;

  a.coeffRef(2, 2) = 1;
  LowerBound(2) =
      simulator->ego_.egoNow_.v * cos(simulator->ego_.egoNow_.theta);
  UpperBound(2) =
      simulator->ego_.egoNow_.v * cos(simulator->ego_.egoNow_.theta);
  a.coeffRef(3, 3) = 1;
  LowerBound(3) =
      simulator->ego_.egoNow_.v * sin(simulator->ego_.egoNow_.theta);
  UpperBound(3) =
      simulator->ego_.egoNow_.v * sin(simulator->ego_.egoNow_.theta);
  // 确保每次轨迹不管迭代到多少步，都是从当前状态开始规划的

  for (int i = 1; i < predict_step_; i++) {
    // 线性模型的第一行 pxi = pxi-1 + dt * vxi-1 + dt^2/2 * axi
    a.coeffRef(4 * i + 0, step_scale_ * i + 0) = 1;  // pxi
    a.coeffRef(4 * i + 0, step_scale_ * i + 4) =
        -Bd.coeff(0, 0);  //-dt^2/2 * axi
    a.coeffRef(4 * i + 0, step_scale_ * (i - 1) + 0) =
        -Ad.coeff(0, 0);  //-pxi-1
    a.coeffRef(4 * i + 0, step_scale_ * (i - 1) + 2) =
        -Ad.coeff(0, 2);  //-dt * vxi-1
    LowerBound(4 * i + 0) = 0;
    UpperBound(4 * i + 0) = 0;
    // 线性模型的第二行 pyi = pyi-1 + dt * vyi-1 + dt^2/2 * ayi
    a.coeffRef(4 * i + 1, step_scale_ * i + 1) = 1;  // pyi
    a.coeffRef(4 * i + 1, step_scale_ * i + 5) =
        -Bd.coeff(1, 1);  //-dt^2/2 * ayi
    a.coeffRef(4 * i + 1, step_scale_ * (i - 1) + 1) =
        -Ad.coeff(1, 1);  //-pyi-1
    a.coeffRef(4 * i + 1, step_scale_ * (i - 1) + 3) =
        -Ad.coeff(1, 3);  //-dt * vyi-1
    LowerBound(4 * i + 1) = 0;
    UpperBound(4 * i + 1) = 0;
    // 线性模型的第三行 vxi = vxi-1 + dt * axi
    a.coeffRef(4 * i + 2, step_scale_ * i + 2) = 1;                // vxi
    a.coeffRef(4 * i + 2, step_scale_ * i + 4) = -Bd.coeff(2, 0);  //-dt *
    // axi
    a.coeffRef(4 * i + 2, step_scale_ * (i - 1) + 2) =
        -Ad.coeff(2, 2);  //-vxi-1
    LowerBound(4 * i + 2) = 0;
    UpperBound(4 * i + 2) = 0;
    // 线性模型的第四行 vyi = vyi-1 + dt * ayi
    a.coeffRef(4 * i + 3, step_scale_ * i + 3) = 1;                // vyi
    a.coeffRef(4 * i + 3, step_scale_ * i + 5) = -Bd.coeff(3, 1);  //-dt*ayi
    a.coeffRef(4 * i + 3, step_scale_ * (i - 1) + 3) =
        -Ad.coeff(3, 3);  //-vyi-1
    LowerBound(4 * i + 3) = 0;
    UpperBound(4 * i + 3) = 0;
  }
  //            1.2 速度、加速度约束
  for (int i = 0; i < predict_step_; i++) {
    // vx
    a.coeffRef(eq_constraint_number_ + 6 * i + 0, step_scale_ * i + 2) = 1;
    if (cos(simulator
                ->referenceLine_[(int)simulator->ego_.localTrajectory_[i].s]
                .theta) >= 0) {
      LowerBound(eq_constraint_number_ + 6 * i + 0) = 0;
      UpperBound(eq_constraint_number_ + 6 * i + 0) =
          simulator->ego_.vTarget_ *
          cos(simulator
                  ->referenceLine_[(int)simulator->ego_.localTrajectory_[i].s]
                  .theta);
    } else {
      LowerBound(eq_constraint_number_ + 6 * i + 0) =
          simulator->ego_.vTarget_ *
          cos(simulator
                  ->referenceLine_[(int)simulator->ego_.localTrajectory_[i].s]
                  .theta);
      UpperBound(eq_constraint_number_ + 6 * i + 0) = 0;
    }
    //  vy
    a.coeffRef(eq_constraint_number_ + 6 * i + 1, step_scale_ * i + 3) = 1;
    if (sin(simulator
                ->referenceLine_[(int)simulator->ego_.localTrajectory_[i].s]
                .theta) >= 0) {
      // 保证在平直道路时车辆依然能够具备充足的侧向速度，维持换道的能力
      if (sin(simulator
                  ->referenceLine_[(int)simulator->ego_.localTrajectory_[i].s]
                  .theta) < 0.4) {
        LowerBound(eq_constraint_number_ + 6 * i + 1) =
            -0.5 * simulator->ego_.vTarget_;
        UpperBound(eq_constraint_number_ + 6 * i + 1) =
            0.5 * simulator->ego_.vTarget_;
      } else {
        LowerBound(eq_constraint_number_ + 6 * i + 1) = 0;
        UpperBound(eq_constraint_number_ + 6 * i + 1) =
            simulator->ego_.vTarget_ *
            sin(simulator
                    ->referenceLine_[(int)simulator->ego_.localTrajectory_[i].s]
                    .theta);
      }
    } else {
      if (sin(simulator
                  ->referenceLine_[(int)simulator->ego_.localTrajectory_[i].s]
                  .theta) > -0.4) {
        LowerBound(eq_constraint_number_ + 6 * i + 1) =
            -0.5 * simulator->ego_.vTarget_;
        UpperBound(eq_constraint_number_ + 6 * i + 1) =
            0.5 * simulator->ego_.vTarget_;
      } else {
        LowerBound(eq_constraint_number_ + 6 * i + 1) =
            simulator->ego_.vTarget_ *
            sin(simulator
                    ->referenceLine_[(int)simulator->ego_.localTrajectory_[i].s]
                    .theta);
        UpperBound(eq_constraint_number_ + 6 * i + 1) = 0;
      }
    }
    //  ax
    a.coeffRef(eq_constraint_number_ + 6 * i + 2, step_scale_ * i + 4) = 1;
    LowerBound(eq_constraint_number_ + 6 * i + 2) = -8;
    UpperBound(eq_constraint_number_ + 6 * i + 2) = 8;
    // ay
    a.coeffRef(eq_constraint_number_ + 6 * i + 3, step_scale_ * i + 5) = 1;
    LowerBound(eq_constraint_number_ + 6 * i + 3) = -8;
    UpperBound(eq_constraint_number_ + 6 * i + 3) = 8;

    //            1.3 边界约束
    // 左边界
    a.coeffRef(eq_constraint_number_ + 6 * i + 4, step_scale_ * i + 0) = -sin(
        simulator->referenceLine_[(int)simulator->ego_.localTrajectory_[i].s]
            .theta);  // xi * -sin(road_index)
    a.coeffRef(eq_constraint_number_ + 6 * i + 4,
               step_scale_ * i + 1) =
        cos(simulator
                ->referenceLine_[(int)simulator->ego_.localTrajectory_[i].s]
                .theta);  // yi * cos(road_index)
    LowerBound(eq_constraint_number_ + 6 * i + 4) = -1 * INF;
    UpperBound(eq_constraint_number_ + 6 * i + 4) =
        -sin(simulator
                 ->referenceLine_[(int)simulator->ego_.localTrajectory_[i].s]
                 .theta) *
            simulator->leftLane_[(int)simulator->ego_.localTrajectory_[i].s].x +
        cos(simulator
                ->referenceLine_[(int)simulator->ego_.localTrajectory_[i].s]
                .theta) *
            simulator->leftLane_[(int)simulator->ego_.localTrajectory_[i].s].y;
    // 右边界
    a.coeffRef(eq_constraint_number_ + 6 * i + 5,
               step_scale_ * i + 0) =
        sin(simulator
                ->referenceLine_[(int)simulator->ego_.localTrajectory_[i].s]
                .theta);  // xi * sin(road_index)
    a.coeffRef(eq_constraint_number_ + 6 * i + 5,
               step_scale_ * i + 1) =
        -cos(simulator
                 ->referenceLine_[(int)simulator->ego_.localTrajectory_[i].s]
                 .theta);  // yi * -cos(road_index)
    LowerBound(eq_constraint_number_ + 6 * i + 5) = -1 * INF;
    UpperBound(eq_constraint_number_ + 6 * i + 5) =
        sin(simulator
                ->referenceLine_[(int)simulator->ego_.localTrajectory_[i].s]
                .theta) *
            simulator->rightLane_[(int)simulator->ego_.localTrajectory_[i].s]
                .x -
        cos(simulator
                ->referenceLine_[(int)simulator->ego_.localTrajectory_[i].s]
                .theta) *
            simulator->rightLane_[(int)simulator->ego_.localTrajectory_[i].s].y;
  }
  //            1.4 障碍物约束(直接引入到边界约束进行处理)
  for (auto obs : simulator->obstacles_) {
    // 如果障碍物在左边, 则影响的是左边界; 如果障碍物在右边, 则影响的是右边界
    for (int i = 0; i < predict_step_; i++) {
      double dist_now = sqrt(pow((simulator->ego_.localTrajectory_[i].x -
                                  obs.predictTrajectory_[i].x),
                                 2) +
                             pow((simulator->ego_.localTrajectory_[i].y -
                                  obs.predictTrajectory_[i].y),
                                 2));
      // 距离障碍物的最小距离为8m，实际上减去车身，仅余4m
      if (dist_now < 6) {
        int temp_step = 4;
        if (i < 2 * temp_step) {
          for (int j = temp_step; j <= i + 2; j++) {
            if (obs.lane_ == 0) {
              UpperBound(eq_constraint_number_ + 6 * j + 4) =
                  -sin(simulator
                           ->referenceLine_
                               [(int)simulator->ego_.localTrajectory_[j].s]
                           .theta) *
                      simulator
                          ->rightLane_[(int)simulator->ego_.localTrajectory_[j]
                                           .s]
                          .x +
                  cos(simulator
                          ->referenceLine_
                              [(int)simulator->ego_.localTrajectory_[j].s]
                          .theta) *
                      simulator
                          ->rightLane_[(int)simulator->ego_.localTrajectory_[j]
                                           .s]
                          .y;
            }
            if (obs.lane_ == 1) {
              UpperBound(eq_constraint_number_ + 6 * j + 5) =
                  sin(simulator
                          ->referenceLine_
                              [(int)simulator->ego_.localTrajectory_[j].s]
                          .theta) *
                      simulator
                          ->leftLane_[(int)simulator->ego_.localTrajectory_[j]
                                          .s]
                          .x -
                  cos(simulator
                          ->referenceLine_
                              [(int)simulator->ego_.localTrajectory_[j].s]
                          .theta) *
                      simulator
                          ->leftLane_[(int)simulator->ego_.localTrajectory_[j]
                                          .s]
                          .y;
              // std::cout << "up:"
              //           << UpperBound(eq_constraint_number_ + 6 * j + 4) <<
              //           ","
              //           << -UpperBound(eq_constraint_number_ + 6 * j + 5)
              //           << std::endl;
            }
          }
        } else if (i > 47) {
          for (int j = i - 2; j <= predict_step_ - 1; j++) {
            if (obs.lane_ == 0) {
              UpperBound(eq_constraint_number_ + 6 * j + 4) =
                  -sin(simulator
                           ->referenceLine_
                               [(int)simulator->ego_.localTrajectory_[j].s]
                           .theta) *
                      simulator
                          ->rightLane_[(int)simulator->ego_.localTrajectory_[j]
                                           .s]
                          .x +
                  cos(simulator
                          ->referenceLine_
                              [(int)simulator->ego_.localTrajectory_[j].s]
                          .theta) *
                      simulator
                          ->rightLane_[(int)simulator->ego_.localTrajectory_[j]
                                           .s]
                          .y;
            }
            if (obs.lane_ == 1) {
              UpperBound(eq_constraint_number_ + 6 * j + 5) =
                  sin(simulator
                          ->referenceLine_
                              [(int)simulator->ego_.localTrajectory_[j].s]
                          .theta) *
                      simulator
                          ->leftLane_[(int)simulator->ego_.localTrajectory_[j]
                                          .s]
                          .x -
                  cos(simulator
                          ->referenceLine_
                              [(int)simulator->ego_.localTrajectory_[j].s]
                          .theta) *
                      simulator
                          ->leftLane_[(int)simulator->ego_.localTrajectory_[j]
                                          .s]
                          .y;
              // std::cout << "up:"
              //           << UpperBound(eq_constraint_number_ + 6 * j + 4) <<
              //           ","
              //           << -UpperBound(eq_constraint_number_ + 6 * j + 5)
              //           << std::endl;
            }
          }
        } else {
          for (int j = i - 2; j <= i + 2; j++) {
            if (obs.lane_ == 0) {
              UpperBound(eq_constraint_number_ + 6 * j + 4) =
                  -sin(simulator
                           ->referenceLine_
                               [(int)simulator->ego_.localTrajectory_[j].s]
                           .theta) *
                      simulator
                          ->rightLane_[(int)simulator->ego_.localTrajectory_[j]
                                           .s]
                          .x +
                  cos(simulator
                          ->referenceLine_
                              [(int)simulator->ego_.localTrajectory_[j].s]
                          .theta) *
                      simulator
                          ->rightLane_[(int)simulator->ego_.localTrajectory_[j]
                                           .s]
                          .y;
            }
            if (obs.lane_ == 1) {
              UpperBound(eq_constraint_number_ + 6 * j + 5) =
                  sin(simulator
                          ->referenceLine_
                              [(int)simulator->ego_.localTrajectory_[j].s]
                          .theta) *
                      simulator
                          ->leftLane_[(int)simulator->ego_.localTrajectory_[j]
                                          .s]
                          .x -
                  cos(simulator
                          ->referenceLine_
                              [(int)simulator->ego_.localTrajectory_[j].s]
                          .theta) *
                      simulator
                          ->leftLane_[(int)simulator->ego_.localTrajectory_[j]
                                          .s]
                          .y;
              // std::cout << "up:"
              //           << UpperBound(eq_constraint_number_ + 6 * j + 4) <<
              //           ","
              //           << -UpperBound(eq_constraint_number_ + 6 * j + 5)
              //           << std::endl;
            }
          }
        }
      }
    }
  }
  //            1.5 终端速度约束:
  //保证轨迹的可控性，在求解失败时保障车辆的安全
  a.coeffRef(eq_constraint_number_ + 6 * predict_step_ + 0,
             step_scale_ * 49 + 2) = 1;
  LowerBound(eq_constraint_number_ + 6 * predict_step_ + 0) = 0;
  UpperBound(eq_constraint_number_ + 6 * predict_step_ + 0) = 0;
  a.coeffRef(eq_constraint_number_ + 6 * predict_step_ + 1,
             step_scale_ * 49 + 3) = 1;
  LowerBound(eq_constraint_number_ + 6 * predict_step_ + 1) = 0;
  UpperBound(eq_constraint_number_ + 6 * predict_step_ + 1) = 0;
  A.clear();
  bound.clear();
  A.emplace_back(a);
  bound.emplace_back(LowerBound);
  bound.emplace_back(UpperBound);
}

bool mpcPlanner::Solution(Simulator* simulator) {
  OsqpEigen::Solver solver;
  solver.settings()->setVerbosity(false);
  solver.settings()->setWarmStart(false);
  Eigen::Matrix<double, 301, 1> mat = Eigen::Matrix<double, 301, 1>::Zero();

  //超车积极性稍高
  for (int i = 0; i < 301; i++) {
    mat(i, 0) = warm_start_[i];
  }

  solver.data()->setNumberOfVariables(input_scale_);
  solver.data()->setNumberOfConstraints(eq_constraint_number_ +
                                        ineq_constraint_number_);

  if (!solver.data()->setHessianMatrix(Q[0])) return false;
  if (!solver.data()->setGradient(gradient[0])) return false;
  if (!solver.data()->setLinearConstraintsMatrix(A[0])) return false;
  if (!solver.data()->setLowerBound(bound[0])) return false;
  if (!solver.data()->setUpperBound(bound[1])) return false;
  Eigen::VectorXd v =
      Eigen::Map<Eigen::VectorXd>(warm_start_.data(), warm_start_.size());
  if (!solver.initSolver()) return false;
  if (!solver.setPrimalVariable(mat)) return false;
  Eigen::VectorXd Solution;
  if (!solver.solve()) {
    std::vector<TrajectoryPoint> temp;
    temp.assign(simulator->ego_.localTrajectory_.begin() + 1,
                simulator->ego_.localTrajectory_.end());
    simulator->ego_.localTrajectory_.assign(temp.begin(), temp.end());
    simulator->ego_.localTrajectory_.push_back(temp.back());
    return false;
  }
  cout << "problem solved" << endl;
  Solution = solver.getSolution();
  ResultToTrajectory(Solution, simulator);
  return true;
}

void mpcPlanner::ResultToTrajectory(Eigen::VectorXd& solution,
                                    Simulator* simulator) {
  // warm-start和local-trajectory
  warm_start_.clear();
  simulator->ego_.localTrajectory_.clear();
  for (int i = 0; i < input_scale_; i++) {
    warm_start_.push_back(solution(i));
  }
  simulator->ego_.localTrajectory_.emplace_back(simulator->ego_.egoNow_);
  if (simulator->ego_.egoNow_.s > simulator->ego_.globalIndex_) {
    simulator->ego_.localTrajectory_.back().s = simulator->ego_.egoNow_.s;
  } else {
    simulator->ego_.localTrajectory_.back().s = simulator->ego_.globalIndex_;
  }
  for (int i = 0; i < predict_step_; i++) {
    TrajectoryPoint temp_point;
    temp_point.relative_time =
        simulator->ego_.localTrajectory_.back().relative_time + 0.1;
    temp_point.x = solution(step_scale_ * i + 0);
    temp_point.y = solution(step_scale_ * i + 1);
    temp_point.theta =
        atan2(solution(step_scale_ * i + 3), solution(step_scale_ * i + 2));
    temp_point.v = sqrt(pow(solution(step_scale_ * i + 3), 2) +
                        pow(solution(step_scale_ * i + 2), 2));
    temp_point.a = sqrt(pow(solution(step_scale_ * i + 4), 2) +
                        pow(solution(step_scale_ * i + 5), 2));
    temp_point.da =
        (temp_point.a - simulator->ego_.localTrajectory_.back().a) * 10;
    temp_point.s =
        simulator->ego_.localTrajectory_.back().s +
        sqrt(pow(temp_point.x - simulator->ego_.localTrajectory_.back().x, 2) +
             pow(temp_point.y - simulator->ego_.localTrajectory_.back().y, 2));
    if (i == 0) simulator->ego_.localTrajectory_.clear();
    simulator->ego_.localTrajectory_.emplace_back(temp_point);
  }
}
