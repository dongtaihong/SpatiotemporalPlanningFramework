#include "randomSimulatorFactory.h"

// 将车辆的四个顶点围绕中心点旋转theta度
void rotate(vector<double>& ego_x, vector<double>& ego_y, double x, double y,
            double x_center, double y_center, double theta) {
  double rad = theta;  // 弧度
  double d = 2.236;    // 计算点之间的距离
  double alpha =
      std::atan2(y - y_center, x - x_center);  // 计算点 a 相对于点 b 的方位角
  alpha += rad;                                // 增加角度
  x = x_center + d * std::cos(alpha);  // 更新点 a 的 x 坐标
  y = y_center + d * std::sin(alpha);  // 更新点 a 的 y 坐标
  ego_x.push_back(x);
  ego_y.push_back(y);
}

//仿真器构造时直接将全局路径等固定量读取，不同的仿真器此处都可以通过读文件的方式来操作，文件路径提前都设定好
randomSimulator::randomSimulator() {
  std::ifstream inFile("/home/dth/local_planner/map/randomSimulator/map.csv",
                       std::ios::in);
  std::string line, position;
  while (getline(inFile, line)) {
    std::istringstream sin(line);
    TrajectoryPoint temp_left_road;
    TrajectoryPoint temp_right_road;
    TrajectoryPoint temp_center_road;
    TrajectoryPoint temp_left_lane;
    TrajectoryPoint temp_right_lane;
    // 车道左边界点
    getline(sin, position, ',');
    temp_left_road.x = atof(position.c_str());
    getline(sin, position, ',');
    temp_left_road.y = atof(position.c_str());
    // 车道右边界点
    getline(sin, position, ',');
    temp_right_road.x = atof(position.c_str());
    getline(sin, position, ',');
    temp_right_road.y = atof(position.c_str());
    // 两车道间的车道线点
    getline(sin, position, ',');
    temp_center_road.x = atof(position.c_str());
    getline(sin, position, ',');
    temp_center_road.y = atof(position.c_str());
    // 左侧车道中心线点
    getline(sin, position, ',');
    temp_left_lane.x = atof(position.c_str());
    getline(sin, position, ',');
    temp_left_lane.y = atof(position.c_str());
    // 右侧车道中心线点
    getline(sin, position, ',');
    temp_right_lane.x = atof(position.c_str());
    getline(sin, position, ',');
    temp_right_lane.y = atof(position.c_str());
    // 每一组点对应的theta
    getline(sin, position, ',');
    temp_left_road.theta = atof(position.c_str());
    temp_right_road.theta = atof(position.c_str());
    temp_center_road.theta = atof(position.c_str());
    temp_left_lane.theta = atof(position.c_str());
    temp_right_lane.theta = atof(position.c_str());

    //每个点的s
    if (referenceLine_.size() != 0) {
      temp_center_road.s =
          centerRoad_.back().s +
          std::sqrt(std::pow(temp_center_road.x - centerRoad_.back().x, 2) +
                    std::pow(temp_center_road.y - centerRoad_.back().y, 2));
      temp_right_lane.s = temp_center_road.s;
      temp_left_lane.s = temp_center_road.s;
      temp_left_road.s = temp_center_road.s;
      temp_right_road.s = temp_center_road.s;
    }
    //全局参考线设置为右车道中心线
    referenceLine_.emplace_back(temp_right_lane);
    leftRoad_.emplace_back(temp_left_road);
    rightRoad_.emplace_back(temp_right_road);
    centerRoad_.emplace_back(temp_center_road);
    leftLane_.emplace_back(temp_left_lane);
    rightLane_.emplace_back(temp_right_lane);
    for (int i = 0; i < referenceLine_.size() - 1; i++) {
      referenceLine_[i].theta =
          atan2(referenceLine_[i + 1].y - referenceLine_[i].y,
                referenceLine_[i + 1].x - referenceLine_[i].x);
      centerRoad_[i].theta = referenceLine_[i].theta;
      if (i == referenceLine_.size() - 2) {
        referenceLine_.back().theta = referenceLine_[i].theta;
        centerRoad_.back().theta = referenceLine_.back().theta;
      }
    }
  }
  // 车道回环处不要太跳跃
  referenceLine_.back().y = -3;
  leftRoad_.back().y = 3;
  rightRoad_.back().y = -3;
  centerRoad_.back().y = 0;
  leftLane_.back().y = 1.5;
  rightLane_.back().y = -1.5;
  inFile.close();
  startTime_ = std::chrono::system_clock::now();

  //初始化ego的创建信息
  for (int i = 0; i < 60; i++) {
    ego_.localTrajectory_.emplace_back(ego_.egoNow_);
  }
}

//如果所有障碍物当前的s都小于了主车的s，则当前场景结束
bool randomSimulator::isOverScenario() {
  std::chrono::system_clock::time_point temp_obs_now =
      std::chrono::system_clock::now();
  std::chrono::milliseconds used =
      std::chrono::duration_cast<std::chrono::milliseconds>(temp_obs_now -
                                                            startTime_);
  double temp_obs_relative_time = used.count() / 1000.0;
  for (auto obs : obstacles_) {
    int a = (temp_obs_relative_time - obs.predictTrajectory_[0].relative_time) /
            0.1;

    if (obs.obsNow_.s + 18 > ego_.globalIndex_) return false;
  }
  for (auto obs : obstacles_) {
    std::cout << obs.obsNow_.s << ", " << ego_.globalIndex_ << std::endl;
  }
  return true;
}

Obstacle randomSimulator::generateRandomObstacle() {
  Obstacle obs;
  TrajectoryPoint temp_obs;
  //障碍物的时间
  std::chrono::system_clock::time_point temp_obs_now =
      std::chrono::system_clock::now();
  std::chrono::milliseconds used =
      std::chrono::duration_cast<std::chrono::milliseconds>(temp_obs_now -
                                                            startTime_);
  temp_obs.relative_time = used.count() / 1000.0;

  //障碍物的速度
  std::random_device rd;
  std::mt19937 eng(rd());
  std::uniform_real_distribution<double> dist_v(0.3 * ego_.vTarget_,
                                                0.6 * ego_.vTarget_);
  temp_obs.v = dist_v(eng);
  double interVal = 0.1 * temp_obs.v;  // 0.5;  // temp_obs.v * 0.1;

  //障碍物的位置————随机生成在全局路径索引的后60～120位
  std::uniform_int_distribution<int> dist_pos(ego_.globalIndex_ + 20,
                                              ego_.globalIndex_ + 60);
  int temp_obs_index = dist_pos(eng);
  std::uniform_int_distribution<int> dist_lane(0, 1);
  int temp_obs_lane = dist_lane(eng);
  if (temp_obs_lane == 0) {
    temp_obs.x = leftLane_[temp_obs_index].x;
    temp_obs.y = leftLane_[temp_obs_index].y;
    temp_obs.theta = centerRoad_[temp_obs_index].theta;
    temp_obs.s = referenceLine_[temp_obs_index].s;
    obs.predictTrajectory_.emplace_back(temp_obs);
    obs.lane_ = 0;
    obs.index_ = temp_obs_index;

    //未来的60s之间的轨迹————障碍物能够存活60s，轨迹的粒度是足够的，与road_index无关，仅与时间相关
    double temp_interVal = 0.0;
    for (int i = 0; i < 600; i++) {
      temp_interVal += interVal;
      while (temp_interVal >= 1) {
        temp_obs_index++;
        temp_interVal -= 1;
      }
      temp_obs.relative_time += 0.1;  //更新预测time
      temp_obs.x = (1 - temp_interVal) * leftLane_[temp_obs_index].x +
                   temp_interVal * leftLane_[temp_obs_index + 1].x;  //更新预测x
      temp_obs.y = (1 - temp_interVal) * leftLane_[temp_obs_index].y +
                   temp_interVal * leftLane_[temp_obs_index + 1].y;  //更新预测y
      temp_obs.theta =
          (1 - temp_interVal) * leftLane_[temp_obs_index].theta +
          temp_interVal * leftLane_[temp_obs_index + 1].theta;  //更新预测theta
      // temp_obs.s += interVal;                                   //更新预测s
      temp_obs.s = temp_obs_index + interVal;
      obs.predictTrajectory_.emplace_back(temp_obs);
    }
  } else if (temp_obs_lane == 1) {
    temp_obs.x = rightLane_[temp_obs_index].x;
    temp_obs.y = rightLane_[temp_obs_index].y;
    temp_obs.theta = centerRoad_[temp_obs_index].theta;
    obs.predictTrajectory_.emplace_back(temp_obs);
    obs.lane_ = 1;
    obs.index_ = temp_obs_index;

    //未来的60s之间的轨迹————障碍物能够存活60s
    double temp_interVal = 0.0;
    for (int i = 0; i < 600; i++) {
      temp_interVal += interVal;
      while (temp_interVal >= 1) {
        temp_obs_index++;
        temp_interVal -= 1;
      }
      temp_obs.relative_time += 0.1;  //更新预测time
      temp_obs.x =
          (1 - temp_interVal) * rightLane_[temp_obs_index].x +
          temp_interVal * rightLane_[temp_obs_index + 1].x;  //更新预测x
      temp_obs.y =
          (1 - temp_interVal) * rightLane_[temp_obs_index].y +
          temp_interVal * rightLane_[temp_obs_index + 1].y;  //更新预测y
      temp_obs.theta =
          (1 - temp_interVal) * rightLane_[temp_obs_index].theta +
          temp_interVal * rightLane_[temp_obs_index + 1].theta;  //更新预测theta
      temp_obs.s = temp_obs_index + interVal;
      obs.predictTrajectory_.emplace_back(temp_obs);
    }
  }
  return obs;
}

//生成下一个场景
void randomSimulator::generateNextScenario() {
  obstacle_60s_.clear();
  std::random_device rd;
  std::mt19937 eng(rd());
  std::uniform_int_distribution<int> dist_num(1, 4);
  int obstacle_num = dist_num(eng);
  std::chrono::system_clock::time_point tbegin =
      std::chrono::system_clock::now();
  std::chrono::system_clock::time_point tend;
  std::chrono::milliseconds used;
  while (obstacle_60s_.size() < obstacle_num) {
    // 障碍物生成时间不能超过10ms
    tend = std::chrono::system_clock::now();
    used = std::chrono::duration_cast<std::chrono::milliseconds>(tend - tbegin);
    if (used.count() > 10) break;

    if (obstacle_60s_.size() == 0) {
      obstacle_60s_.push_back(generateRandomObstacle());
      continue;
    }
    Obstacle temp_obs;
    temp_obs = std::move(generateRandomObstacle());

    bool flag = true;
    for (auto obs : obstacle_60s_) {
      // 如果生成的障碍物在已有障碍物的前方，并且速度低了很多，则障碍物之间会碰撞，舍弃该障碍物
      if (temp_obs.lane_ == obs.lane_ && temp_obs.index_ > obs.index_ &&
          temp_obs.predictTrajectory_[0].v <
              0.8 * obs.predictTrajectory_[0].v) {
        flag = false;
        break;
      }
      // 障碍物出现在了前后8m内
      if (temp_obs.index_ <= obs.index_ + 10 &&
          temp_obs.index_ >= obs.index_ - 10) {
        flag = false;
        break;
      }
    }
    // 障碍物合理则放入场景中
    if (flag) obstacle_60s_.push_back(temp_obs);
  }
}

void randomSimulator::refreshScenario() {
  // 1.更新当前主车的状态———当前时刻主车真实的位置信息=上一帧规划结果的第二个点（第一个点是上一时刻的定位点）
  //                     因为此仿真器没有控制仿真能力，所以假设控制是理想的，如果有控制，则此处应以实际为准
  //                     此处在carla中应该是通过订阅话题得到
  ego_.egoNow_ = ego_.localTrajectory_[1];
  ego_.globalIndex_ = findIndex();
  std::cout << ego_.egoNow_.v << std::endl;

  // 2.更新当前的场景
  if (isOverScenario()) {
    generateNextScenario();
  }

  // 3.更新当前所有障碍物的未来6s轨迹——理论上不用质疑预测轨迹与定位的准确性，因为是按照0.1s的时间来更新的，没有误差。
  obstacles_.clear();
  for (auto obs : obstacle_60s_) {
    Obstacle current_obs;
    current_obs.lane_ = obs.lane_;
    //障碍物的时间
    std::chrono::system_clock::time_point obs_now =
        std::chrono::system_clock::now();
    std::chrono::milliseconds used =
        std::chrono::duration_cast<std::chrono::milliseconds>(obs_now -
                                                              startTime_);
    double relative_time = used.count() / 1000.0;
    int flag_index =
        floor((relative_time - obs.predictTrajectory_[0].relative_time) / 0.1);
    //当前障碍物的定位：此处在carla中应该是通过订阅话题得到
    current_obs.obsNow_ = obs.predictTrajectory_[flag_index];
    current_obs.index_ =
        (int)obs.predictTrajectory_[flag_index]
            .s;  // TODO:障碍物当前6s轨迹的起点index应该严格通过搜索得到

    for (int i = 0; i < 60; i++) {
      current_obs.predictTrajectory_.emplace_back(
          obs.predictTrajectory_[flag_index + i]);
    }
    obstacles_.emplace_back(current_obs);
  }
}

//参考主车的s寻找在参考线上的匹配点
int randomSimulator::findIndex() {
  int index_now = (int)ego_.egoNow_.s;
  int flag = 0;
  double flag_distance = 10000.0;
  if (index_now <= 20) index_now = 20;
  for (int i = index_now - 20; i < index_now + 50; i++) {
    double distance_now = sqrt(pow(ego_.egoNow_.x - rightLane_[i].x, 2) +
                               pow(ego_.egoNow_.y - rightLane_[i].y, 2));
    if (distance_now < flag_distance) {
      flag = i;
      flag_distance = distance_now;
    }
  }
  return flag;
}

void randomSimulator::showScenario() {
  // 打印车道
  plt::clf();
  std::vector<double> center_road_x;
  std::vector<double> center_road_y;
  std::vector<double> left_road_x;
  std::vector<double> left_road_y;
  std::vector<double> right_road_x;
  std::vector<double> right_road_y;
  for (int i = 0; i < centerRoad_.size(); i++) {
    center_road_x.emplace_back(centerRoad_[i].x);
    center_road_y.emplace_back(centerRoad_[i].y);
    left_road_x.emplace_back(leftRoad_[i].x);
    left_road_y.emplace_back(leftRoad_[i].y);
    right_road_x.emplace_back(rightRoad_[i].x);
    right_road_y.emplace_back(rightRoad_[i].y);
  }
  plt::plot(center_road_x, center_road_y, "k--");
  plt::plot(left_road_x, left_road_y, "k");
  plt::plot(right_road_x, right_road_y, "k");

  // 打印当前的场景
  vector<double> ego_x;
  vector<double> ego_y;
  rotate(ego_x, ego_y, ego_.egoNow_.x - 2, ego_.egoNow_.y - 1, ego_.egoNow_.x,
         ego_.egoNow_.y, ego_.egoNow_.theta);
  rotate(ego_x, ego_y, ego_.egoNow_.x - 2, ego_.egoNow_.y + 1, ego_.egoNow_.x,
         ego_.egoNow_.y, ego_.egoNow_.theta);
  rotate(ego_x, ego_y, ego_.egoNow_.x + 2, ego_.egoNow_.y + 1, ego_.egoNow_.x,
         ego_.egoNow_.y, ego_.egoNow_.theta);
  rotate(ego_x, ego_y, ego_.egoNow_.x + 2, ego_.egoNow_.y - 1, ego_.egoNow_.x,
         ego_.egoNow_.y, ego_.egoNow_.theta);

  map<string, string> keywords_ego;
  keywords_ego.insert(pair<string, string>("color", "blue"));
  plt::fill(ego_x, ego_y, keywords_ego);
  vector<double> ego_traj_x;
  vector<double> ego_traj_y;

  for (int i = 0; i < ego_.localTrajectory_.size(); i++) {
    ego_traj_x.push_back(ego_.localTrajectory_[i].x);
    ego_traj_y.push_back(ego_.localTrajectory_[i].y);
  }
  plt::named_plot("mpc_trajectory", ego_traj_x, ego_traj_y, "r");

  for (auto obs : obstacles_) {
    vector<double> x;
    vector<double> y;
    rotate(x, y, obs.obsNow_.x - 2, obs.obsNow_.y - 1, obs.obsNow_.x,
           obs.obsNow_.y, obs.obsNow_.theta);
    rotate(x, y, obs.obsNow_.x - 2, obs.obsNow_.y + 1, obs.obsNow_.x,
           obs.obsNow_.y, obs.obsNow_.theta);
    rotate(x, y, obs.obsNow_.x + 2, obs.obsNow_.y + 1, obs.obsNow_.x,
           obs.obsNow_.y, obs.obsNow_.theta);
    rotate(x, y, obs.obsNow_.x + 2, obs.obsNow_.y - 1, obs.obsNow_.x,
           obs.obsNow_.y, obs.obsNow_.theta);
    map<string, string> keywords;
    keywords.insert(pair<string, string>("color", "black"));
    plt::fill(x, y, keywords);

    vector<double> x1;
    vector<double> y1;
    x1.push_back(obs.index_ - 1);
    x1.push_back(obs.index_ - 1);
    x1.push_back(obs.index_ + 1);
    x1.push_back(obs.index_ + 1);
    y1.push_back(0 - 1);
    y1.push_back(0 + 1);
    y1.push_back(0 + 1);
    y1.push_back(0 - 1);
    map<string, string> keywords1;
    keywords1.insert(pair<string, string>("color", "green"));
    plt::fill(x1, y1, keywords1);
  }

  // 找到曲线的中心点
  plt::xlim(center_road_x[ego_.globalIndex_] - 60,
            center_road_x[ego_.globalIndex_] + 60);
  plt::ylim(right_road_y[ego_.globalIndex_] - 56,
            left_road_y[ego_.globalIndex_ + 50] + 56);
  plt::title("local planner");
  map<string, string> keywords_label;
  keywords_label.insert(pair<string, string>("loc", "upper left"));
  plt::legend(keywords_label);
  plt::pause(0.001);
}

void randomSimulator::check10hz(std::chrono::steady_clock::time_point tbegin) {
  std::chrono::steady_clock::time_point tend = chrono::steady_clock::now();
  std::chrono::milliseconds used =
      chrono::duration_cast<chrono::milliseconds>(tend - tbegin);
  if (used.count() > 100) return;
  std::this_thread::sleep_for(std::chrono::milliseconds(100 - used.count()));
}