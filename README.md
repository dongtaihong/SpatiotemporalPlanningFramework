# 时空联合数值优化的局部轨迹规划框架
1.框架：工厂模式+策略模式

2.仿真器接口：
   全局参考线与道路: 每个路径点包含了x, y, theta
```shell
  std::vector<TrajectoryPoint> referenceLine_;  //主车参考线
  std::vector<TrajectoryPoint> leftRoad_;       //道路左边界
  std::vector<TrajectoryPoint> rightRoad_;      //道路右边界
  std::vector<TrajectoryPoint> centerRoad_;     //道路中心线
  std::vector<TrajectoryPoint> leftLane_;       //道路左车道中心
  std::vector<TrajectoryPoint> rightLane_;      //道路右车道中心
```
   主车定位与运动信息: 包含了当前的定位点的信息(trajectorypoint), 当前局部轨迹, 在全局参考线的投影点, 期望速度
```shell
struct Ego {
  TrajectoryPoint egoNow_;                        //当前的定位
  std::vector<TrajectoryPoint> localTrajectory_;  //局部轨迹
  int globalIndex_;  //当前ego在全局参考线上的投影参考点index(需要实时的去匹配)，这里记录一下index是为了方便缩小匹配的范围
  double vTarget_ = 10.0;  //期望速度
};
```
   障碍物定位与运动信息: 包含了当前的定位点的信息(trajectorypoint), 当前预测轨迹, 所在车道, 在全局参考线的投影点
```shell
struct Obstacle {
  TrajectoryPoint obsNow_;
  std::vector<TrajectoryPoint> predictTrajectory_;  //从当前时刻起，未来的6s轨迹
  int lane_;  // 障碍物的lane, 0左1右
  // 现在相当于这个框架只适合2车道的
  int index_;  //当前障碍物出生时在全局参考线上的投影参考点index,作用是为了获取出生点的轨迹点
};
```
3.说明：
     3.1仿真器框架怎么使用?
   仿真器一共需要提供三个接口, 在此框架中, 所有的数据结构都制定完成, 切换不同的仿真器时, 仅需要继承Simulator基类, 根据各自仿真器的特点订阅话题将接口信息补充完整即可. 最终将simulator对象传给planner
   ```shell
   比如要创建新的carla仿真器, 则创建一个carlaSimulator : Simulator的类, 同时创建对应的.h .cpp文件, 在此新类中, 通过订阅carla的ros话题将Simulator基类中的所有接口信息填充完整即可, 最终通过工厂模式都是传送统一的simulator对象给planner层
   ```

   3.2planner框架怎么使用?
   planner拿到的是当前的仿真器信息, 因此新的局部规划器只需要将规划的结果放到localTrajectory_中即可
```shell
比如要创建新的mpc规划器, 则创建一个newmpcPlanner : Planner的类, 同时创建对应的.h .cpp文件, 在此新类中, 重写plan函数即可, 将planner规划的结果放到localTrajectory_成员变量中
```
4.示例:
4.1在我们写好了新的仿真器类\规划器类后, 只需要在第4行\第8行替换成新写的类即可, main函数内其他所有的地方都不需要更改;  
4.2新写的类完全不会影响原有的类, 是并存的关系, 想用哪个就自己调用哪个, 因为接口均是统一的, 比如新添加了carla类, 那么任何planner都可以继续使用
```shell
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
```