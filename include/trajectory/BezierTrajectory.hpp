#ifndef BezierTrajectory_HPP_
#define BezierTrajectory_HPP_

#include "MotionTrajectory.hpp"
#include "RobotLeg.hpp"
#include "TrajectoryOptimization.hpp"
#include <vector>

template <typename T>
class BezierTrajectory
{

private:
  RobotLeg<T> & robot_;

  T r_ref[4], dth_ref[4];
  T bx_2[4]; // Bezier Cartesian function, x direction
  T by_2[4]; // Bezier Cartesian function, y direction
  Vec2<T> RW_Bezier[4]; // [Legnum][r,th] -> position Reference
  Vec2<T> p0[4], p1[4], p2[4]; // Bezier Control Point [x ; y] -> Cartesian이기에 RW로 변환 필요
  T swing_period[4];
  T Bezier_flight_time[4];
  T t_B[4];

  std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr_;
  std::shared_ptr<typename TrajectoryOptimization<T>::Optimization_param> op_param_ptr_;
  std::shared_ptr<typename TrajectoryOptimization<T>::LO_param> lo_param_ptr_;
  std::shared_ptr<typename TrajectoryOptimization<T>::TD_param> td_param_ptr_;

public:

  BezierTrajectory(RobotLeg<T> & robot);

  void get_traj_pointer(std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr);
  void get_optimization_pointer(std::shared_ptr<typename TrajectoryOptimization<T>::Optimization_param> op_param_ptr,
                                std::shared_ptr<typename TrajectoryOptimization<T>::LO_param> lo_param_ptr,
                                std::shared_ptr<typename TrajectoryOptimization<T>::TD_param> td_param_ptr);


  void Flight_traj_generate(mjData * d);
  void state_update(int Leg_num);
  void Desired_Touch_Down_state(int Leg_num);
  void Desired_Flight_Time(int Leg_num);
  void Bezier_Trajectory(int Leg_num, T time_);

};

#endif // BezierTrajectory_HPP_
