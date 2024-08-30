#ifndef Flight_Control_HPP_
#define Flight_Control_HPP_

#include "MotionTrajectory.hpp"
#include "RobotLeg.hpp"

template <typename T>
class FlightControl
{
private:
  RobotLeg<T> & robot_;

  T kp_r_[4], kd_r_[4], kp_th_[4], kd_th_[4];
  Vec2<T> error_pos_[4], error_vel_[4];

  Vec2<T> force_rw_flight_des_[4];

  std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr_;

public:
  FlightControl(RobotLeg<T> & robot);

  void get_traj_pointer(std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr);

  void th_control(int Leg_num); // th_direction Control
  void r_control(int Leg_num); // r direction Control
  void flight_control(int Leg_num); // Combine th_control and r_control

};

#endif // Flight_Control_HPP_
