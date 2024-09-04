#ifndef Stance_Force_Control_HPP_
#define Stance_Force_Control_HPP_

#include "MotionTrajectory.hpp"
#include "RobotLeg.hpp"


template <typename T>
class StanceForceControl
{
private:
  RobotLeg<T> & robot_;

  /**
   * @brief Stance Force Control
   * @param spring_K_: kp gain in r direction position
   * @param kp_tau_: kp gain in theta direction velocity
   */

  // spring force control
  T spring_K_[4];
  // tau_control
  T kp_tau_[4], kd_tau_[4];

  Vec2<T> error_pos_[4], error_vel_[4];
  T thbr_[4], dthbr_[4], dr_[4];


  T g;

  Vec2<T> force_rw_stance_des_[4];

  std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr_;

public:
  StanceForceControl(RobotLeg<T> & robot);

  void get_traj_pointer(std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr);


  void spring_force_control(int Leg_num); // r direction force control

  void tau_control(int Leg_num); // theta direction torque control

  void stance_control(int Leg_num); // combine spring_force_control and tau_control



};




#endif // Stance_Force_Control_HPP_