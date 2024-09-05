#include "FlightControl.hpp"
#include "orientation_tools.hpp"

#include <iostream>
using namespace std;
using namespace ori;

template <typename T>
FlightControl<T>::FlightControl(RobotLeg<T> & robot) : robot_(robot)
{
  foot_traj_ptr_ = nullptr;

  for (size_t i = 0; i < 4; i++)
  {
    error_pos_[i] = Vec2<T>::Zero();
    error_vel_[i] = Vec2<T>::Zero();

    kp_r_[i] = 20*100;
    kd_r_[i] = 15*10;
    kp_th_[i] =10*200;
    kd_th_[i] = 10*10;

    force_rw_flight_des_[i] = Vec2<T>::Zero();

  }
}

template <typename T>
void FlightControl<T>::th_control(int Leg_num)
{
  /**
   * @brief th_direction Control in Flight phase
   */

  error_pos_[Leg_num][1] = foot_traj_ptr_->foot_pos_rw_des_[Leg_num][1] - robot_.foot_pos_rw_act_local_[Leg_num][1];
  error_vel_[Leg_num][1] = foot_traj_ptr_->foot_vel_rw_des_[Leg_num][1] - robot_.foot_vel_rw_act_local_[Leg_num][1];

  force_rw_flight_des_[Leg_num][1] = kp_th_[Leg_num] * error_pos_[Leg_num][1] + kd_th_[Leg_num] * error_vel_[Leg_num][1];


}

template <typename T>
void FlightControl<T>::r_control(int Leg_num)
{
  /**
   * @brief r direction Control in Flight phase
   */


  error_pos_[Leg_num][0] = foot_traj_ptr_->foot_pos_rw_des_[Leg_num][0] - robot_.foot_pos_rw_act_local_[Leg_num][0];
  error_vel_[Leg_num][0] = foot_traj_ptr_->foot_vel_rw_des_[Leg_num][0] - robot_.foot_vel_rw_act_local_[Leg_num][0];

  force_rw_flight_des_[Leg_num][0] = kp_r_[Leg_num] * error_pos_[Leg_num][0] + kd_r_[Leg_num] * error_vel_[Leg_num][0];


}

template <typename T>
void FlightControl<T>::flight_control(int Leg_num)
{
  /**
   * @brief Combine th_control and r_control
   */
  th_control(Leg_num);
  r_control(Leg_num);


  robot_.inverse_static_rotating(force_rw_flight_des_[Leg_num], Leg_num);

}

template <typename T>
void FlightControl<T>::get_traj_pointer(std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr)
{
  foot_traj_ptr_ = foot_traj_ptr;
}

template class FlightControl<double>;
template class FlightControl<float>;

