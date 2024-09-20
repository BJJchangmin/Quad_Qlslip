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
    error_pos_old_[i] = Vec2<T>::Zero();

    error_vel_[i] = Vec2<T>::Zero();
    error_vel_old_[i] = Vec2<T>::Zero();

    kp_r_[i] = 20*400;
    kd_r_[i] = 15*5;
    kp_th_[i] =10*75;
    kd_th_[i] = 1.1;

    force_rw_flight_des_[i] = Vec2<T>::Zero();

  }


}

template <typename T>
void FlightControl<T>::th_control(int Leg_num)
{
  /**
   * @brief th_direction Control in Flight phase
   */
  error_vel_old_[Leg_num][1] = error_vel_[Leg_num][1];
  error_pos_old_[Leg_num][1] = error_pos_[Leg_num][1];
  error_pos_[Leg_num][1] = foot_traj_ptr_->foot_pos_rw_des_[Leg_num][1] - robot_.foot_pos_rw_act_local_[Leg_num][1];
  // error_vel_[Leg_num][1] = foot_traj_ptr_->foot_vel_rw_des_[Leg_num][1] - robot_.foot_vel_rw_act_local_[Leg_num][1];

  error_vel_[Leg_num][1] = tustin_derivative(error_pos_[Leg_num][1], error_pos_old_[Leg_num][1], error_vel_[Leg_num][1], error_vel_old_[Leg_num][1], 50);

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

  force_rw_flight_des_[Leg_num][0] = kp_r_[Leg_num] * error_pos_[Leg_num][0];


}

template <typename T>
void FlightControl<T>::flight_control(int Leg_num)
{
  /**
   * @brief Combine th_control and r_control
   */
  th_control(Leg_num);
  r_control(Leg_num);
  // cout << "force_rw_flight_des_[Leg_num][0] : " << force_rw_flight_des_[1][0] << endl;
  // cout << "force_rw_flight_des_[Leg_num][1] : " << force_rw_flight_des_[1][1] << endl;
  robot_.inverse_static_rotating(force_rw_flight_des_[Leg_num], Leg_num);



}

template <typename T>
T FlightControl<T>::tustin_derivative(T input, T input_old, T output,T output_old, T cut_off)
{
  /**
  * @brief Tustin derivative filter
  */

  T Ts = 0.001;
  T time_const = 1 / (2 * M_PI * cut_off);
  output = 0;

  output = (2 * (input - input_old) - (Ts - 2 * time_const) * output_old) / (Ts + 2 * time_const);
  return output;

}

template <typename T>
void FlightControl<T>::get_traj_pointer(std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr)
{
  foot_traj_ptr_ = foot_traj_ptr;
}

template class FlightControl<double>;
template class FlightControl<float>;

