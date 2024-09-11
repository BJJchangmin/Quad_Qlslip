#include "StanceForceControl.hpp"
#include "orientation_tools.hpp"

#include <iostream>

using namespace std;
using namespace ori;

template <typename T>
StanceForceControl<T>::StanceForceControl(RobotLeg<T> & robot) : robot_(robot)
{
  foot_traj_ptr_ = nullptr;

  g= 9.81;

  for (size_t i = 0; i < 4; i++)
  {
    error_pos_[i] = Vec2<T>(0,0);
    error_vel_[i] = Vec2<T>(0,0);
    thbr_[i] = 0;
    dthbr_[i] = 0;
    dr_[i] = 0;

    spring_K_[i] = 20*50000;
    kp_tau_[i] = 10*10;
    kd_tau_[i] = 10*2000;

    force_rw_stance_des_[i] = Vec2<T>(0,0);
  }

  //! 뒷다리가 무게로 인해 Tracking이 더 어려워서 따로 게인을 설정해봄



}

template <typename T>
void StanceForceControl<T>::spring_force_control(int Leg_num)
{
/**
 * @brief Stance Force Control
 * @param spring_K_: kp gain in r direction position
 * @param force_rw_stance_des_: desired force in r direction

 *
 */


  thbr_[Leg_num] = robot_.joint_pos_act_[Leg_num][2];
  dthbr_[Leg_num] = robot_.joint_vel_act_[Leg_num][2];
  dr_[Leg_num] = robot_.foot_vel_rw_act_local_[Leg_num][0];

  error_pos_[Leg_num][0] = foot_traj_ptr_->foot_pos_rw_des_[Leg_num][0] - robot_.foot_pos_rw_act_local_[Leg_num][0];

  force_rw_stance_des_[Leg_num][0] = spring_K_[Leg_num] * error_pos_[Leg_num][0] -(robot_.M_d_R)*(1/(2*tan(thbr_[Leg_num]/2)))*dthbr_[Leg_num]*dr_[Leg_num]+
    (robot_.M_d_R + robot_.thigh_mass_[Leg_num]+robot_.shank_mass_[Leg_num] + 100)*g;

  // force_rw_stance_des_[Leg_num][0] = spring_K_[Leg_num] * error_pos_[Leg_num][0];

}

template <typename T>
void StanceForceControl<T>::tau_control(int Leg_num)
{
  /**
   * @brief Stance Force Control in theta direction
   */


  error_vel_[Leg_num][1] = foot_traj_ptr_->foot_vel_rw_des_[Leg_num][1] - robot_.foot_vel_rw_act_local_[Leg_num][1];
  error_pos_[Leg_num][1] = foot_traj_ptr_->foot_pos_rw_des_[Leg_num][1] - robot_.foot_pos_rw_act_local_[Leg_num][1];

  if ( robot_.phase_[Leg_num] == 0)
  {
    force_rw_stance_des_[Leg_num][1] = kp_tau_[Leg_num] * error_pos_[Leg_num][1];
  }
  else
  {
    force_rw_stance_des_[Leg_num][1] = kd_tau_[Leg_num] * error_vel_[Leg_num][1];
  }




}

template <typename T>
void StanceForceControl<T>::stance_control(int Leg_num)
{
  /**
   * @brief stance_control combine spring_force_control and tau_control
   * @brief It is used in FSM.cpp
   * @brief RW 방향으로 제어한 후 joint torqe로 변환해 주는 과정이 포함되어 있음
   * @brief robot_.joint_torque_des_로 변환이 되는데 stance,flight가 첫번째 제어기라 inverse_static_rotating만 사용해주면 됨
   */

  spring_force_control(Leg_num);
  // std::cout << "1 : " << force_rw_stance_des_[0][0] << ", "<< force_rw_stance_des_[0][1] << std::endl;
  tau_control(Leg_num);
  // std::cout << "2 : " << force_rw_stance_des_[0][0] << ", "<< force_rw_stance_des_[0][1] << std::endl;

  robot_.inverse_static_rotating(force_rw_stance_des_[Leg_num],Leg_num);

}

template <typename T>
void StanceForceControl<T>::get_traj_pointer(std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr)
{
  foot_traj_ptr_ = foot_traj_ptr;
}



template class StanceForceControl<float>;
template class StanceForceControl<double>;
