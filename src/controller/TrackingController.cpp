#include "TrackingController.hpp"
#include "orientation_tools.hpp"

#include <iostream>

using namespace ori;
using namespace std;

template <typename T>
TrackingController<T>::TrackingController(RobotLeg<T> & robot) : robot_(robot)
{
  foot_traj_ptr_ = nullptr;
  joint_traj_ptr_ = nullptr;

  for (size_t i = 0; i < 4; i++)
  {

    kp_[i] << 6000,4000;
    kd_[i] << 1000,500;

    force_rw_des_[i] = Vec2<T>::Zero();
  }
}

template <typename T>
void TrackingController<T>::joint_HAA_control()
{
  for (size_t i = 0; i < 4; i++)
  {
    robot_.joint_torque_des_[i][0] = 100 * (joint_traj_ptr_->joint_pos_des_[i][0] - robot_.joint_pos_act_[i][0]) +
                                     5 * (joint_traj_ptr_->joint_vel_des_[i][0] - robot_.joint_vel_act_[i][0]);


  }
}

template <typename T>
void TrackingController<T>::RW_posPD_control()
{
  robot_.forward_kinematics_rotating();

  Vec2<T> error_pos[4];
  Vec2<T> error_vel[4];

  for (size_t i = 0; i < 4; i++)
  {
    error_pos[i] = foot_traj_ptr_->foot_pos_rw_des_[i] - robot_.foot_pos_rw_act_local_[i];
    error_vel[i] = foot_traj_ptr_->foot_vel_rw_des_[i] - robot_.foot_vel_rw_act_local_[i];

    force_rw_des_[i][0] = kp_[i][0]*error_pos[i][0];
    force_rw_des_[i][1] = kp_[i][1]*error_pos[i][1];

    robot_.inverse_static_rotating(force_rw_des_[i],i);

  }


}

template <typename T>
void TrackingController<T>::get_traj_pointer(
    std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr,
    std::shared_ptr<typename MotionTrajectory<T>::DesiredJointTrajectory> joint_traj_ptr)
{

    foot_traj_ptr_ = foot_traj_ptr;
    joint_traj_ptr_ = joint_traj_ptr;

}

template class TrackingController<float>;
template class TrackingController<double>;

