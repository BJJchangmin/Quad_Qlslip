#include "MotionTrajectory.hpp"

#include <cmath>

using namespace std;

template <typename T>
MotionTrajectory<T>::MotionTrajectory(RobotLeg<T> & robot_)
  :
    robot_(robot_),
    t_(0.0),
    T_pause_(1.0),
    T_crouch_(1.0),
    T_land_(0.5),
    T_recover_(0.5),
    h_max_(0.5),
    h_min_(0.2),
    h_home_(0.32355),
    dthr_init_(0.0)
{

    foot_traj_ptr_ = std::make_shared<DesiredFootTrajectory>();
    joint_traj_ptr_ = std::make_shared<DesiredJointTrajectory>();
}

template <typename T>
void MotionTrajectory<T>::stance_test()
{
  /**
   * @brief Stance_Test
   * @param joint_traj_ptr_ for HAA fix
   * @param foot_traj_ptr_ for foot trajectory
   */

  for (size_t i = 0; i < 4; i++)
  {
    //**RW control */
    foot_traj_ptr_->foot_pos_rw_des_[i] << h_home_,M_PI / 2;

    //** Joint Velocity Control for HAA */
    joint_traj_ptr_->joint_vel_des_[i][0]= 0.0;
    joint_traj_ptr_->joint_pos_des_[i][0]= 0.0;

  }

}

template <typename T>
void MotionTrajectory<T>::QLSLIP_Trajectory(T r_ref, T v_ref, mjData * d)
{
  /**
   * @brief QLSLIP Trajectory
   * @param r_ref: reference r position
   * @param v_ref: reference r velocity
   * @param joint_traj_ptr_ for HAA fix
   * @param foot_traj_ptr_ for foot trajectory
   */

  T dth_ref = - v_ref / r_ref;
  T f = 2;

  for (size_t i = 0; i < 4; i++)
  {

    // ****************************************** Stance Control ****************************************** */
    if (robot_.phase_[i] == 1)
    {
      foot_traj_ptr_->foot_pos_rw_des_[i][0] = r_ref;
      foot_traj_ptr_->foot_vel_rw_des_[i][1] = dth_ref;
    }
    // ****************************************** Flight Control ****************************************** */
    else if (robot_.phase_[i] == 2)
    {
      foot_traj_ptr_->foot_pos_rw_des_[i][0] = foot_traj_ptr_->r_optimized_flight_[i];
      foot_traj_ptr_->foot_pos_rw_des_[i][1] = foot_traj_ptr_->th_optimized_flight_[i];

      foot_traj_ptr_->foot_vel_rw_des_[i][0] = foot_traj_ptr_->dr_optimized_flight_[i];
      foot_traj_ptr_->foot_vel_rw_des_[i][1] = foot_traj_ptr_->dth_optimized_flight_[i];
    }
    // ****************************************** Joint Control ****************************************** */
    else
    {
      foot_traj_ptr_->foot_pos_rw_des_[i][0] = 0.4 ;
      foot_traj_ptr_->foot_pos_rw_des_[i][1] = M_PI / 2 ;
      foot_traj_ptr_->foot_vel_rw_des_[i][1] =  0.0 ;
    }

  }
}

template <typename T>
void MotionTrajectory<T>::initial_trotting(T r_init ,T time_, T freq)
{
  /**
   * * 시작전에 trotting을 하다가 우송이형 알고리즘으로 넘어가기 위함
   * @brief Initial Trotting
   * @param FL,RR(0,3) 3*i
   * @param FR,RL(1,2) i+1
   */

  for (size_t i = 0; i < 2; i++)
  {
    //* FL,RR */
    foot_traj_ptr_->foot_pos_rw_des_[3*i][0] = r_init + 0.1*sin(2*M_PI*freq*time_);
    foot_traj_ptr_->foot_pos_rw_des_[3*i][1] = M_PI / 2;

    //* FR,RL */
    foot_traj_ptr_->foot_pos_rw_des_[i+1][0] = r_init - 0.1*sin(2*M_PI*freq*time_);
    foot_traj_ptr_->foot_pos_rw_des_[i+1][1] = M_PI / 2;

  }
}


template <typename T>
std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> MotionTrajectory<T>::get_foot_traj_ptr()
{
  return foot_traj_ptr_;
}

template <typename T>
std::shared_ptr<typename MotionTrajectory<T>::DesiredJointTrajectory> MotionTrajectory<T>::get_joint_traj_ptr()
{
  return joint_traj_ptr_;
}




template class MotionTrajectory<float>;
template class MotionTrajectory<double>;

