#ifndef MOTION_TRAJECTORY_HPP_
#define MOTION_TRAJECTORY_HPP_

#include <mujoco/mujoco.h>
#include <memory>

#include "Mclquad.hpp"
#include "MuJoCoInterface.hpp"
#include "RobotLeg.hpp"

template <typename T>
class MotionTrajectory
{
  private:
    RobotLeg<T> & robot_;

    T t_;
    T T_pause_;   // pause duration
    T T_crouch_;  // crouch duration
    // T f_crouch_;  // crouch frequency = 1 / T_crouch_
    T T_land_;     // landing duration
    T T_recover_;  // recover duration

    T h_max_, h_min_, h_home_;  // max/min and home height of body

    // parameter for Test StanceForceControl
    T dthr_init_;

    T state_;

  public:
    struct DesiredFootTrajectory
    {
        Vec2<T> foot_pos_rw_des_[4];
        Vec2<T> foot_vel_rw_des_[4];

        T r_optimized_flight_[4];
        T dr_optimized_flight_[4];
        T th_optimized_flight_[4];
        T dth_optimized_flight_[4];

    };

    struct DesiredJointTrajectory
    {
        Vec3<T> joint_pos_des_[4];
        Vec3<T> joint_vel_des_[4];
        Vec3<T> joint_torque_des_[4];
    };

    std::shared_ptr<DesiredFootTrajectory> foot_traj_ptr_;
    std::shared_ptr<DesiredFootTrajectory> get_foot_traj_ptr();

    std::shared_ptr<DesiredJointTrajectory> joint_traj_ptr_;
    std::shared_ptr<DesiredJointTrajectory> get_joint_traj_ptr();

  public:

    MotionTrajectory(RobotLeg<T> & robot_);
    void squat(T time);
    void stance_test();
    void QLSLIP_Trajectory(T r_ref, T v_ref);


};

#endif  // MOTION_TRAJECTORY_HPP_
