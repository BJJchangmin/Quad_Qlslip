/*! @file RobotLeg.hpp
 *  @brief Data structure containing parameters of a robot leg
 */

#ifndef MCL_ROBOT_LEG_HPP_
#define MCL_ROBOT_LEG_HPP_

// C++ standard library
#include <iostream>
#include <memory>
#include <vector>

// Eigen3
#include <eigen3/Eigen/StdVector>

// MuJoCo
#include <mujoco/mujoco.h>

// customized headers
#include "cppTypes.hpp"

using std::vector;

template <typename T>
class RobotLeg
{
//! Make Variable in here, declare definition in own Robotfile (Mclquad.hpp)
//! Receive sensor data in RobotLeg.cpp -> if change robot xml file, change sensor number
public:
  //* Physical parameters *//
  T hip_mass_[4], thigh_mass_[4], shank_mass_[4];
  T thigh_link_length_[4], shank_link_length_[4];
  T leg_length_max_[4], leg_length_min_[4], swept_angle_max_[4], swept_angle_min_[4];
  T leg_length_home_[4], swept_angle_home_[4];
  Vec3<T> hip_com_location_[4], thigh_com_location_[4], shank_com_location_[4];
  Vec3<T> hip_joint_location_[4], knee_joint_location_[4];
  Vec3<T> thigh_body_inertia_[4], shank_body_inertia_[4];    // inertia of thigh & shank w.r.t. its' CoM
  Vec3<T> thigh_joint_inertia_[4], shank_joint_inertia_[4];  // inertia of thigh & shank w.r.t. its' joint

  // Vec3<T> foot_pos_cart_local_, foot_vel_cart_local_;
  // Vec3<T> foot_pos_rw_world_, foot_pos_rw_local_, foot_vel_rw_world_, foot_vel_rw_local_;

  //* variables for control parameters *//
  T foot_contact_threshold_[4];
  bool bIsContact_;

  Mat2<T> jacbRW[4], jacbRW_inv[4];
  Mat2<T> jacbRW_tran[4], jacbRW_tran_inv[4];

  //! Be careful Vec3, Vec2 -> Because of HAA
  Vec2<T> kp_[4], kd_[4];
  Vec3<T> joint_pos_des_[4], joint_vel_des_[4], joint_torque_des_[4];
  Vec2<T> joint_pos_bi_act_[4], joint_vel_bi_act_[4], joint_torque_bi_des_[4];
  Vec2<T> foot_pos_cart_des_local_[4], foot_vel_cart_des_local_[4];
  Vec2<T> foot_pos_cart_act_local_[4], foot_vel_cart_act_local_[4];
  Vec2<T> foot_pos_rw_des_local_[4], foot_vel_rw_des_local_[4];
  Vec2<T> foot_pos_rw_act_local_[4], foot_vel_rw_act_local_[4];

  //* variables for sensor data *//
  // todo: Add Body Pos,vel Sensor
  Vec3<T> body_pos_world_, body_vel_world_, body_vel_local_, body_acc_local_;
  Vec3<T> body_ang_vel_local_, body_ang_vel_world_;
  Quat<T> body_quat_;
  T Trunk_mass;

  Vec3<T> joint_pos_act_[4], joint_vel_act_[4];

  Vec3<T> foot_pos_cart_world_[4], foot_vel_cart_world_[4];
  Vec3<T> foot_grf_world_[4], foot_grf_local_[4];
  T foot_contact_[4];

  //************************************* Custom VARIABLES ************************************************
  T M_d_R;
  T phase_[4],event_[4];



  /**
   * @brief Make for StanceForceControl and CompensationControl
   * @brief there is a reason why i write joint_des.please check in controller
   * @todo Flight phase control
   *
   */

  //* variables for StanceForceControl *//
  Vec2<T> stance_force_contorl_joint_des_;

  //* Variables for Compensation Control *//
  Vec2<T> compensation_force_joint_des_;

  //************************************* CONSTANTS ************************************************
  static constexpr size_t k_num_joint = 3;     // num of leg joints
  static constexpr size_t k_num_dof_body = 6;  // num of body DoF
  static constexpr size_t k_num_dof_leg = 4;   // num of leg DoF

  //************************************* METHODS **************************************************
  void get_sensor_data(mjData * data);
  void get_jacobians_rotating();
  void bi_kinematic_transform();
  void bi_inverse_static_transform();

  void inverse_static_rotating(const Vec2<T> & force_rw_des,int Leg_num);
  void forward_kinematics_cartesian();
  void forward_kinematics_rotating();
};

#endif  // MCL_ROBOT_LEG_HPP_