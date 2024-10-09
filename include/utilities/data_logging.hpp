
#ifndef DATA_LOGGING_HPP_
#define DATA_LOGGING_HPP_

// C++ standard libraries
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "MotionTrajectory.hpp"
#include "Mclquad.hpp"
#include "TrajectoryOptimization.hpp"
#include "FSM.hpp"


template <typename T>
class DataLogging
{

  private:
    std::ofstream fout_[5];
    static constexpr int k_save_sampling = 1;  // sampling rate at which data is saved
    RobotLeg<T> & robot_;

    std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr_;
    std::shared_ptr<typename MotionTrajectory<T>::DesiredJointTrajectory> joint_traj_ptr_;
    std::shared_ptr<typename TrajectoryOptimization<T>::Optimization_param> op_param_ptr_;
    std::shared_ptr<typename TrajectoryOptimization<T>::LO_param> lo_param_ptr_;
    std::shared_ptr<typename TrajectoryOptimization<T>::TD_param> td_param_ptr_;
    std::shared_ptr<typename FSM<T>::PCV> pcv_ptr_;

  public:
    explicit DataLogging(RobotLeg<T> & robot_);

    void init_data();
    void save_data(const mjModel * m, mjData * d);


   void set_traj_ptr(
    std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr,
    std::shared_ptr<typename MotionTrajectory<T>::DesiredJointTrajectory> joint_traj_ptr);

    void set_op_param_ptr(
      std::shared_ptr<typename TrajectoryOptimization<T>::Optimization_param> op_param_ptr,
      std::shared_ptr<typename TrajectoryOptimization<T>::LO_param> lo_param_ptr,
      std::shared_ptr<typename TrajectoryOptimization<T>::TD_param> td_param_ptr);

    void set_pcv_ptr(std::shared_ptr<typename FSM<T>::PCV> pcv_ptr);

    int get_logging_freq();



};




#endif  // data_logging_HPP_
