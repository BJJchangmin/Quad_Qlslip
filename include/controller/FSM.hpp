#ifndef FSM_HPP_
#define FSM_HPP_

#include <mujoco/mujoco.h>

#include "RobotLeg.hpp"
#include "CompensationControl.hpp"
#include "StanceForceControl.hpp"
#include "FlightControl.hpp"
#include "TrajectoryOptimization.hpp"

/**
 * todo: state and phase를 어떻게 넘길 것인지 고민해야함
 */

template <typename T>
class FSM
{
  private:
    RobotLeg<T> & robot_;
    CompensationControl<T> & comp_ctrl_;
    FlightControl<T> & flight_ctrl_;
    StanceForceControl<T> & stance_ctrl_;

    T touch_threshold_, start_[4], threshold_size_, time_;
    Eigen::VectorXd touch_[4];
    T phase_[4]; // 0: Stance, 1: Flight
    T event_[4]; // mean Touch down or Lift off

    std::shared_ptr<typename TrajectoryOptimization<T>::LO_param> lo_param_ptr_;
    std::shared_ptr<typename TrajectoryOptimization<T>::TD_param> td_param_ptr_;



  public:
    explicit FSM(RobotLeg<T> & robot, CompensationControl<T> & comp_ctrl, FlightControl<T> & flight_ctrl,
                 StanceForceControl<T> & stance_ctrl);

    void phase_update(mjData * d);
    void Lift_off_state(int Leg_num);
    void Touch_down_state(int Leg_num);
    void FSM_control();


  void get_optimization_pointer(std::shared_ptr<typename TrajectoryOptimization<T>::LO_param> lo_param_ptr,
                                std::shared_ptr<typename TrajectoryOptimization<T>::TD_param> td_param_ptr);

};



#endif // FSM_HPP_