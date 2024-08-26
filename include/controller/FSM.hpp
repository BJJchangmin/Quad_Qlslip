#ifndef FSM_HPP_
#define FSM_HPP_

#include <mujoco/mujoco.h>

#include "RobotLeg.hpp"
#include "CompensationControl.hpp"
#include "StanceForceControl.hpp"

/**
 * todo: Add Flight Control, Stance Control, TrajectoryCasADi
 */

template <typename T>
class FSM
{
  private:
    RobotLeg<T> & robot_;
    CompensationControl<T> & comp_ctrl_;

    T touch_threshold_, start_;
    Eigen::VectorXd touch_[4];
    T Phase_[4]; // 0: Stance, 1: Flight
    T event_[4]; // mean Touch down or Lift off


  public:
    explicit FSM(RobotLeg<T> & robot, CompensationControl<T> & comp_ctrl);

    void phase_update(mjData * d);
    void Lift_off_state(int Leg_num);
    void Touch_down_state(int Leg_num);
    void FSM_control(int Leg_num);




};



#endif // FSM_HPP_