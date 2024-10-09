#ifndef FSM_HPP_
#define FSM_HPP_

#include <mujoco/mujoco.h>

#include "RobotLeg.hpp"
#include "CompensationControl.hpp"
#include "StanceForceControl.hpp"
#include "FlightControl.hpp"
#include "TrajectoryOptimization.hpp"
#include "BezierTrajectory.hpp"
#include "MotionTrajectory.hpp"


template <typename T>
class FSM
{
  private:
    RobotLeg<T> & robot_;
    CompensationControl<T> & comp_ctrl_;
    FlightControl<T> & flight_ctrl_;
    StanceForceControl<T> & stance_ctrl_;
    TrajectoryOptimization<T> & traj_opt_;
    BezierTrajectory<T> & bezier_traj_;

    T touch_threshold_, start_[4], threshold_size_, time_;
    Eigen::VectorXd touch_[4];
    Vec2<T> phase_[4]; // 0: Stance, 1: Flight
    T event_[4]; // mean Touch down or Lift off
    T swing_lock_period_[4];
    T swing_lock_phase_[4];
    bool swing_lock_[4];
    int loop_iter;

    std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr_;
    std::shared_ptr<typename TrajectoryOptimization<T>::LO_param> lo_param_ptr_;
    std::shared_ptr<typename TrajectoryOptimization<T>::TD_param> td_param_ptr_;



  public:
    explicit FSM(RobotLeg<T> & robot, CompensationControl<T> & comp_ctrl, FlightControl<T> & flight_ctrl,
                 StanceForceControl<T> & stance_ctrl, TrajectoryOptimization<T> & traj_opt,
                 BezierTrajectory<T> & bezier_traj);

    struct PCV
    {
      /**
       * * PCV : Phase Control Variable
       * @param time: TD 이후로부터 0으로 시작하는 시간
       * @param Ratio: Stance Period를 Norminalize한 시간
       * @param update_Period: Feedback을 통해 LO time을 조정할 건데 stance period를 update한 값
       * @param p1: Feedback Gain
       * @param desired_Phase: Desired한 Phase 이다. 3점 지지는 0.25, 2점 지지는 0.5가 될텐데 기준이 되는 다리로 부터의 차이가 중요함
       */
      T time[4], Ratio[4], update_Period[4], p1[4], Des_Phase[4], GAP[4];
    };

    std::shared_ptr<PCV> pcv_ptr_;
    std::shared_ptr<PCV> get_pcv_ptr();

    void phase_update(mjData * d);
    void Lift_off_state(int Leg_num);
    void Touch_down_state(int Leg_num);
    void FSM_control();
    void PCV_control(int Leg_num);

    void get_traj_pointer(std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr);
    void get_optimization_pointer(std::shared_ptr<typename TrajectoryOptimization<T>::LO_param> lo_param_ptr,
                                  std::shared_ptr<typename TrajectoryOptimization<T>::TD_param> td_param_ptr);


};



#endif // FSM_HPP_