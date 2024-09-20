#ifndef TRAJECTORYOPTIMIZATION_HPP_
#define TRAJECTORYOPTIMIZATION_HPP_

#include <casadi/casadi.hpp>
#include "MotionTrajectory.hpp"
#include "RobotLeg.hpp"
#include <vector>
#include <eigen3/Eigen/Dense>

template <typename T>
class TrajectoryOptimization
{
private:
  RobotLeg<T> & robot_;
  T optimization_limit_;
  T r_ref[4],dth_ref[4];
  T time;


  std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr_;

public:

  TrajectoryOptimization(RobotLeg<T> & robot);

  struct Optimization_param
  {
    /**
     * @brief parameter for Desired Touch Down state
     * @param : param이 뭔지는 더 잘 아실거라고 생각합니닷.
     * todo : what is t_stance? stance를 하는 총 시간? or stance로 전환되는 시간? 확인
     * ! 위에는 input parameter 아래는 output parameter
     * !h_1이 뭔데? simulink에서는 1.6
     */
    T  h_1[4], u[4], th_r[4];
    T r_des_top[4], r_des_TD[4], dr_des_TD[4], ddr_des_TD[4], th_des_top[4], th_des_TD[4], dth_des_TD[4];
    /**
     * @brief Desired Flight Time
     * ! 위에는 input parameter 아래는 output parameter
     * @param : r_desired_TD, th_desired_TD는 위에서 가져온다.
     */

    T t_flight_des[4];

    /**
     * @brief Radial/Angular optimization
     * !위에서 구한 함수들의 parameter output을 다수 사용
     * TODO: X_r은 vector 9개 , x_th
     * TODO: Eigen을 쓰든 vector를 쓰든 아무거나
     */

    Eigen::VectorXd X_r[4], X_th[4];


    Eigen::VectorXd T_r[4], T_dr[4], T_th[4], T_dth[4];
    T t_r[4], t_th[4];
    T r_optimized_flight[4], dr_optimized_flight[4], th_optimized_flight[4], dth_optimized_flight[4];

  };

  struct LO_param
  {
    T r_LO[4], dr_LO[4], th_LO[4], dth_LO[4], t_LO[4], V_y_LO[4], t_stance[4];
  };

  struct TD_param
  {
    T r_TD[4], dr_TD[4], th_TD[4], dth_TD[4], t_TD[4];
  };

  void get_traj_pointer(std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr);

  std::shared_ptr<LO_param> lo_param_ptr_;
  std::shared_ptr<TD_param> td_param_ptr_;
  std::shared_ptr<Optimization_param> op_param_ptr_;

  std::shared_ptr<LO_param> get_lo_param_ptr();
  std::shared_ptr<TD_param> get_td_param_ptr();
  std::shared_ptr<Optimization_param> get_op_param_ptr();

  void state_update(int Leg_num);
  void Desired_Touch_Down_state(int Leg_num);
  void Desired_Flight_Time(int Leg_num);
  void Radial_Optimization(int Leg_num);
  void Angular_Optimization(int Leg_num);
  void Polynomial_Trajectory(int Leg_num);
  void Flight_traj_generate(mjData * d);





};





#endif // FlightTrajectoryCasADi_HPP_