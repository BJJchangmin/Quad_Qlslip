#include "/home/ycm/mujoco/quad_qlslip/include/trajectory/BezierTrajectory.hpp"
#include "orientation_tools.hpp"

#include <iostream>

using namespace std;
using namespace ori;

template <typename T>
BezierTrajectory<T>::BezierTrajectory(RobotLeg<T> & robot) : robot_(robot)
{

  foot_traj_ptr_ = nullptr;
  op_param_ptr_ = nullptr;
  lo_param_ptr_ = nullptr;
  td_param_ptr_ = nullptr;
}

template <typename T>
void BezierTrajectory<T>::Flight_traj_generate(mjData * d)
{
  /**
   * * Bezier Trajectory로 Trajectory를 만드는 곳
   */
  for (size_t i = 0; i < 4; i++)
  {
    if (robot_.event_[i] == 4)
    {
      state_update(i);
      Desired_Touch_Down_state(i);
      Desired_Flight_Time(i);
    };

    if (robot_.phase_[i] == 2)
    {
      Bezier_Trajectory(i,d->time);
    };
  }
}


template <typename T>
void BezierTrajectory<T>::state_update(int Leg_num)
{
  lo_param_ptr_->th_LO[Leg_num] -= M_PI/2;
  td_param_ptr_->th_TD[Leg_num] -= M_PI/2;

  r_ref[Leg_num] = foot_traj_ptr_->foot_pos_rw_des_[Leg_num][0];
  dth_ref[Leg_num] = foot_traj_ptr_->foot_vel_rw_des_[Leg_num][1];
}

template <typename T>
void BezierTrajectory<T>::Desired_Touch_Down_state(int Leg_num)
{
  /**
   * @brief Desired Touch Down state를 계산하는 함수
   */

  T B_m=2000;
  T k=1e4;
  T m= 6;
  T g = 9.81;

  T alpha=sqrt(k/m);
  T a=B_m/(m*r_ref[Leg_num]);
  T b=g/r_ref[Leg_num];
  T e_1=(-a+sqrt(pow(a,2)-4*b))/2;
  T e_2=(-a-sqrt(pow(a,2)-4*b))/2;
  T c=pow((e_2-e_1),2);
  T x_1=2*b+a*e_1;
  T x_2=2*b+a*e_2;

  T st_time=abs(2*(-(td_param_ptr_->th_TD[Leg_num])/dth_ref[Leg_num]));
  // T st_time = lo_param_ptr_->t_stance[Leg_num];
  // T st_time = 0.1;

  op_param_ptr_->th_r[Leg_num] = (2 * dth_ref[Leg_num]) * (((e_2 -e_1) + (e_1*exp(e_1*st_time) - e_2*exp(e_2*st_time))+
  a*(exp(e_1*st_time) - exp(e_2 * st_time)))/(e_1*e_2*(exp(e_1*st_time)-exp(e_2*st_time))));
  // T th_r = 0;

  // desired top
  op_param_ptr_->r_des_top[Leg_num] = 0.32;
  op_param_ptr_->th_des_top[Leg_num] = 0;



  op_param_ptr_-> h_1[Leg_num] = 1.5;
  op_param_ptr_-> u[Leg_num] = op_param_ptr_-> h_1[Leg_num] * ((-op_param_ptr_->th_r[Leg_num]/2)- lo_param_ptr_->th_LO[Leg_num]) +
    op_param_ptr_->th_r[Leg_num];

  op_param_ptr_->r_des_TD[Leg_num] = 0.4;
  // op_param_ptr_->r_des_TD[Leg_num] = r_ref[Leg_num];
  op_param_ptr_->th_des_TD[Leg_num] = lo_param_ptr_->th_LO[Leg_num] + op_param_ptr_->u[Leg_num];

  T t_b = st_time/2;

  op_param_ptr_->dr_des_TD[Leg_num] = (alpha*(m*g/k)*tan(alpha*t_b));
  //std::cout << "dr_des_TD : " << op_param_ptr_->dr_des_TD[1] << std::endl;
  op_param_ptr_->dth_des_TD[Leg_num] = dth_ref[Leg_num];

  op_param_ptr_->ddr_des_TD[Leg_num] = -g;

}

template <typename T>
void BezierTrajectory<T>::Desired_Flight_Time(int Leg_num)
{
  /**
   * @brief Desried Flight Time을 계산하는 곳
   * @brief 여기 input은 Desired Touch Down state에서 계산한 값들을 가져와서 사용한다.
   * todo 이거 V_y_LO는 Trunk y방향 속도롤 해보기. 그러며 다리에 대해서 같을 듯
   */
  T g = 9.81;
  // cout << "V_y_LO: " << lo_param_ptr_->V_y_LO[0] << endl;
  op_param_ptr_->t_flight_des[Leg_num] = abs(1*2*lo_param_ptr_->V_y_LO[Leg_num]/g)*2;
}

template <typename T>
void BezierTrajectory<T>::Bezier_Trajectory(int Leg_num, T time_)
{
  /**
   * * Bezier Trajectory를 계산하는 곳
   * @param Control_Point: p0, p1, p2
   * todo: swing period를 여러 parameter 써봐야함. st_time or t_flight_des인데 전부 다 같으려면 st_time을 써야할 듯
   * ! FSM에서 Flight period가 정해져서 여기로 넘어온다. period를 맞춰주저야함
   */

  p0[Leg_num][0] = lo_param_ptr_->r_LO[Leg_num]*sin(lo_param_ptr_->th_LO[Leg_num]);
  p0[Leg_num][1] = lo_param_ptr_->r_LO[Leg_num]*cos(lo_param_ptr_->th_LO[Leg_num]);

  p2[Leg_num][0] = op_param_ptr_->r_des_TD[Leg_num]*sin(op_param_ptr_->th_des_TD[Leg_num]);
  p2[Leg_num][1] = op_param_ptr_->r_des_TD[Leg_num]*cos(op_param_ptr_->th_des_TD[Leg_num]);

  p1[Leg_num][0] = 2*(op_param_ptr_->r_des_top[Leg_num]*sin(op_param_ptr_->th_des_top[Leg_num]) - 0.25*p0[Leg_num][0] - 0.25*p2[Leg_num][0]);
  p1[Leg_num][1] = 2*(op_param_ptr_->r_des_top[Leg_num]*cos(op_param_ptr_->th_des_top[Leg_num]) - 0.25*p0[Leg_num][1] - 0.25*p2[Leg_num][1]);

  swing_period[Leg_num] = op_param_ptr_->t_flight_des[Leg_num];

  Bezier_flight_time[Leg_num] = time_ - lo_param_ptr_->t_LO[Leg_num];

  if (Bezier_flight_time[Leg_num] < swing_period[Leg_num])
  {
    t_B[Leg_num] = Bezier_flight_time[Leg_num]/swing_period[Leg_num];

    bx_2[Leg_num] = pow((1-t_B[Leg_num]),2)*p0[Leg_num][0] + 2*t_B[Leg_num]*(1-t_B[Leg_num])*p1[Leg_num][0] + pow(t_B[Leg_num],2)*p2[Leg_num][0];
    by_2[Leg_num] = pow((1-t_B[Leg_num]),2)*p0[Leg_num][1] + 2*t_B[Leg_num]*(1-t_B[Leg_num])*p1[Leg_num][1] + pow(t_B[Leg_num],2)*p2[Leg_num][1];

    RW_Bezier[Leg_num][0] = sqrt(pow(bx_2[Leg_num],2) + pow(by_2[Leg_num],2));
    RW_Bezier[Leg_num][1] = atan2(bx_2[Leg_num],by_2[Leg_num]);
    // cout << "p1 : " << p1[0][0] << endl;
    // cout << "r_des_top : " << op_param_ptr_->r_des_top[0] << endl;
    // cout <<" th_des_TD : " << op_param_ptr_->th_des_TD[0] << endl;
    // cout << "p2 : " << p2[0][0] << endl;
    // cout << "in period : " << RW_Bezier[0][0] << endl;

  }
  else
  {
    // cout << "No period : " << RW_Bezier[Leg_num][0] << endl;
    RW_Bezier[Leg_num][0] = op_param_ptr_->r_des_TD[Leg_num];
    RW_Bezier[Leg_num][1] = op_param_ptr_->th_des_TD[Leg_num];
  }
  // cout << "RW_Bezier[Leg_num][0] : " << RW_Bezier[Leg_num][0] << endl;

  foot_traj_ptr_->r_bezier_flight_[Leg_num] = RW_Bezier[Leg_num][0];
  foot_traj_ptr_->th_bezier_flight_[Leg_num] = RW_Bezier[Leg_num][1] + M_PI/2;


}



template <typename T>
void BezierTrajectory<T>::get_traj_pointer(std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr)
{
  foot_traj_ptr_ = foot_traj_ptr;
}

template <typename T>
void BezierTrajectory<T>::get_optimization_pointer(std::shared_ptr<typename TrajectoryOptimization<T>::Optimization_param> op_param_ptr,
                                                   std::shared_ptr<typename TrajectoryOptimization<T>::LO_param> lo_param_ptr,
                                                   std::shared_ptr<typename TrajectoryOptimization<T>::TD_param> td_param_ptr)
{
  op_param_ptr_ = op_param_ptr;
  lo_param_ptr_ = lo_param_ptr;
  td_param_ptr_ = td_param_ptr;
}

template class BezierTrajectory<double>;
template class BezierTrajectory<float>;
