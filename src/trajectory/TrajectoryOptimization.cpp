#include "TrajectoryOptimization.hpp"
#include "orientation_tools.hpp"

#include <iostream>

using namespace std;
using namespace ori;
using namespace Eigen;
using namespace casadi;

template <typename T>
TrajectoryOptimization<T>::TrajectoryOptimization(RobotLeg<T> & robot) : robot_(robot)
{
  optimization_limit_ = inf;

  foot_traj_ptr_ = nullptr;

  lo_param_ptr_ = std::make_shared<LO_param>();
  td_param_ptr_ = std::make_shared<TD_param>();
  op_param_ptr_ = std::make_shared<Optimization_param>();
}


template <typename T>
void TrajectoryOptimization<T>::Flight_traj_generate(mjData * d)
{
  /**
   * @brief Flight Trajectory를 생성하는 곳
   * @brief 여기서는 Desired Touch Down state를 먼저 생성하고 그 다음에 Radial, Angular Optimization을 하고
   * @brief 마지막으로 Polynomial Trajectory를 생성한다.
   * @param state_update() : TD,LO 상태의 변수들을 받아서 여기서 쓰게끔 setting 해준다
   * @param Desired_Touch_Down_state() : Desired Touch Down state를 계산하는 함수
   * @param Desired_Flight_Time() : Desired Flight Time을 계산하는 함수
   * @param Radial_Optimization() : Radial Optimization을 하는 함수
   * @param Angular_Optimization() : Angular Optimization을 하는 함수
   * @param Polynomial_Trajectory() : Polynomial Trajectory를 생성하는 함수 -> Flight phase Traj가 출력된다
   */

  time = d->time;
  for (size_t i = 0; i < 4; i++)
  {
    if (robot_.event_[i] == 4)
    {
      state_update(i);
      Desired_Touch_Down_state(i);
      Desired_Flight_Time(i);

      Radial_Optimization(i);
      Angular_Optimization(i);
    };

    if (robot_.phase_[i] == 2)
    {
      Polynomial_Trajectory(i);
    };
  }

}

template <typename T>
void TrajectoryOptimization<T>::state_update(int Leg_num)
{
  lo_param_ptr_->th_LO[Leg_num] -= M_PI/2;
  td_param_ptr_->th_TD[Leg_num] -= M_PI/2;

  r_ref[Leg_num] = foot_traj_ptr_->foot_pos_rw_des_[Leg_num][0];
  dth_ref[Leg_num] = foot_traj_ptr_->foot_vel_rw_des_[Leg_num][1];
}


template <typename T>
void TrajectoryOptimization<T>::Desired_Touch_Down_state(int Leg_num)
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

  // T st_time=2*(-(th_TD)/dth_ref);
  T st_time = lo_param_ptr_->t_stance[Leg_num];

  T th_r = (2 * dth_ref[Leg_num]) * (((e_2 -e_1) + (e_1*exp(e_1*st_time) - e_2*exp(e_2*st_time))+
  a*(exp(e_1*st_time) - exp(e_2 * st_time)))/(e_1*e_2*(exp(e_1*st_time)-exp(e_2*st_time))));
  // T th_r = 0;

  // desired top
  op_param_ptr_->r_des_top[Leg_num] = 0.35;
  op_param_ptr_->th_des_top[Leg_num] = 0;

  op_param_ptr_-> h_1[Leg_num] = 1.2;
  op_param_ptr_-> u[Leg_num] = op_param_ptr_-> h_1[Leg_num] * ((-th_r/2)- lo_param_ptr_->th_LO[Leg_num]) +
    th_r;


  op_param_ptr_->r_des_TD[Leg_num] = r_ref[Leg_num];
  op_param_ptr_->th_des_TD[Leg_num] = lo_param_ptr_->th_LO[Leg_num] + op_param_ptr_->u[Leg_num];

  T t_b = st_time/2;

  op_param_ptr_->dr_des_TD[Leg_num] = (alpha*(m*g/k)*tan(alpha*t_b));
  op_param_ptr_->dth_des_TD[Leg_num] = dth_ref[Leg_num];

  op_param_ptr_->ddr_des_TD[Leg_num] = -g;

}

template <typename T>
void TrajectoryOptimization<T>::Desired_Flight_Time(int Leg_num)
{
  /**
   * @brief Desried Flight Time을 계산하는 곳
   * @brief 여기 input은 Desired Touch Down state에서 계산한 값들을 가져와서 사용한다.
   */
  T g = 9.81;
  op_param_ptr_->t_flight_des[Leg_num] = 2*lo_param_ptr_->V_y_LO[Leg_num]/g;

}

template <typename T>
void TrajectoryOptimization<T>::Radial_Optimization(int Leg_num)
{
  /**
   * @brief Radial Optimization을 하는 곳
   * */
  int n_r = 9;

  // setting time
  SX t_top = op_param_ptr_->t_flight_des[Leg_num]/2;
  SX t_des_TD = op_param_ptr_->t_flight_des[Leg_num];
  SX t_1 = op_param_ptr_->t_flight_des[Leg_num]/4;
  SX t_2 = 3*op_param_ptr_->t_flight_des[Leg_num]/4;

  SX A = SX::sym("A", n_r);

  vector<SX> constraints;
  // r(t_LO) = r_LO
  constraints.push_back(A(8));
  // dr(t_LO) = dr_LO
  constraints.push_back(A(7));
  // r(t_TD) = r_des_TD
  constraints.push_back(pow(t_1,8)*A(0) + pow(t_1,7)*A(1) + pow(t_1,6)*A(2) + pow(t_1,5)*A(3) + pow(t_1,4)*A(4) + pow(t_1,3)*A(5) + pow(t_1,2)*A(6) + t_1*A(7) + A(8));
  constraints.push_back(pow(t_des_TD, 8)*A(0) + pow(t_des_TD, 7)*A(1) + pow(t_des_TD, 6)*A(2) + pow(t_des_TD, 5)*A(3) + pow(t_des_TD, 4)*A(4) + pow(t_des_TD, 3)*A(5) + pow(t_des_TD, 2)*A(6) + t_des_TD*A(7) + A(8));
  // dr(t_TD) = dr_des_TD
  constraints.push_back(8*pow(t_des_TD, 7)*A(0) + 7*pow(t_des_TD, 6)*A(1) + 6*pow(t_des_TD, 5)*A(2) + 5*pow(t_des_TD, 4)*A(3) + 4*pow(t_des_TD, 3)*A(4) + 3*pow(t_des_TD, 2)*A(5) + 2*t_des_TD*A(6) + A(7));
  // % -100<dr(t_1)<0;
  constraints.push_back(8*pow(t_1, 7)*A(0) + 7*pow(t_1, 6)*A(1) + 6*pow(t_1, 5)*A(2) + 5*pow(t_1, 4)*A(3) + 4*pow(t_1, 3)*A(4) + 3*pow(t_1, 2)*A(5) + 2*t_1*A(6) + A(7));
  // %  0<dr(t_1)<100;
  constraints.push_back(8*pow(t_2, 7)*A(0) + 7*pow(t_2, 6)*A(1) + 6*pow(t_2, 5)*A(2) + 5*pow(t_2, 4)*A(3) + 4*pow(t_2, 3)*A(4) + 3*pow(t_2, 2)*A(5) + 2*t_2*A(6) + A(7));
  // 0< r(t_TD) <0.4
  constraints.push_back(pow(t_top,8)*A(0) + pow(t_top,7)*A(1) + pow(t_top,6)*A(2) + pow(t_top,5)*A(3) + pow(t_top,4)*A(4) + pow(t_top,3)*A(5) + pow(t_top,2)*A(6) + t_top*A(7) + A(8));
  // dr(t_top) = 0 -> 이거 왜 이렇게 표현함? 고쳐야함
  constraints.push_back(8*pow(t_top, 7)*A(0) + 7*pow(t_top, 6)*A(1) + 6*pow(t_top, 5)*A(2) + 5*pow(t_top, 4)*A(3) + 4*pow(t_top, 3)*A(4) + 3*pow(t_top, 2)*A(5) + 2*t_top*A(6) + A(7));
  //constraints.push_back(0);
  // -10 < ddr(t_TD) = ddr_des_TD
  constraints.push_back(56*pow(t_des_TD, 6)*A(0) + 42*pow(t_des_TD, 5)*A(1) + 30*pow(t_des_TD, 4)*A(2) + 20*pow(t_des_TD, 3)*A(3) + 12*pow(t_des_TD, 2)*A(4) + 6*t_des_TD*A(5) + 2*A(6));
  // %
  // 0< r(t_TD) <0.4
  constraints.push_back(pow(t_2,8)*A(0) + pow(t_2,7)*A(1) + pow(t_2,6)*A(2) + pow(t_2,5)*A(3) + pow(t_2,4)*A(4) + pow(t_2,3)*A(5) + pow(t_2,2)*A(6) + t_2*A(7) + A(8));


  // 제약 조건 정의
  SX g = vertcat(constraints);

  // 목적 함수 정의
  SX r_top_obj = pow(t_top,8)*A(0) + pow(t_top,7)*A(1) + pow(t_top,6)*A(2) + pow(t_top,5)*A(3) + pow(t_top,4)*A(4) + pow(t_top,3)*A(5) + pow(t_top,2)*A(6) + t_top*A(7) + A(8);
  SX f = pow(op_param_ptr_->r_des_top[Leg_num] - r_top_obj,2); // th_des_top은 앞에서 다른 함수에서 정해짐

  // Setting Ipopt options
  Dict ipopt_opts;
  ipopt_opts["linear_solver"] = "ma27";              // Set a valid linear solver option
  ipopt_opts["max_iter"] = 500;                      // Set maximum number of iterations
  ipopt_opts["tol"] = 1e-6;                          // Convergence tolerance
  ipopt_opts["print_level"] = 0;                     // Verbosity level of the solver output
  ipopt_opts["print_timing_statistics"] = "no";      // disable timing statistics
  ipopt_opts["print_options_documentation"] = "no";  // Suppress documentation
  // ipopt_opts["ma27_print_level"] = 0;
  ipopt_opts["acceptable_tol"] = 1e-5;  // Acceptable convergence tolerance
  ipopt_opts["sb"] = "no";             // short banner for startup
  // ipopt_opts["file_print_level"] = 0;            // suppress file output
  Dict nlp_opts;
  nlp_opts["ipopt"] = ipopt_opts;

  Function nlp = nlpsol("nlp", "ipopt", {{"x", A}, {"f", f}, {"g", g}}, nlp_opts);
  // 최적화 문제 설정
  //! x는 최적화 변수, f는 목적함수, g는 제약조건

  // 초기 추정값 및 변수의 경계 설정--
  // ! 여기는 변수의 초기값 및 상한 및 하한을 정해준다
  DM A0 = DM::zeros(n_r, 1);
  DM lbx = DM::ones(n_r) * -optimization_limit_; // 변수 하한
  DM ubx = DM::ones(n_r) * optimization_limit_; // 변수 상한

  // cout << "r_LO : " << lo_param_ptr_->r_LO[1] << endl;

  // ! constraint의 변수의 상한 및 하한을 정해준다.
  DM l_r = DM::vertcat({lo_param_ptr_->r_LO[Leg_num], 0,  0.38, op_param_ptr_->r_des_TD[Leg_num] ,
    op_param_ptr_->dr_des_TD[Leg_num], -100, -100,0,  0,  -10      ,  0.35}); // 제약 조건 순서에 맞게 설정
  DM u_r = DM::vertcat({lo_param_ptr_->r_LO[Leg_num], 0,   0.4, op_param_ptr_->r_des_TD[Leg_num] ,
    op_param_ptr_->dr_des_TD[Leg_num], 100, 100,0.3, 0,  op_param_ptr_->ddr_des_TD[Leg_num],  0.35}); // 제약 조건 순서에 맞게 설정

  // 문제 해결
  std::map<std::string, DM> arg;
  arg["x0"] = A0;
  arg["lbx"] = lbx;
  arg["ubx"] = ubx;
  arg["lbg"] = l_r;
  arg["ubg"] = u_r;

  map<std::string, DM> res = nlp(arg);

  // Extract the optimized solution
  vector<double> solution(res.at("x"));
  op_param_ptr_->X_r[Leg_num] = VectorXd::Map(solution.data(), solution.size());

}

template <typename T>
void TrajectoryOptimization<T>::Angular_Optimization(int Leg_num)
{
  /**
   * @brief Angular Optimization을 하는 곳
   */

  int n_th = 6;
  SX t_top = op_param_ptr_->t_flight_des[Leg_num]/2;
  SX t_des_TD = op_param_ptr_->t_flight_des[Leg_num];

  //최적화 변수 정의
  SX B = SX::sym("B", n_th); // B(0) ~ B(5) Beta

  std::vector<SX> constraints;
  constraints.push_back(B(5)); // theta(t_LO) = theta_LO
  constraints.push_back(B(4)); // dtheta(t_LO) = dtheta_LO
  constraints.push_back(pow(t_des_TD, 5)*B(0) + pow(t_des_TD, 4)*B(1) + pow(t_des_TD, 3)*B(2) + pow(t_des_TD, 2)*B(3) + t_des_TD*B(4) + B(5)); // theta(t_TD) = theta_des_TD
  constraints.push_back(5*pow(t_des_TD, 4)*B(0) + 4*pow(t_des_TD, 3)*B(1) + 3*pow(t_des_TD, 2)*B(2) + 2*t_des_TD*B(3) + B(4)); // dtheta(t_TD) = dtheta_des_TD


  SX g = vertcat(constraints);

  // 목적 함수 정의
  SX th_top_obj = pow(t_top, 5)*B(0) + pow(t_top, 4)*B(1) + pow(t_top, 3)*B(2) + pow(t_top, 2)*B(3) + t_top*B(4) + B(5);
  SX f = pow(op_param_ptr_->th_des_top[Leg_num] - th_top_obj,2); // th_des_top은 앞에서 다른 함수에서 정해짐

  // 최적화 문제 설정
  //! x는 최적화 변수, f는 목적함수, g는 제약조건
  // Setting Ipopt options
  Dict ipopt_opts;
  ipopt_opts["linear_solver"] = "ma27";              // Set a valid linear solver option
  ipopt_opts["max_iter"] = 500;                      // Set maximum number of iterations
  ipopt_opts["tol"] = 1e-6;                          // Convergence tolerance
  ipopt_opts["print_level"] = 0;                     // Verbosity level of the solver output
  ipopt_opts["print_timing_statistics"] = "no";      // disable timing statistics
  ipopt_opts["print_options_documentation"] = "no";  // Suppress documentation
  // ipopt_opts["ma27_print_level"] = 0;
  ipopt_opts["acceptable_tol"] = 1e-5;  // Acceptable convergence tolerance
  ipopt_opts["sb"] = "yes";             // short banner for startup
  // ipopt_opts["file_print_level"] = 0;            // suppress file output
  Dict nlp_opts;
  nlp_opts["ipopt"] = ipopt_opts;

  Function nlp = nlpsol("nlp", "ipopt", {{"x", B}, {"f", f}, {"g", g}}, nlp_opts);

  DM B0 = DM::zeros(n_th, 1);
  DM lbx = DM::ones(n_th) * -optimization_limit_; // 변수 하한
  DM ubx = DM::ones(n_th) * optimization_limit_; // 변수 상한

  DM l_th = DM::vertcat({ lo_param_ptr_->th_LO[Leg_num], lo_param_ptr_->dth_LO[Leg_num],
    op_param_ptr_->th_des_TD[Leg_num], op_param_ptr_->dth_des_TD[Leg_num]});

  DM u_th = DM::vertcat({ lo_param_ptr_->th_LO[Leg_num], lo_param_ptr_->dth_LO[Leg_num],
    op_param_ptr_->th_des_TD[Leg_num], op_param_ptr_->dth_des_TD[Leg_num]});

  // 문제 해결
  // ! x0,lbx,ubx,lth,uth는 꼭 지켜주도록
  std::map<std::string, DM> arg;
  arg["x0"] = DM(B0);
  arg["lbx"] = DM(lbx);
  arg["ubx"] = DM(ubx);
  arg["lbg"] = DM(l_th);
  arg["ubg"] = DM(u_th);

  std::map<std::string, DM> res = nlp(arg);

  // 최적화된 해 추출
  std::vector<double> solution = std::vector<double>(res.at("x"));

  op_param_ptr_->X_th[Leg_num] = VectorXd::Map(solution.data(), solution.size());
}

template <typename T>
void TrajectoryOptimization<T>::Polynomial_Trajectory(int Leg_num)
{
  /**
   * @brief Polynomial Trajectory를 생성pow
   * todo: Time을 넣어줘야함
   */

  T optimized_flight_time =  time - lo_param_ptr_->t_LO[Leg_num];
  T t_des_TD = op_param_ptr_->t_flight_des[Leg_num];

  //******************************** Early TD *************************************/
  if (optimized_flight_time <= t_des_TD)
  {
    op_param_ptr_->t_r[Leg_num] = optimized_flight_time;
    op_param_ptr_->t_th[Leg_num] = optimized_flight_time;

    op_param_ptr_->T_r[Leg_num].resize(9);
    op_param_ptr_->T_dr[Leg_num].resize(9);
    op_param_ptr_->T_th[Leg_num].resize(6);
    op_param_ptr_->T_dth[Leg_num].resize(6);

    op_param_ptr_->T_r[Leg_num] << pow(op_param_ptr_->t_r[Leg_num], 8), pow(op_param_ptr_->t_r[Leg_num], 7),
     pow(op_param_ptr_->t_r[Leg_num], 6), pow(op_param_ptr_->t_r[Leg_num], 5), pow(op_param_ptr_->t_r[Leg_num], 4),
           pow(op_param_ptr_->t_r[Leg_num], 3), pow(op_param_ptr_->t_r[Leg_num], 2), op_param_ptr_->t_r[Leg_num], 1;

    op_param_ptr_->T_dr[Leg_num] << 8*pow(op_param_ptr_->t_r[Leg_num], 7), 7*pow(op_param_ptr_->t_r[Leg_num], 6),
      6*pow(op_param_ptr_->t_r[Leg_num], 5), 5*pow(op_param_ptr_->t_r[Leg_num], 4), 4*pow(op_param_ptr_->t_r[Leg_num], 3),
            3*pow(op_param_ptr_->t_r[Leg_num], 2), 2*op_param_ptr_->t_r[Leg_num], 1, 0;

    op_param_ptr_->T_th[Leg_num] << pow(op_param_ptr_->t_th[Leg_num], 5), pow(op_param_ptr_->t_th[Leg_num], 4),
      pow(op_param_ptr_->t_th[Leg_num], 3), pow(op_param_ptr_->t_th[Leg_num], 2), op_param_ptr_->t_th[Leg_num], 1;

    op_param_ptr_->T_dth[Leg_num] << 5*pow(op_param_ptr_->t_th[Leg_num], 4), 4*pow(op_param_ptr_->t_th[Leg_num], 3),
      3*pow(op_param_ptr_->t_th[Leg_num], 2), 2*op_param_ptr_->t_th[Leg_num], 1, 0;

    op_param_ptr_->r_optimized_flight[Leg_num] = op_param_ptr_->T_r[Leg_num].transpose() * op_param_ptr_->X_r[Leg_num];
    op_param_ptr_->dr_optimized_flight[Leg_num] = op_param_ptr_->T_dr[Leg_num].transpose() * op_param_ptr_->X_r[Leg_num];

    op_param_ptr_->th_optimized_flight[Leg_num] = op_param_ptr_->T_th[Leg_num].transpose() * op_param_ptr_->X_th[Leg_num];
    op_param_ptr_->dth_optimized_flight[Leg_num] = op_param_ptr_->T_dth[Leg_num].transpose() * op_param_ptr_->X_th[Leg_num];

  }
  //*********************************************** late TD *******************************************************************/
  else if (optimized_flight_time > t_des_TD)
  {

    op_param_ptr_->t_r[Leg_num] = optimized_flight_time - t_des_TD;
    op_param_ptr_->t_th[Leg_num] = t_des_TD;

    op_param_ptr_->T_r[Leg_num].resize(2);
    op_param_ptr_->T_dr[Leg_num].resize(2);
    op_param_ptr_->T_th[Leg_num].resize(6);
    op_param_ptr_->T_dth[Leg_num].resize(6);
    op_param_ptr_->X_r[Leg_num].resize(2);

    op_param_ptr_->T_r[Leg_num] << 0, 1;
    op_param_ptr_->T_dr[Leg_num] << 1, 0;

    op_param_ptr_->T_th[Leg_num] << pow(op_param_ptr_->t_th[Leg_num], 5), pow(op_param_ptr_->t_th[Leg_num], 4),
      pow(op_param_ptr_->t_th[Leg_num], 3), pow(op_param_ptr_->t_th[Leg_num], 2), op_param_ptr_->t_th[Leg_num], 1;
    op_param_ptr_->T_dth[Leg_num] << 0, 0, 0, 0, 0, 0;

    op_param_ptr_->X_r[Leg_num] << op_param_ptr_->dr_des_TD[Leg_num], op_param_ptr_->r_des_TD[Leg_num];

    op_param_ptr_->r_optimized_flight[Leg_num] = op_param_ptr_->T_r[Leg_num].transpose() * op_param_ptr_->X_r[Leg_num];
    op_param_ptr_->dr_optimized_flight[Leg_num] = op_param_ptr_->T_dr[Leg_num].transpose() * op_param_ptr_->X_r[Leg_num];

    op_param_ptr_->th_optimized_flight[Leg_num] = op_param_ptr_->T_th[Leg_num].transpose() * op_param_ptr_->X_th[Leg_num];
    op_param_ptr_->dth_optimized_flight[Leg_num] = op_param_ptr_->T_dth[Leg_num].transpose() * op_param_ptr_->X_th[Leg_num];
  }
  //*********************************************** ELSE ************************************************************************* */
  else
  {

    op_param_ptr_->t_r[Leg_num] = 0;
    op_param_ptr_->t_th[Leg_num] = 0;

    op_param_ptr_->r_optimized_flight[Leg_num] = td_param_ptr_->r_TD[Leg_num];
    op_param_ptr_->dr_optimized_flight[Leg_num] = op_param_ptr_->dr_des_TD[Leg_num];

    op_param_ptr_->th_optimized_flight[Leg_num] = td_param_ptr_->th_TD[Leg_num];
    op_param_ptr_->dth_optimized_flight[Leg_num] = td_param_ptr_->dth_TD[Leg_num];
  }

  foot_traj_ptr_->r_optimized_flight_[Leg_num] = op_param_ptr_->r_optimized_flight[Leg_num];
  foot_traj_ptr_->dr_optimized_flight_[Leg_num] = op_param_ptr_->dr_optimized_flight[Leg_num];

  foot_traj_ptr_->th_optimized_flight_[Leg_num] = op_param_ptr_->th_optimized_flight[Leg_num] + M_PI/2;
  foot_traj_ptr_->dth_optimized_flight_[Leg_num] = op_param_ptr_->dth_optimized_flight[Leg_num];
  //cout << "r_optimized_flight : " << lo_param_ptr_->t_LO[2] << endl;
}




template <typename T>
std::shared_ptr<typename TrajectoryOptimization<T>::LO_param> TrajectoryOptimization<T>::get_lo_param_ptr()
{
  return lo_param_ptr_;
}

template <typename T>
std::shared_ptr<typename TrajectoryOptimization<T>::TD_param> TrajectoryOptimization<T>::get_td_param_ptr()
{
  return td_param_ptr_;
}

template <typename T>
std::shared_ptr<typename TrajectoryOptimization<T>::Optimization_param> TrajectoryOptimization<T>::get_op_param_ptr()
{
  return op_param_ptr_;
}

template <typename T>
void TrajectoryOptimization<T>::get_traj_pointer(std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr)
{
  foot_traj_ptr_ = foot_traj_ptr;
}




template class TrajectoryOptimization<float>;
template class TrajectoryOptimization<double>;
