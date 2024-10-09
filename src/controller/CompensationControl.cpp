#include "CompensationControl.hpp"
#include "orientation_tools.hpp"

#include <iostream>

using namespace ori;
using namespace std;

template <typename T>
CompensationControl<T>::CompensationControl(RobotLeg<T> & robot) : robot_(robot)
{
  /**
   * @brief Parameter for Compensation Control
   * ! Parameter is time independent. so I Just using FL leg value
   */
  g = -9.81;
  pi = 3.14159265358979323846;
  sampling_time = 0.001;

  m1 = robot_.thigh_mass_[0];
  m2 = robot_.shank_mass_[0];
  l = robot_.thigh_link_length_[0];
  I1_zz = robot_.thigh_body_inertia_[0][2];
  I2_zz = robot_.shank_body_inertia_[0][2];
  l1_c = robot_.thigh_com_location_[0][0];
  l2_c = robot_.shank_com_location_[0][0];
  M_d = robot_.M_d_R;

  G_m = g*(m1*l1_c + m2*l);
  G_b = g*(m2*l2_c);

  C_I = m2*l2_c*l;

  I_m = I1_zz + m2*l*l+m1*l1_c*l1_c;
  I_b = I2_zz + m2*l2_c*l2_c;

  // Calculate for motor accelelration
  for (int i = 0; i < 4; i++)
  {
    th_m[i] = 0;
    th_b[i] = 0;

    dth_m[i] = 0;
    dth_b[i] = 0;
    dth_br[i] = 0;
    ddth_m[i] = 0;
    ddth_b[i] = 0;

    dth_m_old[i] = 0;
    dth_b_old[i] = 0;
    ddth_m_old[i] = 0;
    ddth_b_old[i] = 0;

    gravity_compensation_joint_des_[i].setZero();
    coriollis_compensation_joint_des_[i].setZero();
    inertia_decoupling_joint_des_[i].setZero();
    inertia_modulation_joint_des_[i].setZero();
    compensation_joint_des_[i].setZero();

  }
}

template <typename T>
void CompensationControl<T>::state_update(int Leg_num)
{

  dth_m_old[Leg_num] = dth_m[Leg_num];
  dth_b_old[Leg_num] = dth_b[Leg_num];
  ddth_m_old[Leg_num] = ddth_m[Leg_num];
  ddth_b_old[Leg_num] = ddth_b[Leg_num];

  th_m[Leg_num] = robot_.joint_pos_bi_act_[Leg_num][0];
  th_b[Leg_num] = robot_.joint_pos_bi_act_[Leg_num][1];
  th_br[Leg_num] = robot_.joint_pos_act_[Leg_num][2];

  dth_m[Leg_num] = robot_.joint_vel_bi_act_[Leg_num][0];
  dth_b[Leg_num] = robot_.joint_vel_bi_act_[Leg_num][1];
  dth_br[Leg_num] = robot_.joint_vel_act_[Leg_num][2];

  ddth_m[Leg_num] = tustin_derivative(dth_m[Leg_num], dth_m_old[Leg_num], ddth_m[Leg_num], ddth_m_old[Leg_num], 15);
  ddth_b[Leg_num] = tustin_derivative(dth_b[Leg_num], dth_b_old[Leg_num], ddth_b[Leg_num], ddth_b_old[Leg_num], 15);

}

template <typename T>
void CompensationControl<T>::Gravity_compensation(int Leg_num)
{
  /**
  * @brief Gravity compensation
  * @param G_m: mass of the mono
  * @param G_b: Mass of the bi
  *
  */


  gravity_compensation_joint_des_[Leg_num] << G_m*cos(th_m[Leg_num]), G_b*cos(th_b[Leg_num]);

}

template <typename T>
void CompensationControl<T>::Coriollis_compensation(int Leg_num)
{
  /**
  * @brief Coriollis compensation
  * @param C_I: Inertia of the bi
  *
  */


  coriollis_compensation_joint_des_[Leg_num] << -C_I*sin(th_br[Leg_num])*pow(dth_b[Leg_num],2),
                                            C_I*sin(th_br[Leg_num])*pow(dth_m[Leg_num],2);

}

template <typename T>
void CompensationControl<T>::Inertia_Decoupling(int Leg_num)
{
  /**
  * @brief Inertia Decoupling compensation
  */


  inertia_decoupling_joint_des_[Leg_num] << C_I*cos(th_br[Leg_num])*ddth_b[Leg_num],
                                        C_I*cos(th_br[Leg_num])*ddth_m[Leg_num];
}

template <typename T>
void CompensationControl<T>::Inertia_modulation(int Leg_num)
{
  /**
  * @brief Inertia modulation compensation
  */

  inertia_modulation_joint_des_[Leg_num] << I_m-M_d*pow(l,2)*(1-cos(0))*ddth_m[Leg_num],
                                            I_b-M_d*pow(l,2)*(1-cos(0))*ddth_b[Leg_num];


}

template <typename T>
void CompensationControl<T>::Trunk_mass_compensation(mjData * d)
{
  /**
   * @brief Trunk mass compensation
   * todo: Trunk Mass compensation 할 때 contact 되는 다리가 달라지면  compensatio이 잘 안될텐데
   * @param i=0,1,2,3 (FL,FR,RL,RR)
   * *r_grf가 어떤 조건에도 만족하지 않으면 0이 되어야 하기 때문에 0으로 초기화 해야함
   *
   */

  Vec2<T> result[2];

  body_com << d->subtree_com[0], d->subtree_com[1], d->subtree_com[2];

  body_weight << 9.81*43, 0;

  for (int i = 0; i < 4; i++)
  {
    r_grf[i] << 0, 0;
    foot_pos[i] << d->site_xpos[6+6*i], d->site_xpos[6+6*i + 1], d->site_xpos[6+6*i + 2];
    vec_body2foot[i] = foot_pos[i] - body_com;
  }

  if(fmod(d->time, 0.5) == 0)
  {
    // cout << d->subtree_com[0] << endl;
    // cout << d->subtree_com[1] << endl;
    // cout << d->subtree_com[2] << endl;
    // cout << foot_pos[1] << endl;
    // cout << foot_pos[2] << endl;
    // cout << foot_pos[3] << endl;
  }
  // cout <<  << endl;
  // cout << "0 : "<< foot_pos[0][0]<<", "<< foot_pos[0][1]<< ", "<< foot_pos[0][2] << endl;
  // cout << "1 : "<< foot_pos[1][0]<<", "<< foot_pos[1][1]<< ", "<< foot_pos[1][2] << endl;
  // cout << "2 : "<< foot_pos[2][0]<<", "<< foot_pos[2][1]<< ", "<< foot_pos[2][2] << endl;
  // cout << "3 : "<< foot_pos[3][0]<<", "<< foot_pos[3][1]<< ", "<< foot_pos[3][2] << endl;

  cal_Mat[0] << 1, 1, vec_body2foot[0][0], vec_body2foot[3][0];
  cal_Mat[1] << 1, 1, vec_body2foot[1][0], vec_body2foot[2][0];
  robot_.phase_[2] = 1;
  robot_.phase_[3] = 1;

  // cout << "phase_0 "<< robot_.phase_[0] << endl;
  // cout << "phase_1 "<< robot_.phase_[1] << endl;
  // cout << "phase_2 "<< robot_.phase_[2] << endl;
  // cout << "phase_3 "<< robot_.phase_[3] << endl;
  //* 4점 지지
  if (robot_.phase_[0] == 1 && robot_.phase_[1] == 1 &&
      robot_.phase_[2] == 1 && robot_.phase_[3] == 1)
  {

    result[0] = cal_Mat[0].inverse()*body_weight/2;
    result[1] = cal_Mat[1].inverse()*body_weight/2;

    r_grf[0] << result[0][0], 0;
    r_grf[1] << result[1][0], 0;
    r_grf[2] << result[1][1], 0;
    r_grf[3] << result[0][1], 0;
    // cout<< "1" << endl;
  }
  else if (robot_.phase_[0] == 1 && robot_.phase_[3] == 1)
  {
    //* 2점 지지
    result[0] = cal_Mat[0].inverse()*body_weight;
    r_grf[0] << result[0][0], 0;
    r_grf[3] << result[0][1], 0;

    r_grf[1] << 0, 0;
    r_grf[2] << 0, 0;
    // cout<< "2" << endl;
  }
  else if (robot_.phase_[1]==1 && robot_.phase_[2] == 1)
  {
    result[1] = cal_Mat[1].inverse()*body_weight;
    r_grf[1] << result[1][0], 0;
    r_grf[2] << result[1][1], 0;

    r_grf[0] << 0, 0;
    r_grf[3] << 0, 0;
  }

  robot_.phase_[2] == 0;
  robot_.phase_[3] == 0;

  //* For Debugging
  // result[0] = cal_Mat[0].inverse()*body_weight/2;
  // result[1] = cal_Mat[1].inverse()*body_weight/2;

  // r_grf[0] << result[0][0], 0;
  // r_grf[1] << result[1][0], 0;
  // r_grf[2] << result[1][1], 0;
  // r_grf[3] << result[0][1], 0;

  // cout << "0 : "<< r_grf[0][0]<< endl;
  // cout << "1 : "<< r_grf[1][0]<< endl;
  // cout << "2 : "<< r_grf[2][0]<< endl;
  // cout << "3 : "<< r_grf[3][0]<< endl;

  for (int i = 0; i < 4; i++)
  {
    //* Flight phase는 무조건 0으로 들어가게끔 setting
    // if (robot_.phase_[i] == 2)
    // {
    //   r_grf[i] << 0, 0;
    // }

    Trunk_mass_compensation_joint_des_[i] = robot_.jacbRW[i].transpose() * r_grf[i]/cos(M_PI/2-robot_.joint_pos_bi_act_[i][1]);
    // Trunk_mass_compensation_joint_des_[i] = robot_.jacbRW[i].transpose() * r_grf[i];
    robot_.joint_torque_des_[i][1] += Trunk_mass_compensation_joint_des_[i][0] + Trunk_mass_compensation_joint_des_[i][1];
    robot_.joint_torque_des_[i][2] += Trunk_mass_compensation_joint_des_[i][1];

  }



}

template <typename T>
void CompensationControl<T>::compensation_control(int Leg_num)
{
  /**
   * @brief Compensation Control
   * @param gravity_compensation_joint_des_: desired gravity compensation
   * @param coriollis_compensation_joint_des_: desired coriollis compensation
   * @param inertia_decoupling_joint_des_: desired inertia decoupling compensation
   * @param inertia_modulation_joint_des_: desired inertia modulation compensation
   */

  state_update(Leg_num);

  Gravity_compensation(Leg_num);
  Coriollis_compensation(Leg_num);
  Inertia_Decoupling(Leg_num);
  Inertia_modulation(Leg_num);


  compensation_joint_des_[Leg_num] = gravity_compensation_joint_des_[Leg_num] + coriollis_compensation_joint_des_[Leg_num] +
    inertia_decoupling_joint_des_[Leg_num] + inertia_modulation_joint_des_[Leg_num];




  robot_.joint_torque_des_[Leg_num][1] = robot_.joint_torque_des_[Leg_num][1] + compensation_joint_des_[Leg_num][0] + compensation_joint_des_[Leg_num][1];
  robot_.joint_torque_des_[Leg_num][2] = robot_.joint_torque_des_[Leg_num][2] + compensation_joint_des_[Leg_num][1];


}







template <typename T>
T CompensationControl<T>::tustin_derivative(T input, T input_old, T output,T output_old, T cut_off)
{
  /**
  * @brief Tustin derivative filter
  */

  T Ts = sampling_time;
  T time_const = 1 / (2 * pi * cut_off);
  output = 0;

  output = (2 * (input - input_old) - (Ts - 2 * time_const) * output_old) / (Ts + 2 * time_const);
  return output;

}


template class CompensationControl<float>;
template class CompensationControl<double>;

