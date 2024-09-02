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
  // g =  0;
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
void CompensationControl<T>::state_update()
{
  for (int i = 0; i < 4; i++)
  {
    dth_m_old[i] = dth_m[i];
    dth_b_old[i] = dth_b[i];
    ddth_m_old[i] = ddth_m[i];
    ddth_b_old[i] = ddth_b[i];

    th_m[i] = robot_.joint_pos_bi_act_[i][0];
    th_b[i] = robot_.joint_pos_bi_act_[i][1];
    th_br[i] = robot_.joint_pos_act_[i][2];

    dth_m[i] = robot_.joint_vel_bi_act_[i][0];
    dth_b[i] = robot_.joint_vel_bi_act_[i][1];
    dth_br[i] = robot_.joint_vel_act_[i][2];

    ddth_m[i] = tustin_derivative(dth_m[i], dth_m_old[i], ddth_m[i], ddth_m_old[i], 15);
    ddth_b[i] = tustin_derivative(dth_b[i], dth_b_old[i], ddth_b[i], ddth_b_old[i], 15);
  }
}

template <typename T>
void CompensationControl<T>::Gravity_compensation()
{
  /**
  * @brief Gravity compensation
  * @param G_m: mass of the mono
  * @param G_b: Mass of the bi
  *
  */

  for (int i = 0; i < 4; i++)
  {
    gravity_compensation_joint_des_[i] << G_m*cos(th_m[i]), G_b*cos(th_b[i]);
  }
}

template <typename T>
void CompensationControl<T>::Coriollis_compensation()
{
  /**
  * @brief Coriollis compensation
  * @param C_I: Inertia of the bi
  *
  */

  for (int i = 0; i < 4; i++)
  {
    coriollis_compensation_joint_des_[i] << -C_I*sin(th_br[i])*pow(dth_b[i],2),
                                            C_I*sin(th_br[i])*pow(dth_m[i],2);
  }
}

template <typename T>
void CompensationControl<T>::Inertia_Decoupling()
{
  /**
  * @brief Inertia Decoupling compensation
  */

  for (int i = 0; i < 4; i++)
  {
    inertia_decoupling_joint_des_[i] << C_I*cos(th_br[i])*ddth_b[i],
                                        C_I*cos(th_br[i])*ddth_m[i];
  }
}

template <typename T>
void CompensationControl<T>::Inertia_modulation()
{
  /**
  * @brief Inertia modulation compensation
  */
  for (int i = 0; i < 4; i++)
  {
    inertia_modulation_joint_des_[i] << I_m-M_d*pow(l,2)*(1-cos(0))*ddth_m[i],
                                        I_b-M_d*pow(l,2)*(1-cos(0))*ddth_b[i];



  }

}

template <typename T>
void CompensationControl<T>::Trunk_mass_compensation()
{
  /**
   * @brief Trunk mass compensation
   * todo: Trunk Mass compensation 할 때 contact 되는 다리가 달라지면  compensatio이 잘 안될텐데
   *! check 해보기
   */

}

template <typename T>
void CompensationControl<T>::compensation_control()
{
  /**
   * @brief Compensation Control
   * @param gravity_compensation_joint_des_: desired gravity compensation
   * @param coriollis_compensation_joint_des_: desired coriollis compensation
   * @param inertia_decoupling_joint_des_: desired inertia decoupling compensation
   * @param inertia_modulation_joint_des_: desired inertia modulation compensation
   */

  state_update();

  Gravity_compensation();
  Coriollis_compensation();
  Inertia_Decoupling();
  Inertia_modulation();

  for (int i = 0; i < 4; i++)
  {
    // compensation_joint_des_[i] = gravity_compensation_joint_des_[i] + coriollis_compensation_joint_des_[i] +
                                    // inertia_decoupling_joint_des_[i] + inertia_modulation_joint_des_[i];
    compensation_joint_des_[i] = gravity_compensation_joint_des_[i] + coriollis_compensation_joint_des_[i] +
     inertia_decoupling_joint_des_[i] + inertia_modulation_joint_des_[i];

    // compensation_joint_des_[i] = inertia_modulation_joint_des_[i];
  }

  for (int i = 0; i < 4; i++)
  {
    robot_.joint_torque_des_[i][1] = robot_.joint_torque_des_[i][1] + compensation_joint_des_[i][0] + compensation_joint_des_[i][1];
    robot_.joint_torque_des_[i][2] = robot_.joint_torque_des_[i][2] + compensation_joint_des_[i][1];





  }

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

