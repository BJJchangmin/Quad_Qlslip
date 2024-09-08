
#include "RobotLeg.hpp"
#include "orientation_tools.hpp"

using namespace ori;
using namespace std;

template <typename T>
void RobotLeg<T>::get_sensor_data(mjData * data)
{
  //* get body position *//
  for (size_t i = 0; i< 3; i++)
  {
    body_pos_world_[i] = data->sensordata[i];
    body_vel_world_[i] = data->sensordata[i + 3];

  }

  for (size_t i = 0; i < 4; i++)
  {
    foot_contact_[i] = data->sensordata[18 + 4*i];


     for(size_t j = 0; j < 3; j++)
     {
       //! qpos and qvel is different number
       joint_pos_act_[i][j] = data->qpos[7 + 3*i + j];
       joint_vel_act_[i][j] = data->qvel[7 + 3*i + j -1];

       foot_grf_world_[i][j] = data->sensordata[19 + 4*i + j];

     }

  }
}

template <typename T>
void RobotLeg<T>::bi_kinematic_transform()
{
  for (size_t i = 0; i < 4; i++)
  {
    //! pos_act[1] = hip, pos_act[2] = knee
    joint_pos_bi_act_[i][0] = joint_pos_act_[i][1];
    joint_pos_bi_act_[i][1] = joint_pos_act_[i][1] + joint_pos_act_[i][2];
    joint_vel_bi_act_[i][0] = joint_vel_act_[i][1];
    joint_vel_bi_act_[i][1] = joint_vel_act_[i][1] + joint_vel_act_[i][2];
  }

}

template <typename T>
void RobotLeg<T>::get_jacobians_rotating()
{
  for (size_t i = 0; i < 4; i++)
  {
    jacbRW[i] << thigh_link_length_[i] * sin(joint_pos_act_[i][2] / 2),
                 -thigh_link_length_[i] * sin(joint_pos_act_[i][2] / 2),
                 thigh_link_length_[i] * cos(joint_pos_act_[i][2] / 2),
                 thigh_link_length_[i] * cos(joint_pos_act_[i][2] / 2);
    jacbRW_inv[i] = jacbRW[i].inverse();
    jacbRW_tran[i] = jacbRW[i].transpose();
    jacbRW_tran_inv[i] = jacbRW_tran[i].inverse();
   }
}

template <typename T>
void RobotLeg<T>::bi_inverse_static_transform()
{
  for (size_t i = 0; i < 4; i++)
  {
    joint_torque_des_[i][1] = joint_torque_bi_des_[i][0] + joint_torque_bi_des_[i][1];
    joint_torque_des_[i][2] = joint_torque_bi_des_[i][1];
  }
}

template <typename T>
void RobotLeg<T>::inverse_static_rotating(const Vec2<T> & force_rw_des,int Leg_num)
{
    /**
     * @brief RW 방향 force가 들어오고 여기서의 output은 torque th_1, th_2
     * !Leg_num = 0,1,2,3 (FL,FR,RL,RR)
     */

   get_jacobians_rotating();
   joint_torque_bi_des_[Leg_num] = jacbRW[Leg_num].transpose() * force_rw_des;


   joint_torque_des_[Leg_num][1] = joint_torque_bi_des_[Leg_num][0] + joint_torque_bi_des_[Leg_num][1];
   joint_torque_des_[Leg_num][2] = joint_torque_bi_des_[Leg_num][1];
}

template <typename T>
void RobotLeg<T>::forward_kinematics_cartesian(){}

template <typename T>
void RobotLeg<T>::forward_kinematics_rotating()
{
  /**
    @param foot_pos_rw_act_local_[0] -> r direction
    @param foot_pos_rw_act_local_[1] -> th_r direction
    @param foot_vel_rw_act_local_[0] -> y_r direction velocity
    @param foot_vel_rw_act_local_[1] -> x_r direction velocity so i divid by r for make th_r velocity

  */
  for (size_t i = 0; i < 4; i++)
  {
    foot_pos_rw_act_local_[i][0]=
      2*thigh_link_length_[i]* cos((joint_pos_bi_act_[i][1] - joint_pos_bi_act_[i][0])/2);
    foot_pos_rw_act_local_[i][1] =((joint_pos_bi_act_[i][0] + joint_pos_bi_act_[i][1])/2);

    foot_vel_rw_act_local_[i] = jacbRW[i] * joint_vel_bi_act_[i];
    foot_vel_rw_act_local_[i][1] = foot_vel_rw_act_local_[i][1] / foot_pos_rw_act_local_[i][0];

  }

}



template class RobotLeg<double>;
template class RobotLeg<float>;
