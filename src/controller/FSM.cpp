#include "FSM.hpp"
#include "orientation_tools.hpp"

#include <iostream>

using namespace ori;
using namespace std;


template <typename T>
FSM<T>::FSM(RobotLeg<T> & robot, CompensationControl<T> & comp_ctrl, FlightControl<T> & flight_ctrl,
             StanceForceControl<T> & stance_ctrl, TrajectoryOptimization<T> & traj_opt, BezierTrajectory<T> & bezier_traj)
: robot_(robot), comp_ctrl_(comp_ctrl), flight_ctrl_(flight_ctrl),
  stance_ctrl_(stance_ctrl), traj_opt_(traj_opt), bezier_traj_(bezier_traj)
{
  lo_param_ptr_ = nullptr;
  td_param_ptr_ = nullptr;

  touch_threshold_ = 50;

  threshold_size_ = 15;
  loop_iter = 0;

  for (size_t i = 0; i < 4; i++)
  {
    touch_[i].resize(threshold_size_);
    touch_[i].setZero();

    start_[i] = 0;
    phase_[i][0] = 0;
    phase_[i][1] = 0; // old value
    event_[i] = 0;
  }
}

template <typename T>
void FSM<T>::phase_update(mjData * d)
{
  /**
   * @brief Phase Update
   * @param phase_: 0 Stance, 1: Flight
   * @param event_: 4: Lift off, 3: Touch down
   */

  time_ = d->time;


  for(size_t i = 0; i < robot_.k_num_dof_leg; i++)
  {
    for(size_t j = 0; j < threshold_size_-2 ; j++)
    {
      touch_[i][j+1] = touch_[i][j];
    }
    if (loop_iter % 1 == 0)
    {
      touch_[i][0] = robot_.foot_contact_[i];
    }


    //******************************************* Free Faling Start Check ****************************************** */
    if (touch_[i][0] > touch_threshold_ && start_[i] == 0 )
    {

      start_[i] = 1;
      if (start_[0] == 1 && start_[3] == 1)
      {
        /**
         * * galloping 이다.
         * @brief 시작할 때 나머지 다리 두개를 Swing Leg로 지정해주기 위해서 설계
         * @param i: 0, 1 -> Stance LEG (3*i)
         * @param i: 2, 3 -> Swing LEG (i+1)
         * todo : Optimization 실행시켜줘야함 그 부분을 뭘로 할지 고민해봐야함
         * ! 문제가 될 여지가 있는 param은 밑에 기록해둠
         */


        for (size_t i = 0; i < 2; i++)
        {
          Touch_down_state(3*i);
          start_[i+1] = 1;
          td_param_ptr_->t_TD[i+1] = 0;
          period_[i+1] = 0;

          lo_param_ptr_->r_LO[i+1] = robot_.foot_pos_rw_act_local_[i+1][0];
          lo_param_ptr_->dr_LO[i+1] = robot_.foot_vel_rw_act_local_[i+1][0];
          lo_param_ptr_->th_LO[i+1] = robot_.foot_pos_rw_act_local_[i+1][1];
          lo_param_ptr_->dth_LO[i+1] = robot_.foot_vel_rw_act_local_[i+1][1];
          lo_param_ptr_->t_LO[i+1] = time_;

          //? t_stance가 굉장히 짧을 수 있음. 어떤 값을 사용해야하나?
          lo_param_ptr_->t_stance[i+1] =  10*(lo_param_ptr_->t_LO[i+1] - td_param_ptr_->t_TD[i+1]);

          //? L_O 속도 어떤 값을 사용해야하나? 떨어 질 때 속도를 부호 바꿔서 사용? 생각해봐야함
          lo_param_ptr_->V_y_LO[i+1] = -0.1;

          td_param_ptr_->r_TD[i+1] = td_param_ptr_->r_TD[3*i];
          td_param_ptr_->dr_TD[i+1] = td_param_ptr_->dr_TD[3*i];
          td_param_ptr_->th_TD[i+1] = td_param_ptr_->th_TD[3*i];
          td_param_ptr_->dth_TD[i+1] = td_param_ptr_->dth_TD[3*i];

          bezier_traj_.state_update(i+1);
          bezier_traj_.Desired_Touch_Down_state(i+1);
          bezier_traj_.Desired_Flight_Time(i+1);

        }
      }
    }
    else if (touch_[i][0] < touch_threshold_ && start_[i] == 0)
    {
      // std::cout<<"start in FSM "<<std::endl;
      start_[i] = 0;
    }

    if (start_[i] == 1)
    {
      //*************************************** TD LO Check ***************************************** */
      event_[i] = 0;

      if (touch_[i][0] <= touch_threshold_ && touch_[i][1] > touch_threshold_ && touch_[i][2] > touch_threshold_ &&
        touch_[i][3] > touch_threshold_ && touch_[i][4] > touch_threshold_ && touch_[i][5] > touch_threshold_ &&
        touch_[i][6] > touch_threshold_ && touch_[i][7] > touch_threshold_ && touch_[i][8] > touch_threshold_ &&
        touch_[i][9] > touch_threshold_ && touch_[i][10] > touch_threshold_ && touch_[i][11] > touch_threshold_ &&
        touch_[i][12] > touch_threshold_ && touch_[i][13] > touch_threshold_ && touch_[i][14] > touch_threshold_ )
        {
          // event_[i] = 4;
          //Lift_off_state(i);
        }
      if (touch_[i][0] > touch_threshold_ && touch_[i][1] <= touch_threshold_ && touch_[i][2] <= touch_threshold_ &&
        touch_[i][3] <= touch_threshold_ && touch_[i][4] <= touch_threshold_ && touch_[i][5] <= touch_threshold_ &&
        touch_[i][6] <= touch_threshold_ && touch_[i][7] <= touch_threshold_ && touch_[i][8] <= touch_threshold_ &&
        touch_[i][9] <= touch_threshold_ && touch_[i][10] <= touch_threshold_ && touch_[i][11] <= touch_threshold_ &&
        touch_[i][12] <= touch_threshold_ && touch_[i][13] <= touch_threshold_ && touch_[i][14] <= touch_threshold_ )
        {

          event_[i] = 3;
          Touch_down_state(i);
          // todo 3.75는 trajectory에서 가져옴. 나중에 변수화 해줘야함
          period_[i] = 1*abs(td_param_ptr_->th_TD[i] -M_PI/2)*2/3.75;
          // period_[i] = 0.05;
          // cout << "0 : " <<td_param_ptr_->t_TD[0] << endl;
          // cout << "3 : " <<td_param_ptr_->t_TD[3] << endl;
          //cout << "period: " << period_[0] << endl;
        }



      //********************************************** Phase Check ***************************************** */
      // cout << td_param_ptr_->t_TD[0] << endl;
      if (time_ - td_param_ptr_->t_TD[i] <= period_[i])
      {
        phase_[i][0] = 1;
        // cout<< "phase: stance" << endl;
      }
      else if (time_ - td_param_ptr_->t_TD[i] > period_[i])
      {
        // cout << "i" << i << endl;
        // cout << "time : "<< time_ << endl;
        phase_[i][0] = 2;
        // cout<< "phase: flight" << endl;
      }
      else
      {
        cout << "else" << endl;
      }

      if (phase_[i][0] == 2 && phase_[i][1] == 1)
      {
        event_[i] = 4;
        Lift_off_state(i);
        cout << "Lift off" << endl;
      }

      // cout << "Leg : " << 0 <<", " << time_ - td_param_ptr_->t_TD[0] <<", "<< period_[0] <<endl;
      // cout << "Phase_Leg_ " << 0 <<": "<<phase_[0][0] << endl;

    }
    phase_[i][1] = phase_[i][0];
    robot_.phase_[i] = phase_[i][0];
    robot_.event_[i]= event_[i];

  }
  // std::cout<<"robot_.phase in FSM "<< robot_.phase_[0] <<std::endl;
  // std::cout<<"robot_.event in FSM "<< robot_.event_[0] <<std::endl;

  loop_iter++;
}

template <typename T>
void FSM<T>::Lift_off_state(int Leg_num)
{
  /**
   * @brief Update Lift off state
   */

  lo_param_ptr_->r_LO[Leg_num] = robot_.foot_pos_rw_act_local_[Leg_num][0];
  lo_param_ptr_->dr_LO[Leg_num] = robot_.foot_vel_rw_act_local_[Leg_num][0];
  lo_param_ptr_->th_LO[Leg_num] = robot_.foot_pos_rw_act_local_[Leg_num][1];
  lo_param_ptr_->dth_LO[Leg_num] = robot_.foot_vel_rw_act_local_[Leg_num][1];
  lo_param_ptr_->t_LO[Leg_num] = time_;

  lo_param_ptr_->t_stance[Leg_num] = lo_param_ptr_->t_LO[Leg_num] - td_param_ptr_->t_TD[Leg_num];
  // lo_param_ptr_->V_y_LO[Leg_num] = lo_param_ptr_->dr_LO[Leg_num]*sin(lo_param_ptr_->th_LO[Leg_num]) -
  //   lo_param_ptr_->r_LO[Leg_num]*lo_param_ptr_->dth_LO[Leg_num]*cos(lo_param_ptr_->th_LO[Leg_num]);

  lo_param_ptr_->V_y_LO[Leg_num] = robot_.hip_v_vel_[Leg_num];
  cout << "V_y_LO: " << lo_param_ptr_->V_y_LO[0] << endl;

}

template <typename T>
void FSM<T>::Touch_down_state(int Leg_num)
{
  /**
   * @brief Update Touch down state
   */
  td_param_ptr_->r_TD[Leg_num] = robot_.foot_pos_rw_act_local_[Leg_num][0];
  td_param_ptr_->dr_TD[Leg_num] = robot_.foot_vel_rw_act_local_[Leg_num][0];
  td_param_ptr_->th_TD[Leg_num] = robot_.foot_pos_rw_act_local_[Leg_num][1];
  td_param_ptr_->dth_TD[Leg_num] = robot_.foot_vel_rw_act_local_[Leg_num][1];
  td_param_ptr_->t_TD[Leg_num] = time_;
}

template <typename T>
void FSM<T>::FSM_control()
{
 /**
  * @brief FSM Control. Phase에 따라 제어기가 선택된다.
  * @param phase_: 0: Stance, 1: Flight
  */


  for(size_t i = 0; i < robot_.k_num_dof_leg; i++)
  {
    if (phase_[i][0] == 1 )
    {
      stance_ctrl_.stance_control(i);
      comp_ctrl_.compensation_control(i); // Phase에 상관없이 실행되어야함.
    }
    else if (phase_[i][0] == 2)
    {
      flight_ctrl_.flight_control(i);
      comp_ctrl_.compensation_control(i); // Phase에 상관없이 실행되어야함.
    }
  }



}



template <typename T>
void FSM<T>::get_optimization_pointer(std::shared_ptr<typename TrajectoryOptimization<T>::LO_param> lo_param_ptr,
                                      std::shared_ptr<typename TrajectoryOptimization<T>::TD_param> td_param_ptr)
{
  lo_param_ptr_ = lo_param_ptr;
  td_param_ptr_ = td_param_ptr;
}

template class FSM<float>;
template class FSM<double>;
