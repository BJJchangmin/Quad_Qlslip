#include "FSM.hpp"
#include "orientation_tools.hpp"

#include <iostream>

using namespace ori;
using namespace std;


template <typename T>
FSM<T>::FSM(RobotLeg<T> & robot, CompensationControl<T> & comp_ctrl, FlightControl<T> & flight_ctrl,
             StanceForceControl<T> & stance_ctrl)
: robot_(robot), comp_ctrl_(comp_ctrl), flight_ctrl_(flight_ctrl), stance_ctrl_(stance_ctrl)
{
  lo_param_ptr_ = nullptr;
  td_param_ptr_ = nullptr;

  touch_threshold_ = 5;

  threshold_size_ = 15;

  for (size_t i = 0; i < 4; i++)
  {
    touch_[i].resize(threshold_size_);
    touch_[i].setZero();

    start_[i] = 0;
    phase_[i] = 0;
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

  for(size_t i = 0; i < 4; i++)
  {
    for(size_t j = 0; j < threshold_size_-2 ; j++)
    {
      touch_[i][j+1] = touch_[i][j];
    }
    touch_[i][0] = robot_.foot_contact_[i];

    //******************************************* Free Faling Start Check ****************************************** */
    if (touch_[i][0] > touch_threshold_ && start_[i] == 0)
    {
      start_[i] = 1;
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
        touch_[i][12] > touch_threshold_ )
        {
          cout << "Lift off state in  FSM" << endl;
          event_[i] = 4;
          Lift_off_state(i);
        }
      else if (touch_[i][0] > touch_threshold_ && touch_[i][1] <= touch_threshold_ && touch_[i][2] <= touch_threshold_ &&
        touch_[i][3] <= touch_threshold_ && touch_[i][4] <= touch_threshold_ && touch_[i][5] <= touch_threshold_ &&
        touch_[i][6] <= touch_threshold_ && touch_[i][7] <= touch_threshold_ && touch_[i][8] <= touch_threshold_ &&
        touch_[i][9] <= touch_threshold_ && touch_[i][10] <= touch_threshold_ && touch_[i][11] <= touch_threshold_ &&
        touch_[i][12] <= touch_threshold_ && touch_[i][13] <= touch_threshold_ && touch_[i][14] <= touch_threshold_)
        {

          event_[i] = 3;
          Touch_down_state(i);
        }



      //********************************************** Phase Check ***************************************** */
      if (time_ >= td_param_ptr_->t_TD[i] && td_param_ptr_->t_TD[i] > lo_param_ptr_->t_LO[i] )
      {
        phase_[i] = 1;
      }
      else if (time_ >= lo_param_ptr_->t_LO[i] && lo_param_ptr_->t_LO[i] > td_param_ptr_->t_TD[i])
      {
        phase_[i] = 2;
      }

    }
    robot_.phase_[i] = phase_[i];
    robot_.event_[i]= event_[i];

  }
  // std::cout<<"robot_.phase in FSM "<< robot_.phase_[0] <<std::endl;
  // std::cout<<"robot_.event in FSM "<< robot_.event_[0] <<std::endl;


}

template <typename T>
void FSM<T>::Lift_off_state(int Leg_num)
{
  /**
   * @brief Update Lift off state
   */
  cout << "Lift off state" << endl;
  lo_param_ptr_->r_LO[Leg_num] = robot_.foot_pos_rw_act_local_[Leg_num][0];
  lo_param_ptr_->dr_LO[Leg_num] = robot_.foot_vel_rw_act_local_[Leg_num][0];
  lo_param_ptr_->th_LO[Leg_num] = robot_.foot_pos_rw_act_local_[Leg_num][1];
  lo_param_ptr_->dth_LO[Leg_num] = robot_.foot_vel_rw_act_local_[Leg_num][1];
  lo_param_ptr_->t_LO[Leg_num] = time_;

  lo_param_ptr_->t_stance[Leg_num] = lo_param_ptr_->t_LO[Leg_num] - td_param_ptr_->t_TD[Leg_num];
  lo_param_ptr_->V_y_LO[Leg_num] = lo_param_ptr_->dr_LO[Leg_num]*sin(lo_param_ptr_->th_LO[Leg_num]) -
  lo_param_ptr_->r_LO[Leg_num]*lo_param_ptr_->dth_LO[Leg_num]*cos(lo_param_ptr_->th_LO[Leg_num]);

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

  for(size_t i = 0; i < 4; i++)
  {
    if (phase_[i] == 1 || phase_[i] == 0)
    {
      // std::cout << "1 : " << i << std::endl;
      stance_ctrl_.stance_control(i);
    }
    else if (phase_[i] == 2)
    {
      flight_ctrl_.flight_control(i);
    }
  }
  comp_ctrl_.compensation_control(); // Phase에 상관없이 실행되어야함.


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
