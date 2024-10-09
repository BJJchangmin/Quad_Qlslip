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
  foot_traj_ptr_ = nullptr;
  pcv_ptr_ = std::make_shared<PCV>();

  touch_threshold_ = 20;

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
    swing_lock_[i] = false;
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
      swing_lock_phase_[i] = 1;
      // if (start_[0] == 1 && start_[3] == 1)
      // {
      //   /**
      //    * * galloping 이다.
      //    * @brief 시작할 때 나머지 다리 두개를 Swing Leg로 지정해주기 위해서 설계
      //    * @param i: 0, 1 -> Stance LEG (3*i)
      //    * @param i: 2, 3 -> Swing LEG (i+1)
      //    * todo : Optimization 실행시켜줘야함 그 부분을 뭘로 할지 고민해봐야함
      //    * ! 문제가 될 여지가 있는 param은 밑에 기록해둠
      //    */


      //   for (size_t i = 0; i < 2; i++)
      //   {
      //     Touch_down_state(3*i);
      //     start_[i+1] = 1;
      //     td_param_ptr_->t_TD[i+1] = 0;
      //     period_[i+1] = 0;

      //     lo_param_ptr_->r_LO[i+1] = robot_.foot_pos_rw_act_local_[i+1][0];
      //     lo_param_ptr_->dr_LO[i+1] = robot_.foot_vel_rw_act_local_[i+1][0];
      //     lo_param_ptr_->th_LO[i+1] = robot_.foot_pos_rw_act_local_[i+1][1];
      //     lo_param_ptr_->dth_LO[i+1] = robot_.foot_vel_rw_act_local_[i+1][1];
      //     lo_param_ptr_->t_LO[i+1] = time_;

      //     //? t_stance가 굉장히 짧을 수 있음. 어떤 값을 사용해야하나?
      //     lo_param_ptr_->t_stance[i+1] =  10*(lo_param_ptr_->t_LO[i+1] - td_param_ptr_->t_TD[i+1]);

      //     //? L_O 속도 어떤 값을 사용해야하나? 떨어 질 때 속도를 부호 바꿔서 사용? 생각해봐야함
      //     lo_param_ptr_->V_y_LO[i+1] = -0.1;

      //     td_param_ptr_->r_TD[i+1] = td_param_ptr_->r_TD[3*i];
      //     td_param_ptr_->dr_TD[i+1] = td_param_ptr_->dr_TD[3*i];
      //     td_param_ptr_->th_TD[i+1] = td_param_ptr_->th_TD[3*i];
      //     td_param_ptr_->dth_TD[i+1] = td_param_ptr_->dth_TD[3*i];

      //     bezier_traj_.state_update(i+1);
      //     bezier_traj_.Desired_Touch_Down_state(i+1);
      //     bezier_traj_.Desired_Flight_Time(i+1);

      //   }
      // }
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
      swing_lock_period_[i] = 0.05;
      // period_[i] = 1*abs(td_param_ptr_->th_TD[i] -M_PI/2)*0.2/2.5;

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
          if (swing_lock_[i] == false && swing_lock_phase_[i] == 1)
          {
            event_[i] = 3;
            Touch_down_state(i);
            //** PCV_Control */
            PCV_control(i);
          }
          else if (swing_lock_[i] == true)
          {
            //* Noting happen
          }
        }


    }
  }
  //************************************ 2족처럼 Phase 맞춰주기 위한 과정 ******************************* */
  if(start_[0] == 1 || start_[1] == 1)
  {
    for(size_t i = 0; i < 2; i++)
    {
      if(time_ >= td_param_ptr_->t_TD[i] && td_param_ptr_->t_TD[i] > lo_param_ptr_->t_LO[i] )
      {
        //! 여기서 Lift off를 결정해줘야한다. 그런데 다른 다리가 TD 했다는 항도 함께 있어야 하기 때문에 그거 생각해서 짜줘야함. 그럴려면 여기 있어도 되는가?
        //! 상대 다리가 TD 하면 이 안에 들어와진다. 둘다 이 안에 있을 때 하나 결정해서 LO 시켜서 나가면 됨
        phase_[i][0] = 1;
        swing_lock_phase_[i] = 0;
        // cout << i << endl;
        pcv_ptr_->time[i] = time_ - td_param_ptr_->t_TD[i];
        pcv_ptr_->Ratio[i] = pcv_ptr_->time[i]/pcv_ptr_->update_Period[i];
        cout << "Ratio : " << 0 << " : " << pcv_ptr_->Ratio[0] << endl;

        // ? 이러면 한번만 들어와지는게 맞는가? 일단 Event에서 Phase로 바꿈 생각했을 떄는 괜찮을 듯
        if (pcv_ptr_->Ratio[0] >= 1 && phase_[1][0] == 1 && phase_[0][0] != 2)
        {
          event_[0] = 4;
          Lift_off_state(0);
          pcv_ptr_->Ratio[0] = .0;
        }
        if (pcv_ptr_->Ratio[1] >= 1 && phase_[0][0] == 1 && phase_[1][0] != 2)
        {

          event_[1] = 4;
          Lift_off_state(1);
          pcv_ptr_->Ratio[1] = .0;
        }
        if (pcv_ptr_->Ratio[2] >= 1 && phase_[3][0] == 1)
        {
          event_[2] = 4;
          Lift_off_state(2);
          pcv_ptr_->Ratio[2] = .0;
        }
        if (pcv_ptr_->Ratio[3] >= 1 && phase_[2][0] == 1)
        {
          event_[3] = 4;
          Lift_off_state(3);
          pcv_ptr_->Ratio[3] = .0;
        }

      }
      else if(time_ >= lo_param_ptr_->t_LO[i] && lo_param_ptr_->t_LO[i] > td_param_ptr_->t_TD[i] )
      {
        phase_[i][0] = 2;
        swing_lock_phase_[i] = 1;

        //* Swing Lock Algorithm */
        if (time_ - lo_param_ptr_->t_LO[i] <= swing_lock_period_[i])
        {
          swing_lock_[i] = true;
        }
        else
        {
          swing_lock_[i] = false;
        }

      }

    }

  }

  for(size_t i = 0; i < robot_.k_num_dof_leg; i++)
  {
    phase_[i][1] = phase_[i][0];
    robot_.phase_[i] = phase_[i][0];
    robot_.event_[i]= event_[i];

  }

  //! Debugging
  // cout << "TD : " << 0 << " : " << td_param_ptr_->t_TD[0] << endl;
  // cout << "GAP : " << 0 << " : " << pcv_ptr_->GAP[0] << endl;
  // cout << "stance_period : " << 0 << " : " << td_param_ptr_->stance_Period[0] << endl;
  // cout << "pcv_update_time : " << 0 << " : " << pcv_ptr_->update_Period[0] << endl;
  // cout << "pcv_time : " << 1 << " : " << pcv_ptr_->time[1] << endl;
  // cout << "Ratio : " << 1 << " : " << pcv_ptr_->Ratio[1] << endl;
  // cout << "phase : " << 1 << " : " << phase_[1][0] << endl;
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

  lo_param_ptr_->V_y_LO[Leg_num] = robot_.hip_v_vel_[Leg_num];


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
  td_param_ptr_->stance_Period[Leg_num] = 2*abs((td_param_ptr_->th_TD[Leg_num] -M_PI/2)/foot_traj_ptr_->foot_vel_rw_des_[0][1]);

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
void FSM<T>::PCV_control(int Leg_num)
{
  /**
   * * PCV_Conrtol , Location is under the TD Detection
   * @param FL,FR,RL,RR(0,1,2,3)
   * ! 각 다리별로 누구 기준으로 해줘야하는지 직접 설정해줘야함
   * ! 일단 2개의 다리로만 문제를 풀기
   * todo : matching이 되어야 하는 다리 설정, gain setting, desired phase setting
   */

  pcv_ptr_->Des_Phase[Leg_num] = 0.5;
  pcv_ptr_->p1[Leg_num] = 0.0105;


  if(Leg_num == 0)
  {
    pcv_ptr_->GAP[0] = pcv_ptr_->Ratio[1] - pcv_ptr_->Ratio[0];
    cout << "GAP : " << Leg_num << " : " << pcv_ptr_->GAP[Leg_num] << endl;
    cout << "Gap_Rattio : " << Leg_num << " : " << pcv_ptr_->Ratio[1] << " : " << pcv_ptr_->Ratio[0] << endl;

  }

  if(Leg_num == 1)
  {
    // cout << "Ratio : " << 0 << " : " << pcv_ptr_->Ratio[1] << endl;
    // cout << "Ratio : " << 1 << " : " << pcv_ptr_->Ratio[0] << endl;

    pcv_ptr_->GAP[1] = pcv_ptr_->Ratio[0] - pcv_ptr_->Ratio[1];
    cout << "GAP : " << Leg_num << " : " << pcv_ptr_->GAP[Leg_num] << endl;
    cout << "Gap_Rattio : " << Leg_num << " : " << pcv_ptr_->Ratio[0] << " : " << pcv_ptr_->Ratio[1] << endl;
  }

  if(Leg_num == 2)
  {
    pcv_ptr_->GAP[2] = pcv_ptr_->Ratio[3] - pcv_ptr_->Ratio[2];
  }

  if(Leg_num == 3)
  {
    pcv_ptr_->GAP[3] = pcv_ptr_->Ratio[2] - pcv_ptr_->Ratio[3];
  }
  // cout << "Ratio : " << Leg_num << " : " << pcv_ptr_->Ratio[Leg_num] << endl;

  pcv_ptr_->update_Period[Leg_num] = td_param_ptr_->stance_Period[Leg_num] + pcv_ptr_->p1[Leg_num]*(pcv_ptr_->Des_Phase[Leg_num] - pcv_ptr_->GAP[Leg_num]);
  // cout << "stance_Period : " << Leg_num << " : " << td_param_ptr_->stance_Period[0] << endl;
  cout << "update_Period : " << Leg_num << " : " << pcv_ptr_->update_Period[0] << endl;

}



template <typename T>
std::shared_ptr<typename FSM<T>::PCV> FSM<T>::get_pcv_ptr()
{
  return pcv_ptr_;
}


template <typename T>
void FSM<T>::get_traj_pointer(std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr)
{
  foot_traj_ptr_ = foot_traj_ptr;
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
