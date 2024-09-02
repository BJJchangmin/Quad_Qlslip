#include "data_logging.hpp"

using namespace std;

/**
 * ! If you don't change file name, and run the simulation again, the data will be overwritten.
 */
template <typename T>
DataLogging<T>::DataLogging(RobotLeg<T> & robot_)
  : robot_(robot_)
{
  cout << "Data Logger object is created" << endl;
  fout_[0].open("../data/data_FL.csv");
  fout_[1].open("../data/data_FR.csv");
  fout_[2].open("../data/data_RL.csv");
  fout_[3].open("../data/data_RR.csv");
  fout_[4].open("../data/data_trunk.csv");


  foot_traj_ptr_ = nullptr;
  for (int i = 0; i < 5; i++)
  {
    if (!fout_[i])
    {
      std::cerr << "Cannot open file: " << i <<std::endl;
      exit(1);
    }
  }

  init_data();

}


template <typename T>
void DataLogging<T>::save_data(const mjModel* m, mjData* d)
{
  /**
  * @brief Select data to save
  * You can select which data of the LoggingData struct to save.
  * * Data are separated by a comma (,) followed by a space
  * ! comma (,) should be omitted in the last line and newline must exist after the last data.
  */


  for (int i = 0; i < 4; i++)
  {
    //************************************ Leg Data ********************************************** */
    if (!fout_[i])
    {

      std::cerr << "Cannot open file: " << i <<std::endl;
      exit(1);
    }
    else
    {

      fout_[i] << d->time << ","; // time
      fout_[i] << foot_traj_ptr_->foot_pos_rw_des_[i][0] << ","; // r direction ref
      fout_[i] << robot_.foot_pos_rw_act_local_[i][0] << ","; // r direction,FL
      fout_[i] << foot_traj_ptr_->foot_pos_rw_des_[i][1] << ","; // th_r direction ref
      fout_[i] << robot_.foot_pos_rw_act_local_[i][1] << ","; // th_r direction,FL

      fout_[i] << foot_traj_ptr_->foot_vel_rw_des_[i][0] << ","; // r direction vel ref
      fout_[i] << robot_.foot_vel_rw_act_local_[i][0] << ","; // r direction vel,FL
      fout_[i] << foot_traj_ptr_->foot_vel_rw_des_[i][1] << ","; // th_r direction vel ref
      fout_[i] << robot_.foot_vel_rw_act_local_[i][1] << ","; // th_r direction vel,FL

      fout_[i] << robot_.joint_torque_des_[i][0] << ","; // joint torque HAA
      fout_[i] << robot_.joint_torque_des_[i][1] << ","; // joint torque HFE
      fout_[i] << robot_.joint_torque_des_[i][2] << ",";  // joint torque KFE

      fout_[i] << robot_.phase_[i] << ",";  // phase
      fout_[i] << robot_.event_[i] << ",";  // event
      fout_[i] << robot_.foot_contact_[i] ;

      // ! Don't remove the newline
      fout_[i] << endl;

    }
  }


  //************************************Trunk Data ********************************************** */
  if (!fout_[4])
  {
    std::cerr << "Cannot open file: " << 4 <<std::endl;
    exit(1);
  }
  else
  {

    fout_[4] << d->time; // time


    // ! Don't remove the newline
    fout_[4] << endl;
  }



}

template <typename T>
void DataLogging<T>::init_data()
{
  //************************************ Leg Data ********************************************** */
  for (int i = 0; i < 4; i++)
  {
    if (!fout_[i])
    {
      std::cerr << "Cannot open file: " << i <<std::endl;
      exit(1);
    }
    else
    {
      fout_[i] << "time, r_des, r_act, th_des, th_act,";
      fout_[i] << "dr_des, dr_act, dth_des, dth_act, ";
      fout_[i] << "tau_HAA, tau_HFE, tau_KFE, ";
      fout_[i] << "phase, event, touch "<<std::endl;
    }
  }
  //************************************Trunk Data ********************************************** */
  if (!fout_[4])
  {
    std::cerr << "Cannot open file: " << 4 <<std::endl;
    exit(1);
  }
  else
  {
    fout_[4] << "time" << std::endl;
  }

}

template <typename T>
void DataLogging<T>::set_traj_ptr(
    std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr,
    std::shared_ptr<typename MotionTrajectory<T>::DesiredJointTrajectory> joint_traj_ptr)
{
  foot_traj_ptr_ = foot_traj_ptr;
  joint_traj_ptr_ = joint_traj_ptr;
}


template <typename T>
int DataLogging<T>::get_logging_freq()
{
  return k_save_sampling;
}

template class DataLogging<float>;
template class DataLogging<double>;
