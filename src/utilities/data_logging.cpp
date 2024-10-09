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
  op_param_ptr_ = nullptr;
  lo_param_ptr_ = nullptr;
  td_param_ptr_ = nullptr;
  pcv_ptr_ = nullptr;


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
      fout_[i] << robot_.foot_contact_[i] << ",";  // touch

      fout_[i] << op_param_ptr_->r_des_TD[i] << ",";  // r_des_TD
      fout_[i] << op_param_ptr_->dr_des_TD[i] << ",";  // dr_des_TD
      fout_[i] << op_param_ptr_->th_des_TD[i] << ",";  // th_des_TD
      fout_[i] << op_param_ptr_->dth_des_TD[i] << ",";  // dth_des_TD
      fout_[i] << op_param_ptr_->t_flight_des[i] << ",";  // t_flight_des

      fout_[i] << td_param_ptr_->r_TD[i] << ",";  // r_TD
      fout_[i] << td_param_ptr_->dr_TD[i] << ",";  // dr_TD
      fout_[i] << td_param_ptr_->th_TD[i] << ",";  // th_TD
      fout_[i] << td_param_ptr_->dth_TD[i] << ",";  // dth_TD
      fout_[i] << td_param_ptr_->t_TD[i] << ",";  // t_TD

      fout_[i] << lo_param_ptr_->r_LO[i] << ",";  // r_LO
      fout_[i] << lo_param_ptr_->dr_LO[i] << ",";  // dr_LO
      fout_[i] << lo_param_ptr_->th_LO[i] << ",";  // th_LO
      fout_[i] << lo_param_ptr_->dth_LO[i] << ",";  // dth_LO
      fout_[i] << lo_param_ptr_->t_LO[i] << ",";  // t_LO
      fout_[i] << pcv_ptr_->Des_Phase[i] << ","; // Desired Phase
      fout_[i] << pcv_ptr_->GAP[i]; // In PCV Ratio GAP

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

    fout_[4] << d->sensordata[34] << ","; // x velocity of the trunk
    fout_[4] << d->sensordata[35] << ","; // y velocity of the trunk
    fout_[4] << d->sensordata[36] << ","; // z velocity of the trunk
    fout_[4] << d->sensordata[38] << ","; // y position of the trunk
    fout_[4] << d->sensordata[39]; // z position of the trunk
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
      fout_[i] << "phase, event, touch ";
      fout_[i] << "r_des_TD, dr_des_TD, th_des_TD, dth_des_TD, t_flight_des, ";
      fout_[i] << "r_TD, dr_TD, th_TD, dth_TD, t_TD, ";
      fout_[i] << "r_LO, dr_LO, th_LO, dth_LO, t_LO, ";
      fout_[i] << "Des_phase, phase_GAP " << std::endl;
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
    fout_[4] << "trunk_x_vel, trunk_y_vel, trunk_z_vel, ";
    fout_[4] << "trunk_y_pos, trunk_z_pos " << std::endl;
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
void DataLogging<T>::set_op_param_ptr(
    std::shared_ptr<typename TrajectoryOptimization<T>::Optimization_param> op_param_ptr,
    std::shared_ptr<typename TrajectoryOptimization<T>::LO_param> lo_param_ptr,
    std::shared_ptr<typename TrajectoryOptimization<T>::TD_param> td_param_ptr)
{
  op_param_ptr_ = op_param_ptr;
  lo_param_ptr_ = lo_param_ptr;
  td_param_ptr_ = td_param_ptr;
}

template <typename T>
void DataLogging<T>::set_pcv_ptr(std::shared_ptr<typename FSM<T>::PCV> pcv_ptr)
{
  pcv_ptr_ = pcv_ptr;
}


template <typename T>
int DataLogging<T>::get_logging_freq()
{
  return k_save_sampling;
}

template class DataLogging<float>;
template class DataLogging<double>;

