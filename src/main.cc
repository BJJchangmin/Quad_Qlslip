// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <mujoco/mujoco.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>

#include "glfw_adapter.h"
#include "simulate.h"
#include "array_safety.h"

//** Customized header for handling MuJoCo data */
#include "Mclquad.hpp"
#include "MotionTrajectory.hpp"
#include "MuJoCoInterface.hpp"
#include "TrackingController.hpp"
#include "data_logging.hpp"
#include "CompensationControl.hpp"
#include "TrajectoryOptimization.hpp"
#include "BezierTrajectory.hpp"
#include "StanceForceControl.hpp"
#include "FlightControl.hpp"
#include "FSM.hpp"


#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

extern "C" {
#if defined(_WIN32) || defined(__CYGWIN__)
  #include <windows.h>
#else
  #if defined(__APPLE__)
    #include <mach-o/dyld.h>
  #endif
  #include <sys/errno.h>
  #include <unistd.h>
#endif
}

namespace {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

// constants
const double syncMisalign = 0.1;        // maximum mis-alignment before re-sync (simulation seconds)
const double simRefreshFraction = 0.7;  // fraction of refresh available for simulation
const int kErrorLength = 1024;          // load error string length

//* ******************************************************************************************** *//
//* ******************************** MY CONSTANTS & OBJECT DECLARATION ************************* *//
//* ******************************************************************************************** *//

const double sim_end_time = 11.0;       // simulation end time (seconds)
unsigned int loop_iter = 0;             // loop iteration counter
RobotLeg<float> robot = buildMclQuad<float>();  // robot model
TrackingController<float> track_ctrl(robot);     // tracking controller
MotionTrajectory<float> traj_generator(robot);            // motion trajectory
DataLogging<float> data_logger(robot);         // data logger
CompensationControl<float> comp_ctrl(robot);   // compensation controller
TrajectoryOptimization<float> traj_opt(robot);  // trajectory optimization
BezierTrajectory<float> bezier_traj(robot);    // bezier trajectory
StanceForceControl<float> stance_ctrl(robot);  // stance force controller
FlightControl<float> flight_ctrl(robot);       // flight controller
FSM<float> fsm(robot,comp_ctrl,flight_ctrl,stance_ctrl, traj_opt, bezier_traj);  // finite state machine



std::shared_ptr<MuJoCoInterface<float>::MuJoCoActuatorCommand> actuator_cmd_ptr;
std::shared_ptr<MotionTrajectory<float>::DesiredFootTrajectory> foot_traj_ptr;
std::shared_ptr<MotionTrajectory<float>::DesiredJointTrajectory> joint_traj_ptr;
std::shared_ptr<TrajectoryOptimization<float>::LO_param> lo_param_ptr;
std::shared_ptr<TrajectoryOptimization<float>::TD_param> td_param_ptr;
std::shared_ptr<TrajectoryOptimization<float>::Optimization_param> op_param_ptr;


//* ******************************************************************************************** *//



// model and data
mjModel* m = nullptr;
mjData* d = nullptr;

// control noise variables
mjtNum* ctrlnoise = nullptr;

using Seconds = std::chrono::duration<double>;


//---------------------------------------- plugin handling -----------------------------------------

// return the path to the directory containing the current executable
// used to determine the location of auto-loaded plugin libraries
std::string getExecutableDir() {
#if defined(_WIN32) || defined(__CYGWIN__)
  constexpr char kPathSep = '\\';
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    DWORD buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new(std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      DWORD written = GetModuleFileNameA(nullptr, realpath.get(), buf_size);
      if (written < buf_size) {
        success = true;
      } else if (written == buf_size) {
        // realpath is too small, grow and retry
        buf_size *=2;
      } else {
        std::cerr << "failed to retrieve executable path: " << GetLastError() << "\n";
        return "";
      }
    }
    return realpath.get();
  }();
#else
  constexpr char kPathSep = '/';
#if defined(__APPLE__)
  std::unique_ptr<char[]> buf(nullptr);
  {
    std::uint32_t buf_size = 0;
    _NSGetExecutablePath(nullptr, &buf_size);
    buf.reset(new char[buf_size]);
    if (!buf) {
      std::cerr << "cannot allocate memory to store executable path\n";
      return "";
    }
    if (_NSGetExecutablePath(buf.get(), &buf_size)) {
      std::cerr << "unexpected error from _NSGetExecutablePath\n";
    }
  }
  const char* path = buf.get();
#else
  const char* path = "/proc/self/exe";
#endif
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    std::uint32_t buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new(std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      std::size_t written = readlink(path, realpath.get(), buf_size);
      if (written < buf_size) {
        realpath.get()[written] = '\0';
        success = true;
      } else if (written == -1) {
        if (errno == EINVAL) {
          // path is already not a symlink, just use it
          return path;
        }

        std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
        return "";
      } else {
        // realpath is too small, grow and retry
        buf_size *= 2;
      }
    }
    return realpath.get();
  }();
#endif

  if (realpath.empty()) {
    return "";
  }

  for (std::size_t i = realpath.size() - 1; i > 0; --i) {
    if (realpath.c_str()[i] == kPathSep) {
      return realpath.substr(0, i);
    }
  }

  // don't scan through the entire file system's root
  return "";
}



// scan for libraries in the plugin directory to load additional plugins
void scanPluginLibraries() {
  // check and print plugins that are linked directly into the executable
  int nplugin = mjp_pluginCount();
  if (nplugin) {
    std::printf("Built-in plugins:\n");
    for (int i = 0; i < nplugin; ++i) {
      std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
    }
  }

  // define platform-specific strings
#if defined(_WIN32) || defined(__CYGWIN__)
  const std::string sep = "\\";
#else
  const std::string sep = "/";
#endif


  // try to open the ${EXECDIR}/plugin directory
  // ${EXECDIR} is the directory containing the simulate binary itself
  const std::string executable_dir = getExecutableDir();
  if (executable_dir.empty()) {
    return;
  }

  const std::string plugin_dir = getExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
  mj_loadAllPluginLibraries(
      plugin_dir.c_str(), +[](const char* filename, int first, int count) {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
      });
}


//------------------------------------------- simulation -------------------------------------------


mjModel* LoadModel(const char* file, mj::Simulate& sim) {
  // this copy is needed so that the mju::strlen call below compiles
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);

  // make sure filename is not empty
  if (!filename[0]) {
    return nullptr;
  }

  // load and compile
  char loadError[kErrorLength] = "";
  mjModel* mnew = 0;
  if (mju::strlen_arr(filename)>4 &&
      !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                    mju::sizeof_arr(filename) - mju::strlen_arr(filename)+4)) {
    mnew = mj_loadModel(filename, nullptr);
    if (!mnew) {
      mju::strcpy_arr(loadError, "could not load binary model");
    }
  } else {
    mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);
    // remove trailing newline character from loadError
    if (loadError[0]) {
      int error_length = mju::strlen_arr(loadError);
      if (loadError[error_length-1] == '\n') {
        loadError[error_length-1] = '\0';
      }
    }
  }

  mju::strcpy_arr(sim.load_error, loadError);

  if (!mnew) {
    std::printf("%s\n", loadError);
    return nullptr;
  }

  // compiler warning: print and pause
  if (loadError[0]) {
    // mj_forward() below will print the warning message
    std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
    sim.run = 0;
  }

  return mnew;
}

//* ******************************************************************************************** *//
//* ******************************** Custom Function  ****************************************** *//
void apply_joint_control(mjData * d)
{
  //! qpos and qvel is different number
  d->qpos[2] = 0;
  d->qpos[5] = 0;
  d->qpos[8] = 0;
  d->qpos[11] = 0;



  for (size_t i = 0; i < 4; i++)
  {

    for (size_t j = 1; j < 3; j++)
    {
      d->ctrl[3*i+j] = robot.joint_torque_des_[i][j];
    }

  }

}


void YCM_controller()
{

  //! Custom Controller
  // * **************************************************************************************** *//
  //* ***** GENERATE DESIRED JOINT COMMAND AND APPLY CONTROL INPUT FOR SIMULATION***** *//

  //* Parameter
  bool bIsPerturbOn = false;
  double r_init = 0.3;
  double r_ref = 0.4;
  double v_ref;
  double t = d->time;
  double t1 = 3; // 3
  double t2 = 6; // 6
  double t3 = 9;
  double t4 = 12;


  //*************** Velocity Trans ***************/

  // double v_1 = 0.5;
  // double v_2 = 0.85;
  // double v_3 = 1.2;
  double v_1 = 0.5;
  double v_2 = 0.5;
  double v_3 = 0.5;
  if ((0 <= t) & (t < t1))
  {
    v_ref = v_1;  // 0.2
  }
  else if ((t1 <= t) && (t < t2))
  {
    v_ref = (v_2-v_1)*t/(t2-t1)+(v_2-(v_2-v_1)*t2/(t2-t1));  // 0.25
  }
  else if (t2 <= t && t < t3)
  {
    v_ref = v_2;
  }
  else if (t3 <= t && t < t4)
  {
    v_ref = (v_3-v_2)*t/(t4-t3)+(v_3-(v_3-v_2)*t4/(t4-t3));
  }
  else
  {
    v_ref = v_3;
  }

  //******************Perturbation ***********************/
  if (bIsPerturbOn)
  {
    if(d->time >= 2.706 && d->time <= 2.78)
    {
      std::cout << "Perturbation" << std::endl;
      d->xfrc_applied[2*6] = 500;
    }
  }

  // std::cout << "com : " << d->subtree_com[0] << std::endl;

  fsm.phase_update(d);

  // traj_generator.initial_trotting(r_init,d->time,1);
  // track_ctrl.RW_posPD_control();

  // * Controller Function
  // traj_opt.Flight_traj_generate(d);
  bezier_traj.Flight_traj_generate(d);
  traj_generator.QLSLIP_Trajectory(r_ref, v_ref, d);
  fsm.FSM_control();
  comp_ctrl.Trunk_mass_compensation(d);
  apply_joint_control(d);

}

//* ******************************************************************************************** *//

// simulate in background thread (while rendering in main thread)
void PhysicsLoop(mj::Simulate& sim) {
  // cpu-sim syncronization point
  std::chrono::time_point<mj::Simulate::Clock> syncCPU;
  mjtNum syncSim = 0;

  // run until asked to exit
  while (!sim.exitrequest.load())
  {
    // ! Exit simulation if time exceeded the specified end time //
    if (sim.d_->time > sim_end_time)
    {
      sim.exitrequest = 1;
    }

    if (sim.droploadrequest.load()) {
      sim.LoadMessage(sim.dropfilename);
      mjModel* mnew = LoadModel(sim.dropfilename, sim);
      sim.droploadrequest.store(false);

      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.dropfilename);

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        mj_forward(m, d);

        // allocate ctrlnoise
        free(ctrlnoise);
        ctrlnoise = (mjtNum*) malloc(sizeof(mjtNum)*m->nu);
        mju_zero(ctrlnoise, m->nu);
      } else {
        sim.LoadMessageClear();
      }
    }

    if (sim.uiloadrequest.load()) {
      sim.uiloadrequest.fetch_sub(1);
      sim.LoadMessage(sim.filename);
      mjModel* mnew = LoadModel(sim.filename, sim);
      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.filename);

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        mj_forward(m, d);

        // allocate ctrlnoise
        free(ctrlnoise);
        ctrlnoise = static_cast<mjtNum*>(malloc(sizeof(mjtNum)*m->nu));
        mju_zero(ctrlnoise, m->nu);
      } else {
        sim.LoadMessageClear();
      }
    }

    // sleep for 1 ms or yield, to let main thread run
    //  yield results in busy wait - which has better timing but kills battery life
    if (sim.run && sim.busywait) {
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

      // run only if model is present
      if (m) {
        // running
        if (sim.run) {
          bool stepped = false;

          // record cpu time at start of iteration
          const auto startCPU = mj::Simulate::Clock::now();

          // elapsed CPU and simulation time since last sync
          const auto elapsedCPU = startCPU - syncCPU;
          double elapsedSim = d->time - syncSim;

          // inject noise
          if (sim.ctrl_noise_std) {
            // convert rate and scale to discrete time (Ornstein–Uhlenbeck)
            mjtNum rate = mju_exp(-m->opt.timestep / mju_max(sim.ctrl_noise_rate, mjMINVAL));
            mjtNum scale = sim.ctrl_noise_std * mju_sqrt(1-rate*rate);

            for (int i=0; i<m->nu; i++) {
              // update noise
              ctrlnoise[i] = rate * ctrlnoise[i] + scale * mju_standardNormal(nullptr);

              // apply noise
              d->ctrl[i] = ctrlnoise[i];
            }
          }

          // requested slow-down factor
          double slowdown = 100 / sim.percentRealTime[sim.real_time_index];

          // double slowdown;
          // if (d->time >= 2.7 && d->time <= 3.5)  // 0.7 ~ 0.704
          // {
          //   slowdown = 20;  // x-direction force
          // }
          // else
          // {
          //   slowdown = 1;
          // }


          // misalignment condition: distance from target sim time is bigger than syncmisalign
          bool misaligned =
              mju_abs(Seconds(elapsedCPU).count()/slowdown - elapsedSim) > syncMisalign;

          // out-of-sync (for any reason): reset sync times, step
          if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 ||
              misaligned || sim.speed_changed) {
            // re-sync
            syncCPU = startCPU;
            syncSim = d->time;
            sim.speed_changed = false;


            // run single step, let next iteration deal with timing
            //! Controller Location
            YCM_controller();
            mj_step(m, d);
            stepped = true;
            //* ******** READ SENSOR DATA AND CONDUCT CALCULATOIN FOR STATE ESTIMATION ********* *//
            robot.get_sensor_data(d);
            robot.bi_kinematic_transform();
            robot.forward_kinematics_rotating();
            //************ Data logging *************/
            if (loop_iter % data_logger.get_logging_freq() == 0)
            {
              data_logger.save_data(m, d);
            }
             loop_iter++;

          }

          // in-sync: step until ahead of cpu
          else {
            bool measured = false;
            mjtNum prevSim = d->time;

            double refreshTime = simRefreshFraction/sim.refresh_rate;

            // step while sim lags behind cpu and within refreshTime
            while (Seconds((d->time - syncSim)*slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                   mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime)) {
              // measure slowdown before first step
              if (!measured && elapsedSim) {
                sim.measured_slowdown =
                    std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                measured = true;
              }

            // run single step, let next iteration deal with timing
            //! Controller Location
            YCM_controller();
            mj_step(m, d);
            stepped = true;
            //* ******** READ SENSOR DATA AND CONDUCT CALCULATOIN FOR STATE ESTIMATION ********* *//
            robot.get_sensor_data(d);
            robot.bi_kinematic_transform();
            robot.forward_kinematics_rotating();
            //************ Data logging *************/
            if (loop_iter % data_logger.get_logging_freq() == 0)
            {
              data_logger.save_data(m, d);
            }
             loop_iter++;

              //* **************************************************************************************** *//
              // break if reset
              if (d->time < prevSim) {
                break;
              }
            }
          }

          // save current state to history buffer
          if (stepped) {
            sim.AddToHistory();
          }
        }

        // paused
        else {
          // run mj_forward, to update rendering and joint sliders
          mj_forward(m, d);
          sim.speed_changed = true;
        }
      }
    }  // release std::lock_guard<std::mutex>
  }
}
}  // namespace

//-------------------------------------- physics_thread --------------------------------------------

void PhysicsThread(mj::Simulate* sim, const char* filename) {
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr) {
    sim->LoadMessage(filename);
    m = LoadModel(filename, *sim);
    if (m) {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      d = mj_makeData(m);
    }
    if (d) {
      sim->Load(m, d, filename);

      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      mj_forward(m, d);

      // allocate ctrlnoise
      free(ctrlnoise);
      ctrlnoise = static_cast<mjtNum*>(malloc(sizeof(mjtNum)*m->nu));
      mju_zero(ctrlnoise, m->nu);
    } else {
      sim->LoadMessageClear();
    }
  }

  PhysicsLoop(*sim);

  // delete everything we allocated
  free(ctrlnoise);
  mj_deleteData(d);
  mj_deleteModel(m);
}

//------------------------------------------ main --------------------------------------------------

// machinery for replacing command line error by a macOS dialog box when running under Rosetta
#if defined(__APPLE__) && defined(__AVX__)
extern void DisplayErrorDialogBox(const char* title, const char* msg);
static const char* rosetta_error_msg = nullptr;
__attribute__((used, visibility("default"))) extern "C" void _mj_rosettaError(const char* msg) {
  rosetta_error_msg = msg;
}
#endif

// run event loop
int main(int argc, char** argv) {

  // display an error if running on macOS under Rosetta 2
#if defined(__APPLE__) && defined(__AVX__)
  if (rosetta_error_msg) {
    DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
    std::exit(1);
  }
#endif

  // print version, check compatibility
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER!=mj_version()) {
    mju_error("Headers and library have different versions");
  }

  // scan for libraries in the plugin directory to load additional plugins
  scanPluginLibraries();

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  // simulate object encapsulates the UI
  auto sim = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      &cam, &opt, &pert, /* is_passive = */ false
  );

  const char* filename = "../model/mcl_quad/scene.xml"; // Settting My model
  if (argc >  1) {
    filename = argv[1];
  }

  //* ****************************************************************************************** *//
  //* ********** CREATE MUJOCO INTERFACE OBJECT AND SENSOR & ACTUATOR POINTER ****************** *//
  //* ****************************************************************************************** *//
  auto mujoco_interface = std::make_shared<MuJoCoInterface<float>>(sim.get(), robot);
  foot_traj_ptr = traj_generator.get_foot_traj_ptr();
  joint_traj_ptr = traj_generator.get_joint_traj_ptr();
  lo_param_ptr = traj_opt.get_lo_param_ptr();
  td_param_ptr = traj_opt.get_td_param_ptr();
  op_param_ptr = traj_opt.get_op_param_ptr();


  //* ****************************************************************************************** *//
  //* *************** GET POINTER s.t. EACH OBJECT CAN SHARE THE SAME DATA ********************* *//
  //* ****************************************************************************************** *//
  track_ctrl.get_traj_pointer(foot_traj_ptr, joint_traj_ptr);
  traj_opt.get_traj_pointer(foot_traj_ptr);
  stance_ctrl.get_traj_pointer(foot_traj_ptr);
  flight_ctrl.get_traj_pointer(foot_traj_ptr);
  fsm.get_optimization_pointer(lo_param_ptr, td_param_ptr);
  data_logger.set_traj_ptr(foot_traj_ptr, joint_traj_ptr);
  data_logger.set_op_param_ptr(op_param_ptr, lo_param_ptr, td_param_ptr);
  bezier_traj.get_traj_pointer(foot_traj_ptr);
  bezier_traj.get_optimization_pointer(op_param_ptr, lo_param_ptr, td_param_ptr);






  // start physics thread
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename);

  // start simulation UI loop (blocking call)
  sim->RenderLoop();
  physicsthreadhandle.join();

  return 0;
}
