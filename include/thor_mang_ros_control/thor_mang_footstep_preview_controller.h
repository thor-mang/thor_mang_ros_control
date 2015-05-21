//=================================================================================================
// Copyright (c) 2015, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef THOR_MANG_FOOTSTEP_PREVIEW_CONTROLLER_H__
#define THOR_MANG_FOOTSTEP_PREVIEW_CONTROLLER_H__

#include <math.h>

#include <ros/ros.h>
#include <tf/tf.h>

// ros control
#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>

// THOR-OP includes
#include <framework/Thor.h>

#include <vigir_footstep_planning_lib/helper.h>
#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>

#include <thor_mang_footstep_planning_msgs/thor_mang_step_plan_msg_plugin.h>
#include <hardware_interface/joint_command_interface.h>

#include <dynamic_reconfigure/server.h>
#include <thor_mang_ros_control/FootstepPreviewControllerConfig.h>

#include <queue>



namespace Thor
{
class ThorMangFootstepPreviewController
  : public controller_interface::Controller<hardware_interface::PositionJointInterface>
  , public MotionModule
{
public:
  ThorMangFootstepPreviewController();

  // ros control
  bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& nh);

  void update(const ros::Time& time, const ros::Duration& period);

  void starting(const ros::Time& time);
  void stopping(const ros::Time& time);

  // THOR-OP framework
  void Initialize();
  void Process();

  void StartWalking();

protected:
  void InitImuData();
  void addImuData();
  void InitFtDataOnGround();
  void addFtData();

  void initWalkingParameters();
  void claimJoints();
  void unclaimJoints();

  bool claim_arms;
  double hip_pitch_offset;
  double ankle_pitch_offset;
  double walk_stabilizer_gain_ratio;
  double imu_gyro_gain_ratio;
  double force_moment_distribution_ratio;
  double balance_hip_pitch_gain;
  double balance_z_gain_by_ft;
  double balance_right_roll_gain_by_ft;
  double balance_right_pitch_gain_by_ft;
  double balance_left_roll_gain_by_ft;
  double balance_left_pitch_gain_by_ft;
  double foot_landing_offset_gain;
  double foot_landing_detect_n;

  // action server calls
  typedef vigir_footstep_planning::SimpleActionServer<vigir_footstep_planning::msgs::ExecuteStepPlanAction> ActionServer;
  void executeStepPlanAction(ActionServer::Ptr& as);
  void stepPlanPreempted();

  int remaining_steps;
  int total_steps;
  bool goal_waiting;

  // action servers
  ActionServer::Ptr execute_step_plan_as;

  //dyn_reconfigure_callback
  void dynRecParamCallback(thor_mang_ros_control::FootstepPreviewControllerConfig &config, uint32_t level);

  // time measurement to get current rate
  ros::Time last_call;
  double system_control_unit_time_sec;

	typedef dynamic_reconfigure::Server<thor_mang_ros_control::FootstepPreviewControllerConfig> FootstepPreviewConfigServer;
  boost::shared_ptr<FootstepPreviewConfigServer> dyn_rec_server_;

  // Sensor resetting
  bool imu_resetted;
  bool ft_resetted;
  unsigned int current_imu_measurements;
  unsigned int current_ft_measurements;
  unsigned int max_imu_measurements;
  unsigned int max_ft_measurements;
  double imu_bias[2];
  double ft_bias_on_ground[12];


};
}

#endif
