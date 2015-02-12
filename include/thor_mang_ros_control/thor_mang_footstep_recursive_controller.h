//=================================================================================================
// Copyright (c) 2013, Stefan Kohlbrecher, TU Darmstadt
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

#ifndef THOR_MANG_FOOTSTEP_RECURSIVE_CONTROLLER_H__
#define THOR_MANG_FOOTSTEP_RECURSIVE_CONTROLLER_H__

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

#include <thor_mang_footstep_planner/thor_mang_step_plan_msg_plugin.h>
#include <thor_mang_ros_control/thor_mang_footstep_interface.h>



namespace Thor
{
class ThorMangFootstepRecursiveController
  : public controller_interface::Controller<hardware_interface::ThorMangFootstepInterface>
  , public MotionModule
{
public:
  ThorMangFootstepRecursiveController();

  // ros control
  bool init(hardware_interface::ThorMangFootstepInterface* hw, ros::NodeHandle& nh);

  void update(const ros::Time& time, const ros::Duration& period);

  void starting(const ros::Time& time);
  void stopping(const ros::Time& time);

  // THOR-OP framework
  void Initialize();
  void Process();

  void StartWalking();

protected:
  void InitImuData();
  void InitFtDataOnGround();

  void InitWalking();
  void SetMotionEnableList();

  // action server calls
  void executeStepPlanAction(vigir_footstep_planning::SimpleActionServer<vigir_footstep_planning::msgs::ExecuteStepPlanAction>::Ptr& as);

  // action servers
  vigir_footstep_planning::SimpleActionServer<vigir_footstep_planning::msgs::ExecuteStepPlanAction>::Ptr execute_step_plan_as;

  hardware_interface::ThorMangFootstepsHandle footsteps_handle;

  std::vector<StepData> last_step_data_list;

  // time measurement to get current rate
  ros::Time last_call;
  double system_control_unit_time_sec;
};
}

#endif
