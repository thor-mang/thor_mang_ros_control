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

#ifndef THOR_MANG_ROS_CONTROLLER_H_
#define THOR_MANG_ROS_CONTROLLER_H_

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

#include <controller_manager/controller_manager.h>
#include <thor_mang_ros_control/thor_mang_hardware_interface.h>



namespace Thor
{
class ThorMangRosControllerNode
{
public:
  ThorMangRosControllerNode(bool torque_on = true);
  ~ThorMangRosControllerNode();

  void update(ros::Time time, ros::Duration period);

protected:
  void setTorqueOn(std_msgs::BoolConstPtr enable);
  void enableLights(std_msgs::BoolConstPtr enable);
  void resetFtSensor(const std_msgs::EmptyConstPtr& empty_ptr, unsigned int sensor_id);
  void measureFtScaling(const std_msgs::EmptyConstPtr& empty_ptr);

  // subscriber
  ros::Subscriber torque_on_sub;
  ros::Subscriber enable_lights_sub;
  ros::Subscriber reset_ft_sub[ThorMangHardwareInterface::MAXIMUM_NUMBER_OF_FT_SENSORS];
  ros::Subscriber measure_scaling_sub;

  boost::shared_ptr<controller_manager::ControllerManager> controller_manager;
};
}

#endif

