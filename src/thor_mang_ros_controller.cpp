//=================================================================================================
// Copyright (c) 2014, Alexander Stumpf, TU Darmstadt
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

#include <thor_mang_ros_control/thor_mang_ros_controller.h>



namespace Thor
{
ThorMangRosControllerNode::ThorMangRosControllerNode()
{
  ros::NodeHandle nh;

  // init controller
  std::string thor_mang_ini_file;
  nh.param("thor_mang_ros_controller/thor_mang_ini_file", thor_mang_ini_file, std::string(""));

  double joint_state_rate;
  nh.param("joint_state_controller/publish_rate", joint_state_rate, 50.0);

  // Initialize THOR-MANG Framework
  if(Thor::MotionManager::GetInstance()->Initialize() == true)
  {
    ThorMangHardwareInterface::Instance()->setJointStateRate(joint_state_rate);
    Thor::MotionManager::GetInstance()->AddModule(ThorMangHardwareInterface::Instance().get());

    minIni* ini = new minIni(thor_mang_ini_file);
    Thor::MotionManager::GetInstance()->LoadINISettings(ini);
    delete ini;

    //Thor::MotionManager::GetInstance()->StartTimer(); // sadly crashes
  }
  else
  {
    ROS_ERROR("Initializing Motion Manager failed!");
    ThorMangHardwareInterface::Instance()->Initialize();
  }

  // Initialize Ros Control
  controller_manager.reset(new controller_manager::ControllerManager(ThorMangHardwareInterface::Instance().get(), nh));

  // subscribe topics
  torque_on_sub = nh.subscribe("torque_on", 1, &ThorMangRosControllerNode::setTorqueOn, this);

  for (unsigned int sensor_id = 0; sensor_id < ThorMangHardwareInterface::MAXIMUM_NUMBER_OF_FT_SENSORS; sensor_id++) {
      reset_ft_sub[sensor_id] = nh.subscribe<std_msgs::Empty>(
                  "reset_ft/" + ThorMangHardwareInterface::ftSensorUIDs[sensor_id], 1, boost::bind(&ThorMangRosControllerNode::resetFtSensor, this, _1, sensor_id));
  }

  ROS_INFO("Initialization of ros controller completed!");
}

ThorMangRosControllerNode::~ThorMangRosControllerNode()
{
}

void ThorMangRosControllerNode::setTorqueOn(std_msgs::BoolConstPtr enable)
{
  ThorMangHardwareInterface::Instance()->setTorqueOn(enable->data);
}

void ThorMangRosControllerNode::resetFtSensor(const std_msgs::EmptyConstPtr &empty_ptr, unsigned int sensor_id) {
  ThorMangHardwareInterface::Instance()->resetFtSensor(sensor_id);
}

void ThorMangRosControllerNode::update(ros::Time time, ros::Duration period)
{
  // manually updates motion manager and modules of THOR-MANG
  {
    // manually lock dynamixel access for thor mang framework
    boost::mutex::scoped_lock lock(ThorMangHardwareInterface::Instance()->getDynamixelMutex());
    Thor::MotionManager::GetInstance()->Process();
  }

  // ros control update cycle
  ThorMangHardwareInterface::Instance()->read(time, period);
  controller_manager->update(time, period);
  ThorMangHardwareInterface::Instance()->write(time, period);
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "thor_mang_ros_controller");

  ros::NodeHandle nh;
  double control_rate;
  nh.param("thor_mang_ros_controller/control_rate", control_rate, 125.0);

  Thor::ThorMangRosControllerNode thor_mang_ros_controller_node;

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::Time last_time = ros::Time::now();
  ros::Rate rate(control_rate);

  while (ros::ok())
  {
    rate.sleep();

    ros::Time current_time = ros::Time::now();
    ros::Duration elapsed_time = current_time - last_time;
    last_time = current_time;

    thor_mang_ros_controller_node.update(current_time, elapsed_time);
  }

  return 0;
}
