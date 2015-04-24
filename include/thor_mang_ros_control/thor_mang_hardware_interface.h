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

#ifndef THOR_MANG_INTERFACE_H_
#define THOR_MANG_INTERFACE_H_

#include <ros/ros.h>
#include <tf/tf.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

// Dynamixel Pro Driver
//#include <dynamixel_pro_driver/dynamixel_pro_driver.h>

// THOR-OP includes
#include <framework/Thor.h>
#include <motion/motionmodule.h>

// ROS Control includes
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/robot_hw.h>
#include <thor_mang_ros_control/thor_mang_footstep_interface.h>

// FT tools lib
#include <vigir_force_torque_compensation_lib/compensation.h>

// Robot State
#include <robot_transforms/robot_transforms.h>

#define G_ACC 9.80665



namespace Thor
{
class ThorMangHardwareInterface
  : public MotionModule
  , public hardware_interface::RobotHW
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  enum JointIDs
  {
    R_SHOULDER_PITCH      =  1,
    L_SHOULDER_PITCH      =  2,
    R_SHOULDER_ROLL       =  3,
    L_SHOULDER_ROLL       =  4,
    R_SHOULDER_YAW        =  5,
    L_SHOULDER_YAW        =  6,
    R_ELBOW               =  7,
    L_ELBOW               =  8,
    R_WRIST_YAW1          =  9,
    L_WRIST_YAW1          = 10,
    R_WRIST_ROLL          = 11,
    L_WRIST_ROLL          = 12,
    R_WRIST_YAW2          = 13,
    L_WRIST_YAW2          = 14,
    R_HIP_YAW             = 15,
    L_HIP_YAW             = 16,
    R_HIP_ROLL            = 17,
    L_HIP_ROLL            = 18,
    R_HIP_PITCH           = 19,
    L_HIP_PITCH           = 20,
    R_KNEE                = 21,
    L_KNEE                = 22,
    R_ANKLE_PITCH         = 23,
    L_ANKLE_PITCH         = 24,
    R_ANKLE_ROLL          = 25,
    L_ANKLE_ROLL          = 26,
    WAIST_PAN             = 27,
    WAIST_TILT            = 28,
    HEAD_PAN              = 29,
    HEAD_TILT             = 30,
    R_HAND_THUMB          = 31,
    L_HAND_THUMB          = 32,
    R_HAND_INDEX_FINGER   = 33,
    L_HAND_INDEX_FINGER   = 34,
    R_HAND_MIDDLE_FINGER  = 35,
    L_HAND_MIDDLE_FINGER  = 36,
    WAIST_LIDAR           = 37
  };

  enum ftSensorIndex
  {
    R_ARM                         = 0,
    L_ARM                         = 1,
    R_LEG                         = 2,
    L_LEG                         = 3,
    MAXIMUM_NUMBER_OF_FT_SENSORS  = 4
  };

  ~ThorMangHardwareInterface();

  // THOR-OP framework
  void Initialize();
  void Process();

  // Ros Control
  void read(ros::Time time, ros::Duration period);
  void write(ros::Time time, ros::Duration period);

  // interfaces
  static boost::mutex& getDynamixelMutex();

  void setJointStateRate(double joint_state_rate);

  void setTorqueOn(JointData& joint, bool enable);
  void setTorqueOn(int id, bool enable);
  void setTorqueOn(bool enable);

  // typedefs
  typedef boost::shared_ptr<ThorMangHardwareInterface> Ptr;
  typedef boost::shared_ptr<const ThorMangHardwareInterface> ConstPtr;

  static ThorMangHardwareInterface::Ptr& Instance();

  // UIDs of joints and sensors
  static const std::string jointUIDs[MotionStatus::MAXIMUM_NUMBER_OF_JOINTS-1];
  static const std::string ftSensorUIDs[MAXIMUM_NUMBER_OF_FT_SENSORS];

  void resetFtSensor(unsigned int sensor_id);

protected:
  ThorMangHardwareInterface();
  ThorMangHardwareInterface(ThorMangHardwareInterface const&);

  ThorMangHardwareInterface& operator=(ThorMangHardwareInterface const&);

  JointData* getJoint(int id);

  // Robot bringup
  bool robotBringUp();
  bool goReadyPose();

  int setIndirectAddress(unsigned int joint_index);
  void initJointPosition(unsigned int joint_index, int value);
  void initINS();
  void InitForceTorque();

  void update_force_torque_compensation();
  void update_force_torque_sensors();
  void compensate_force_torque(int ft_sensor_index);

  static ThorMangHardwareInterface::Ptr singelton;

  /** joint offsets from ROS zero to Robotis zero
  /* Robotis zero: "ready stand pose"
   * ROS zero: "fully extended arms/legs"
   **/
  static const int ros_joint_offsets[MotionStatus::MAXIMUM_NUMBER_OF_JOINTS-1];

  // parameters
  double joint_state_intervall;
  ros::Time last_joint_state_read;

  // mutex
  mutable boost::mutex dynamixel_mutex;

  // INS
  boost::shared_ptr<Ins> ins;

  // ros controll stuff
  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::PositionJointInterface pos_joint_interface;

  hardware_interface::ImuSensorInterface imu_sensor_interface;
  hardware_interface::ForceTorqueSensorInterface force_torque_sensor_interface;

  hardware_interface::ThorMangFootstepInterface footstep_interface;

  double cmd[MotionStatus::MAXIMUM_NUMBER_OF_JOINTS-1]; // todo: replace with std::map<std::string, double>
  double pos[MotionStatus::MAXIMUM_NUMBER_OF_JOINTS-1]; // todo: replace with std::map<std::string, double>
  double vel[MotionStatus::MAXIMUM_NUMBER_OF_JOINTS-1]; // todo: replace with std::map<std::string, double>
  double eff[MotionStatus::MAXIMUM_NUMBER_OF_JOINTS-1]; // todo: replace with std::map<std::string, double>

  // IMU
  hardware_interface::ImuSensorHandle::Data imu_data;
  double imu_orientation[4];
  double imu_angular_velocity[3];
  double imu_linear_acceleration[3];

  // FT-Sensors
  double force_raw[MAXIMUM_NUMBER_OF_FT_SENSORS][3];
  double torque_raw[MAXIMUM_NUMBER_OF_FT_SENSORS][3];
  double force_compensated[MAXIMUM_NUMBER_OF_FT_SENSORS][3];
  double torque_compensated[MAXIMUM_NUMBER_OF_FT_SENSORS][3];

  // Zero
  FTCompensation::Vector6d force_torque_offset[MAXIMUM_NUMBER_OF_FT_SENSORS];
  unsigned int num_ft_measurements[MAXIMUM_NUMBER_OF_FT_SENSORS];
  bool has_ft_offsets[MAXIMUM_NUMBER_OF_FT_SENSORS];

  FTCompensation::Compensation ft_compensation[MAXIMUM_NUMBER_OF_FT_SENSORS];

  // Robot Transforms
  robot_tools::RobotTransforms robot_transforms;
};
}

#endif

