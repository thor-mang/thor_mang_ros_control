#include <thor_mang_ros_control/thor_mang_hardware_interface.h>

#include <algorithm>
#include <cmath>

namespace Thor
{
const std::string ThorMangHardwareInterface::jointUIDs[MotionStatus::MAXIMUM_NUMBER_OF_JOINTS-1] =
{
  "r_shoulder_pitch",
  "l_shoulder_pitch",
  "r_shoulder_roll",
  "l_shoulder_roll",
  "r_shoulder_yaw",
  "l_shoulder_yaw",
  "r_elbow",
  "l_elbow",
  "r_wrist_yaw1",
  "l_wrist_yaw1",
  "r_wrist_roll",
  "l_wrist_roll",
  "r_wrist_yaw2",
  "l_wrist_yaw2",
  "r_hip_yaw",
  "l_hip_yaw",
  "r_hip_roll",
  "l_hip_roll",
  "r_hip_pitch",
  "l_hip_pitch",
  "r_knee",
  "l_knee",
  "r_ankle_pitch",
  "l_ankle_pitch",
  "r_ankle_roll",
  "l_ankle_roll",
  "waist_pan",
  "waist_tilt",
  "head_pan",
  "head_tilt",
  "r_f0_j0",        // r_hand_thumb
  "l_f0_j0",        // l_hand_thumb
  "r_f1_j0",        // r_hand_index_finger
  "l_f1_j0",        // l_hand_index_finger
  "r_hand_middle_finger",
  "l_hand_middle_finger",
  "waist_lidar"
};

const std::string ThorMangHardwareInterface::ftSensorUIDs[ThorMangHardwareInterface::MAXIMUM_NUMBER_OF_FT_SENSORS] =
{
  "r_hand",
  "l_hand",
  "r_foot",
  "l_foot"
};

const int ThorMangHardwareInterface::ros_joint_offsets[MotionStatus::MAXIMUM_NUMBER_OF_JOINTS-1] =
{
  0,        // r_shoulder_pitch
  0,        // l_shoulder_pitch
  -125500,  // r_shoulder_roll
  125500,   // l_shoulder_roll
  125500,   // r_shoulder_yaw
  -125500,  // l_shoulder_yaw
  -62750,   // r_elbow
  62750,    // l_elbow
  0,        // r_wrist_yaw1
  0,        // l_wrist_yaw1
  0,        // r_wrist_roll
  0,        // l_wrist_roll
  0,        // r_wrist_yaw2
  0,        // l_wrist_yaw2
  0,        // r_hip_yaw
  0,        // l_hip_yaw
  0,        // r_hip_roll
  0,        // l_hip_roll
  0,        // r_hip_pitch
  0,        // l_hip_pitch
  62750,    // r_knee
  -62750,   // l_knee
  0,        // r_ankle_pitch
  0,        // l_ankle_pitch
  0,        // r_ankle_roll
  0,        // l_ankle_roll
  0,        // waist_pan
  0,        // waist_tilt
  0,        // head_pan
  0,        // head_tilt
  0,        // r_hand_thumb
  0,        // l_hand_thumb
  0,        // r_hand_index_finger
  0,        // l_hand_index_finger
  0,        // r_hand_middle_finger
  0,        // l_hand_middle_finger
  0         // waist_lidar
};

ThorMangHardwareInterface::Ptr ThorMangHardwareInterface::singelton = ThorMangHardwareInterface::Ptr();

ThorMangHardwareInterface::ThorMangHardwareInterface()
  : MotionModule()
  , hardware_interface::RobotHW()
  , joint_state_interval(20.0)
  , last_joint_state_read(ros::Time::now())
  , has_foot_ft_offsets_in_air(true)
{
  uID = const_cast<char*>("thor_mang_hardware_interface");

  for (unsigned int i = 0; i < MAXIMUM_NUMBER_OF_FT_SENSORS; i++)
  {
    has_ft_offsets[i] = true;
    num_ft_measurements[i] = 0;
    force_torque_offset[i] = FTCompensation::Vector6d::Zero();
  }

  ros::NodeHandle nh("joint_offset_calibration");
  dyn_rec_server_.reset(new HardwareInterfaceConfigServer(nh));
  dyn_rec_server_->setCallback(boost::bind(&ThorMangHardwareInterface::dynRecParamCallback, this, _1, _2));

  ros::NodeHandle nh_servo_gains("servo_gains_configuration");
  dyn_rec_servo_gains_server_.reset(new ServoGainsConfigServer(nh_servo_gains));
  dyn_rec_servo_gains_server_->setCallback(boost::bind(&ThorMangHardwareInterface::dynRecServoGainsConfigCallback, this, _1, _2));
}

ThorMangHardwareInterface::ThorMangHardwareInterface(ThorMangHardwareInterface const&)
{
}

ThorMangHardwareInterface::~ThorMangHardwareInterface()
{
  if (ins)
  {
    ins->StopSensingDataUpdate();
    ins->Disconnect();
  }
}

ThorMangHardwareInterface& ThorMangHardwareInterface::operator=(ThorMangHardwareInterface const&)
{
}

ThorMangHardwareInterface::Ptr& ThorMangHardwareInterface::Instance()
{
  if (!singelton)
    singelton.reset(new ThorMangHardwareInterface());
  return singelton;
}

void ThorMangHardwareInterface::Initialize()
{
  if (MotionStatus::m_CurrentJoints.size() != 0)
  {
    m_RobotInfo = MotionStatus::m_CurrentJoints;
    for (unsigned int joint_index = 0; joint_index < MotionStatus::m_CurrentJoints.size(); joint_index++)
    {
      int id_index = m_RobotInfo[joint_index].m_ID-1;
      m_RobotInfo[joint_index].m_Value -= MotionManager::GetInstance()->m_Offset[id_index];

    }
  }
  else
  {
    ROS_ERROR("MotionStatus is not initialized");
  }
  ROS_INFO("Initialize INS...");
  initINS();
  ROS_INFO("Initialize FT-Sensors...");
  InitForceTorque();

  /** register joints */

  std::set<unsigned int> found_ids;
  std::set<unsigned int> all_ids;
  for (unsigned int i = 1; i < MotionStatus::MAXIMUM_NUMBER_OF_JOINTS; i++) {
    all_ids.insert(i);
    pos[i-1] = 0;
  }

  // dispatching joints
  for (unsigned int joint_index = 0; joint_index < m_RobotInfo.size(); joint_index++)
  {
    if (m_RobotInfo[joint_index].m_ID > MotionStatus::MAXIMUM_NUMBER_OF_JOINTS-1)
    {
      ROS_WARN("Robot has joint with invalid id: %u", m_RobotInfo[joint_index].m_ID);
      continue;
    }

    unsigned int id_index = m_RobotInfo[joint_index].m_ID-1;
    found_ids.insert(id_index+1);

    // connect and register the joint state interface
    hardware_interface::JointStateHandle joint_state_handle(jointUIDs[id_index], &pos[id_index], &vel[id_index], &eff[id_index]);
    joint_state_interface.registerHandle(joint_state_handle);

    // connect and register the joint position interface
    hardware_interface::JointHandle joint_handle(joint_state_handle, &cmd[id_index]);
    pos_joint_interface.registerHandle(joint_handle);

    // activate control
    MotionStatus::m_EnableList[id_index].uID = uID;
  }
  // Register missing joints so controllers don't fail to load
  std::set<unsigned int> missing_ids;
  std::set_difference(all_ids.begin(), all_ids.end(), found_ids.begin(), found_ids.end(), std::inserter(missing_ids, missing_ids.begin()));
  std::stringstream error_stream;
  for (std::set<unsigned int>::const_iterator it = missing_ids.begin(); it != missing_ids.end(); it++) {

    unsigned int id_index = *it -1;

    if ((jointUIDs[id_index] == "r_hand_middle_finger") || (jointUIDs[id_index] == "l_hand_middle_finger"))
      continue;

    error_stream << jointUIDs[id_index] << ",";
    // connect and register the joint state interface
    hardware_interface::JointStateHandle joint_state_handle(jointUIDs[id_index], &pos[id_index], &vel[id_index], &eff[id_index]);
    joint_state_interface.registerHandle(joint_state_handle);

    // connect and register the joint position interface
    hardware_interface::JointHandle joint_handle(joint_state_handle, &cmd[id_index]);
    pos_joint_interface.registerHandle(joint_handle);
  }
  ROS_ERROR_STREAM("Following joints are missing: " << error_stream.str());

  registerInterface(&joint_state_interface);
  registerInterface(&pos_joint_interface);

  /** register sensors */
  // IMU
  imu_data.name = "pelvis_imu";
  imu_data.frame_id = "pelvis";
  imu_data.orientation = imu_orientation;
  imu_data.angular_velocity = imu_angular_velocity;
  imu_data.linear_acceleration = imu_linear_acceleration;
  hardware_interface::ImuSensorHandle imu_sensor_handle(imu_data);
  imu_sensor_interface.registerHandle(imu_sensor_handle);
  registerInterface(&imu_sensor_interface);

  // FT-Sensors
  for (unsigned int sensorIndex = 0; sensorIndex < MAXIMUM_NUMBER_OF_FT_SENSORS; sensorIndex++)
  {
    hardware_interface::ForceTorqueSensorHandle force_torque_sensor_handle_raw(ftSensorUIDs[sensorIndex] + "_raw", ftSensorUIDs[sensorIndex], force_raw[sensorIndex], torque_raw[sensorIndex]);
    force_torque_sensor_interface.registerHandle(force_torque_sensor_handle_raw);

    hardware_interface::ForceTorqueSensorHandle force_torque_sensor_handle_compensated(ftSensorUIDs[sensorIndex], ftSensorUIDs[sensorIndex], force_compensated[sensorIndex], torque_compensated[sensorIndex]);
    force_torque_sensor_interface.registerHandle(force_torque_sensor_handle_compensated);
  }
  registerInterface(&force_torque_sensor_interface);

  // load compensation data from parameter server
  for (unsigned int sensorIndex = 0; sensorIndex < MAXIMUM_NUMBER_OF_FT_SENSORS; sensorIndex++)
  {
    ros::NodeHandle nh(ftSensorUIDs[sensorIndex]);
    if (!(ft_compensation[sensorIndex].loadMassComBias(nh) && ft_compensation[sensorIndex].loadHandToSensorOffset(nh, "sensor_offset")))
      ROS_WARN_STREAM("Couldn't load complete ft sensor compensation data for " << ftSensorUIDs[sensorIndex] << " in " << nh.getNamespace() << ".");
    ft_compensation[sensorIndex].initGravityPublisher(ftSensorUIDs[sensorIndex] + "_gravity", ftSensorUIDs[sensorIndex]);
  }

  // Init robot transforms and state estimation
  robot_transforms_ptr.reset(new robot_tools::RobotTransforms());
  robot_transforms_ptr->init();
  state_estimator.setRobotTransforms(robot_transforms_ptr);
  state_estimator.init(ros::NodeHandle("state_estimator"));

  ros::NodeHandle nh;
  joint_cmds_pub_ = nh.advertise<sensor_msgs::JointState>("joint_cmds", 1000);
}

void ThorMangHardwareInterface::Process()
{
  //  boost::mutex::scoped_lock lock(hardware_mutex);
  //  m_RobotInfo[m_Pan_joint_index].m_Value = m_RobotInfo[m_Pan_joint_index].m_DXLInfo->Rad2Value(cmd[m_Pan_joint_index]);
  //  m_RobotInfo[m_Tilt_joint_index].m_Value = m_RobotInfo[m_Tilt_joint_index].m_DXLInfo->Rad2Value(cmd[m_Tilt_joint_index]);
  //  m_RobotInfo[m_Tilt_joint_index].m_Pgain = 8;
}

void ThorMangHardwareInterface::read(ros::Time time, ros::Duration period)
{
  for (unsigned int i = 0; i < MotionStatus::MAXIMUM_NUMBER_OF_JOINTS-1; i++)
    cmd[i] = std::numeric_limits<double>::quiet_NaN();

  // read joint values
  for (unsigned int joint_index = 0; joint_index < MotionStatus::m_CurrentJointsStatus.size(); joint_index++)
  {
    int id_index = MotionStatus::m_CurrentJointsStatus[joint_index].m_ID-1;
    pos[id_index] = MotionStatus::m_CurrentJointsStatus[joint_index].m_DXLInfo->Value2Rad(MotionStatus::m_CurrentJointsStatus[joint_index].m_Value - ros_joint_offsets[id_index]) + calibration_joint_offsets[id_index];
  }

  // Update Robot state
  for (unsigned int i = 0; i < 30; i++) // iterate over all body joints
    robot_transforms_ptr->updateState(jointUIDs[i], pos[i]);

  // read IMU and transform it to pelvis frame
  tf::Quaternion imu_orient;
  imu_orient.setRPY(ins->GetEulerAngle().pitch, ins->GetEulerAngle().roll, -ins->GetEulerAngle().yaw);

  imu_orientation[0] = imu_orient.x();
  imu_orientation[1] = imu_orient.y();
  imu_orientation[2] = imu_orient.z();
  imu_orientation[3] = imu_orient.w();

  imu_angular_velocity[0] =  ins->GetGyroData().scaled_gyro[1];
  imu_angular_velocity[1] =  ins->GetGyroData().scaled_gyro[0];
  imu_angular_velocity[2] = -ins->GetGyroData().scaled_gyro[2];

  imu_linear_acceleration[0] =  ins->GetAccelData().scaled_accel[1] * G_ACC;
  imu_linear_acceleration[1] =  ins->GetAccelData().scaled_accel[0] * G_ACC;
  imu_linear_acceleration[2] = -ins->GetAccelData().scaled_accel[2] * G_ACC;

  // Update robot state root transform
  Eigen::Affine3d imu_orient_rot(Eigen::Quaternion<double>(imu_orientation[3], imu_orientation[0], imu_orientation[1], imu_orientation[2]));
  imu_orient_rot.translation()  = Eigen::Vector3d::Zero();
  robot_transforms_ptr->updateRootTransform(imu_orient_rot);

  // read FT-Sensors
  force_raw[R_ARM][0] = -MotionStatus::R_ARM_FX;
  force_raw[R_ARM][1] = MotionStatus::R_ARM_FY;
  force_raw[R_ARM][2] = MotionStatus::R_ARM_FZ;
  torque_raw[R_ARM][0] = -MotionStatus::R_ARM_TX;
  torque_raw[R_ARM][1] = MotionStatus::R_ARM_TY;
  torque_raw[R_ARM][2] = MotionStatus::R_ARM_TZ;

  force_raw[L_ARM][0] = -MotionStatus::L_ARM_FX;
  force_raw[L_ARM][1] = MotionStatus::L_ARM_FY;
  force_raw[L_ARM][2] = MotionStatus::L_ARM_FZ;
  torque_raw[L_ARM][0] = -MotionStatus::L_ARM_TX;
  torque_raw[L_ARM][1] = MotionStatus::L_ARM_TY;
  torque_raw[L_ARM][2] = MotionStatus::L_ARM_TZ;

  force_raw[R_LEG][0] = -MotionStatus::R_LEG_FX;
  force_raw[R_LEG][1] = MotionStatus::R_LEG_FY;
  force_raw[R_LEG][2] = MotionStatus::R_LEG_FZ;
  torque_raw[R_LEG][0] = -MotionStatus::R_LEG_TX;
  torque_raw[R_LEG][1] = MotionStatus::R_LEG_TY;
  torque_raw[R_LEG][2] = MotionStatus::R_LEG_TZ;

  force_raw[L_LEG][0] = -MotionStatus::L_LEG_FX;
  force_raw[L_LEG][1] = MotionStatus::L_LEG_FY;
  force_raw[L_LEG][2] = MotionStatus::L_LEG_FZ;
  torque_raw[L_LEG][0] = -MotionStatus::L_LEG_TX;
  torque_raw[L_LEG][1] = MotionStatus::L_LEG_TY;
  torque_raw[L_LEG][2] = MotionStatus::L_LEG_TZ;

  // apply compensation
  update_force_torque_compensation();
  update_force_torque_sensors();

  state_estimator.setIMU(ins->GetEulerAngle().pitch, ins->GetEulerAngle().roll, -ins->GetEulerAngle().yaw);
  state_estimator.setFeetForceZ(force_compensated[L_LEG][2], force_compensated[R_LEG][2]);
  state_estimator.update();
}

// not hard real-time safe
void ThorMangHardwareInterface::publishJointCmds() {
  sensor_msgs::JointState joint_cmds;
  joint_cmds.header.stamp = ros::Time::now();

  joint_cmds.name.resize(m_RobotInfo.size());

  joint_cmds.position.resize(m_RobotInfo.size());
  joint_cmds.effort.resize(m_RobotInfo.size(), 0);
  joint_cmds.velocity.resize(m_RobotInfo.size(), 0);

  for (unsigned int i = 0; i < m_RobotInfo.size(); i++) {
    joint_cmds.position[i] = m_RobotInfo[i].m_Value;
    joint_cmds.name[i] = jointUIDs[m_RobotInfo[i].m_ID -1];
  }
  joint_cmds_pub_.publish(joint_cmds);
}

void ThorMangHardwareInterface::write(ros::Time time, ros::Duration period)
{
  if (m_RobotInfo.size() == 0)
    return;

  for (unsigned int joint_index = 0; joint_index < m_RobotInfo.size(); joint_index++)
  {
    int id_index = m_RobotInfo[joint_index].m_ID-1;

    if (cmd[id_index] != cmd[id_index]) //checks that cmd[id_index] is not nan
      continue;

    if (m_RobotInfo[joint_index].m_ID < 1 || m_RobotInfo[joint_index].m_ID > MotionStatus::MAXIMUM_NUMBER_OF_JOINTS-1)
      continue;

    m_RobotInfo[joint_index].m_Value = m_RobotInfo[joint_index].m_DXLInfo->Rad2Value(cmd[id_index] - calibration_joint_offsets[id_index]) + ros_joint_offsets[id_index];
  }
  if (joint_cmds_pub_.getNumSubscribers() != 0) {
    publishJointCmds();
  }
}

void ThorMangHardwareInterface::setJointStateRate(double joint_state_rate)
{
  this->joint_state_interval = 1.0/joint_state_rate;
}

void ThorMangHardwareInterface::setTorqueOn(int id, bool enable)
{
  JointData* joint;
  if (joint = getJoint(id))
    MotionManager::GetInstance()->SetTorqueOn(*joint, enable);
  else
    ROS_ERROR("[setTorqueOn] No joint with ID %i available!", id);
}

void ThorMangHardwareInterface::setTorqueOn(bool enable)
{
  if (enable)
    ROS_WARN("Enable torque!");
  else
    ROS_WARN("Disable torque!");

  MotionManager::GetInstance()->SetTorqueOn(enable);
}

void ThorMangHardwareInterface::setLightsEnabled(bool enable)
{
  if (enable)
    ROS_INFO("Enable lights!");
  else
    ROS_INFO("Disable lights!");

  MotionManager::GetInstance()->EnableLights(enable);
}

JointData* ThorMangHardwareInterface::getJoint(int id)
{
  for (unsigned int joint_index = 0; joint_index < m_RobotInfo.size(); joint_index++)
  {
    if (m_RobotInfo[joint_index].m_ID == id)
      return &(m_RobotInfo[joint_index]);
  }
  return NULL;
}

void ThorMangHardwareInterface::limitJointSpeed(unsigned int limit) {
  // speed down servos
  for (unsigned int joint_index = 0; joint_index < m_RobotInfo.size(); joint_index++)
  {
    setJointVelocity(joint_index, limit);
    setJointAcceleration(joint_index, 4);

    usleep(1000);
  }
}

void ThorMangHardwareInterface::unlimitJointSpeed() {
  for (unsigned int joint_index = 0; joint_index < m_RobotInfo.size(); joint_index++)
  {
    setJointVelocity(joint_index, 0); // remove limits
    setJointAcceleration(joint_index, 0);

    usleep(1000);
  }
}

bool ThorMangHardwareInterface::goReadyPose()
{
  ROS_WARN("Going to ready pose!");
  limitJointSpeed(2000);

  // compute trajectory
  // ROS_INFO("Compute trajectory to initial pose.");

  int dir_output[16];
  double InitAngle[16];

  //for thor
  dir_output[0] = -1; dir_output[1] = -1; dir_output[2] = -1; dir_output[3] = -1; dir_output[4] =  1; dir_output[5] = 1;
  dir_output[6] = -1; dir_output[7] = -1; dir_output[8] =  1; dir_output[9] =  1; dir_output[10]= -1; dir_output[11] = 1;
  dir_output[12] = -1; dir_output[13] = 1;  dir_output[14] = -1; dir_output[15] =  1;
  InitAngle[0]  =   0.0;  InitAngle[1]  =  0.0;  InitAngle[2]  =  5.7106;  InitAngle[3] =  33.5788; InitAngle[4]  = -5.7106; InitAngle[5]  = 0.0;
  InitAngle[6]  =   0.0;  InitAngle[7]  =  0.0;  InitAngle[8]  = -5.7106;  InitAngle[9] = -33.5788; InitAngle[10] =  5.7106; InitAngle[11] = 0.0;
  InitAngle[12] = -45.0,  InitAngle[13] = 45.0;  InitAngle[14] =  45.0;    InitAngle[15] =  -45.0;

  double angle[16];
  int outValue[16];

  matd GtoCOB = GetTransformMatrix(0, 0, 650.0, 0, 0, 0 );
  matd GtoRF = GetTransformMatrix(0, -125.0, 0, 0, 0, 0);
  matd GtoLF = GetTransformMatrix(0,  125.0, 0, 0, 0, 0);

  matd RHtoCOB = GetTranslationMatrix(0,  Kinematics::LEG_SIDE_OFFSET*0.5, 0);
  matd LHtoCOB = GetTranslationMatrix(0, -Kinematics::LEG_SIDE_OFFSET*0.5, 0);

  matd COBtoG = GetTransformMatrixInverse(GtoCOB);
  matd RHtoRF = RHtoCOB*COBtoG*GtoRF;
  matd LHtoLF = LHtoCOB*COBtoG*GtoLF;

  Pose3D epr, epl;

  epr = GetPose3DfromTransformMatrix(RHtoRF);
  epl = GetPose3DfromTransformMatrix(LHtoLF);

  if (PreviewControlWalking::GetInstance()->computeIK(&angle[0], epr.x, epr.y, epr.z+Kinematics::LEG_LENGTH, epr.roll, epr.pitch, epr.yaw) == false)
  {
    ROS_ERROR("[Ready Pose] Right leg IKsolve failed");
    return false;
  }

  if (PreviewControlWalking::GetInstance()->computeIK(&angle[6], epl.x, epl.y, epl.z+Kinematics::LEG_LENGTH, epl.roll, epl.pitch, epl.yaw) == false)
  {
    ROS_ERROR("[Ready Pose] Left leg IKsolve failed");
    return false;
  }

  for (int idx = 0; idx < 6; idx++)
  {
    angle[idx] = (double)dir_output[idx]*angle[idx]*180.0/M_PI + InitAngle[idx];
    angle[idx+6] = (double)dir_output[idx+6]*angle[idx+6]*180.0/M_PI + InitAngle[idx+6];
  }


  for (int idx = 0; idx < 16; idx++)
  {
    outValue[idx] = 251000.0*(angle[idx])/180.0;
  }

  double gdHipPitchOffset = 11.0;
  outValue[2] -= (double)dir_output[2] * gdHipPitchOffset * 251000.0/180.0;
  outValue[8] -= (double)dir_output[8] * gdHipPitchOffset * 251000.0/180.0;

  for (unsigned int joint_index = 0; joint_index < m_RobotInfo.size(); joint_index++)
  {
    int id = m_RobotInfo[joint_index].m_ID;

    if (id >= 15 && id <= 26)
      setVelocityIGain(joint_index, 0);

    usleep(1000);
  }

  // let's move now
  ROS_WARN("Moving to ready pose now!");

  // Close hands
  for (unsigned int joint_index = 0; joint_index < m_RobotInfo.size(); joint_index++)
  {
    int id = m_RobotInfo[joint_index].m_ID;

    if (id == 31)
      setJointPosition(joint_index, 3280 - MotionManager::GetInstance()->m_Offset[id-1]);
    else if (id == 32)
      setJointPosition(joint_index, 3500 - MotionManager::GetInstance()->m_Offset[id-1]);
    else if (id == 33)
      setJointPosition(joint_index, 3600 - MotionManager::GetInstance()->m_Offset[id-1]);
    else if (id == 34)
      setJointPosition(joint_index, 3850 - MotionManager::GetInstance()->m_Offset[id-1]);
  }
  usleep(1000000); // 1 second

  // Move arms outwards to prevent self-collisions
  for (unsigned int joint_index = 0; joint_index < m_RobotInfo.size(); joint_index++)
  {
    int id = m_RobotInfo[joint_index].m_ID;

    if (id == 3)
      setJointPosition(joint_index, -62750);
    else if (id == 4)
      setJointPosition(joint_index, 62750);

  }
  usleep(3000000); // 3 seconds

  // Move arms to ready state
  for (unsigned int joint_index = 0; joint_index < m_RobotInfo.size(); joint_index++)
  {
    int id = m_RobotInfo[joint_index].m_ID;

    if (id == 5)
      setJointPosition(joint_index, 125500);
    else if (id == 6)
      setJointPosition(joint_index, -125500);
    else if (id == 7)
      setJointPosition(joint_index, 62750);
    else if (id == 8)
      setJointPosition(joint_index, -62750);
    else if (id == 9)
      setJointPosition(joint_index, -75000);
    else if (id == 10)
      setJointPosition(joint_index,  75000);
    else if (id == 11)
      setJointPosition(joint_index, 0);
    else if (id == 12)
      setJointPosition(joint_index, 0);
    else if (id == 13)
      setJointPosition(joint_index, 0);
    else if (id == 14)
      setJointPosition(joint_index, 0);
  }

  usleep(3000000); // 3 seconds

  // Move rest to ready pose
  for (unsigned int joint_index = 0; joint_index < m_RobotInfo.size(); joint_index++)
  {
    int id = m_RobotInfo[joint_index].m_ID;

    if (id == 1)
      setJointPosition(joint_index, -62750);
    else if (id == 2)
      setJointPosition(joint_index, 62750);
    else if (id == 3)
      setJointPosition(joint_index, -109520);
    else if (id == 4)
      setJointPosition(joint_index, 109520);

    else if (id == 27)
      setJointPosition(joint_index, 0);
    else if (id == 28)
      setJointPosition(joint_index, 0);
    else if (id == 29)
      setJointPosition(joint_index, 0);
    else if (id == 30)
      setJointPosition(joint_index, 0);

    else if (id == 15)
      setJointPosition(joint_index, outValue[0]);
    else if (id == 17)
      setJointPosition(joint_index, outValue[1]);
    else if (id == 19)
      setJointPosition(joint_index, outValue[2] + 6000);
    else if (id == 21)
      setJointPosition(joint_index, outValue[3]);
    else if (id == 23)
      setJointPosition(joint_index, outValue[4] + 6000);
    else if (id == 25)
      setJointPosition(joint_index, outValue[5]);

    else if (id == 16)
      setJointPosition(joint_index, outValue[6]);
    else if (id == 18)
      setJointPosition(joint_index, outValue[7]);
    else if (id == 20)
      setJointPosition(joint_index, outValue[8] - 6000);
    else if (id == 22)
      setJointPosition(joint_index, outValue[9]);
    else if (id == 24)
      setJointPosition(joint_index, outValue[10] - 6000);
    else if (id == 26)
      setJointPosition(joint_index, outValue[11]);

    else if (id == 37)
      setJointPosition(joint_index, 2048);

    usleep(1000);
  }
  usleep(5000000); // 5

  // speed up servos again
  unlimitJointSpeed();

  return true;
}

void ThorMangHardwareInterface::setJointPosition(unsigned int joint_index, int value)
{
  int id = m_RobotInfo[joint_index].m_ID;
  MotionStatus::m_CurrentJoints[joint_index].m_Value = value + MotionManager::GetInstance()->m_Offset[id-1];
  m_RobotInfo[joint_index].m_Value = value;
  MotionManager::GetInstance()->WriteGoalPosition(MotionStatus::m_CurrentJoints[joint_index]);
}

void ThorMangHardwareInterface::setJointVelocity(unsigned int joint_index, int value)
{
  MotionManager::GetInstance()->WriteGoalVelocity(MotionStatus::m_CurrentJoints[joint_index], value);
}

void ThorMangHardwareInterface::setJointAcceleration(unsigned int joint_index, int value)
{
  MotionManager::GetInstance()->WriteGoalAcceleration(MotionStatus::m_CurrentJoints[joint_index], value);
}

void ThorMangHardwareInterface::setPositionPGain(unsigned int joint_index, int value) {
  m_RobotInfo[joint_index].m_Pgain = value;
}

void ThorMangHardwareInterface::setVelocityPGain(unsigned int joint_index, int value)
{
  MotionManager::GetInstance()->WriteVelocityPGain(MotionStatus::m_CurrentJoints[joint_index], value);
}

void ThorMangHardwareInterface::setVelocityIGain(unsigned int joint_index, int value)
{
  MotionManager::GetInstance()->WriteVelocityIGain(MotionStatus::m_CurrentJoints[joint_index], value);
}

void ThorMangHardwareInterface::initINS()
{
  imu_orientation[0] = imu_orientation[1] = imu_orientation[2] = 0.0;
  imu_orientation[3] = 1.0;
  imu_angular_velocity[0] = imu_angular_velocity[1] = imu_angular_velocity[2] = 0.0;
  imu_linear_acceleration[0] = imu_linear_acceleration[1] = imu_linear_acceleration[2] = 0.0;

  // in case of reinit
  if (ins)
  {
    ROS_INFO("Reinitialize INS!\n");
    ins->StopSensingDataUpdate();
    ins->Disconnect();
  }

  // init INS
  ins.reset(new Ins());
  if (ins)
  {
    if (ins->Connect("ttyACM0", 921600) != MIP_INTERFACE_OK)
      ROS_ERROR("Failed to connect INS!\n");
    else if (ins->Initialize() != MIP_INTERFACE_OK)
      ROS_ERROR("Failed to init INS!\n");
    else if (ins->SetEnableAHRSDataCallBack() != MIP_INTERFACE_OK)
      ROS_ERROR("Failed to init AHRS callback!\n");
    else
      ins->StartSensingDataUpdate();
  }
}

void ThorMangHardwareInterface::InitForceTorque()
{
  MotionManager::GetInstance()->InitFTSensors();

  // Set every sensor to 0
  for (unsigned int sensor_id = 0; sensor_id < MAXIMUM_NUMBER_OF_FT_SENSORS; sensor_id++)
  {
    for (unsigned int axis_id = 0; axis_id < 3; axis_id++)
    {
      force_raw[sensor_id][axis_id] = 0.0;
      torque_raw[sensor_id][axis_id] = 0.0;
    }
  }
}

void ThorMangHardwareInterface::startCalibration()
{
  if (!goReadyPose())
  {
    ROS_ERROR("Calibration failed! Couldn't move to ready pose.");
    return;
  }
  ROS_INFO("Starting calibration of feet");
  // Start robotis calibration
  has_foot_ft_offsets_in_air = false;
  MotionManager::GetInstance()->RightLegFTSensor.startForceTorqueCalibration();
  MotionManager::GetInstance()->LeftLegFTSensor.startForceTorqueCalibration();
}

void ThorMangHardwareInterface::resetFtSensor(unsigned int sensor_id)
{
  if (sensor_id < 0 || sensor_id >= MAXIMUM_NUMBER_OF_FT_SENSORS)
  {
    ROS_ERROR_STREAM("Id: " << sensor_id << " is not a valid ft sensor id");
    return;
  }
  ROS_INFO_STREAM("Starting reset of id " << sensor_id);
  has_ft_offsets[sensor_id] = false;
  num_ft_measurements[sensor_id] = 0;
  ft_compensation[sensor_id].setBias(FTCompensation::Vector6d::Zero());

  // Set offset to 0
  force_torque_offset[sensor_id] = FTCompensation::Vector6d::Zero();
}

void ThorMangHardwareInterface::update_force_torque_compensation()
{
  for (unsigned int i = 0; i < MAXIMUM_NUMBER_OF_FT_SENSORS; i++)
  {
    Eigen::Matrix3d world_gripper_rot = (robot_transforms_ptr->getRootTransform().rotation() * robot_transforms_ptr->getTransform(ftSensorUIDs[i]).rotation()).inverse();
    ft_compensation[i].setWorldGripperRotation(world_gripper_rot);
  }
}

Thor::FTSensor& ThorMangHardwareInterface::getFTSensorInstance(unsigned int sensor_id)
{
  switch (sensor_id)
  {
    case 0: return MotionManager::GetInstance()->RightArmFTSensor;
    case 1: return MotionManager::GetInstance()->LeftArmFTSensor;
    case 2: return MotionManager::GetInstance()->RightLegFTSensor;
    case 3: return MotionManager::GetInstance()->LeftLegFTSensor;
  }
}

void ThorMangHardwareInterface::update_force_torque_sensors()
{
  compensate_force_torque(R_ARM);
  compensate_force_torque(L_ARM);
  compensate_force_torque(R_LEG);
  compensate_force_torque(L_LEG);

  // initialize walking engines with foot ft offsets
  if (!has_foot_ft_offsets_in_air)
  {
    if (MotionManager::GetInstance()->RightLegFTSensor.hasBias() && MotionManager::GetInstance()->LeftLegFTSensor.hasBias())
    {
      double right_foot_offset[6];
      double left_foot_offset[6];

      // get offsets
      MotionManager::GetInstance()->RightLegFTSensor.getForceTorqueBias(&right_foot_offset[0], &right_foot_offset[1], &right_foot_offset[2],
          &right_foot_offset[3], &right_foot_offset[4], &right_foot_offset[5]);

      MotionManager::GetInstance()->LeftLegFTSensor.getForceTorqueBias(&left_foot_offset[0], &left_foot_offset[1], &left_foot_offset[2],
          &left_foot_offset[3], &left_foot_offset[4], &left_foot_offset[5]);

      ROS_INFO_STREAM("Initial values right foot: " << right_foot_offset[0] << " " << right_foot_offset[1] << " " << right_foot_offset[2] << " "
                                                                            << right_foot_offset[3] << " " << right_foot_offset[4] << " " << right_foot_offset[5]);

      ROS_INFO_STREAM("Initial values left foot: " << left_foot_offset[0] << " " << left_foot_offset[1] << " " << left_foot_offset[2] << " "
                                                                          << left_foot_offset[3] << " " << left_foot_offset[4] << " " << left_foot_offset[5]);

      // set offsets at walking controllers
      RecursiveWalking::GetInstance()->SetInitForceTorque(right_foot_offset[0], right_foot_offset[1], right_foot_offset[2] ,
          right_foot_offset[3], right_foot_offset[4], right_foot_offset[5],
          left_foot_offset[0], left_foot_offset[1], left_foot_offset[2],
          left_foot_offset[3], left_foot_offset[4], left_foot_offset[5]);

      PreviewControlWalking::GetInstance()->SetInitForceTorque(right_foot_offset[0], right_foot_offset[1], right_foot_offset[2] ,
          right_foot_offset[3], right_foot_offset[4], right_foot_offset[5],
          left_foot_offset[0], left_foot_offset[1], left_foot_offset[2],
          left_foot_offset[3], left_foot_offset[4], left_foot_offset[5]);

      has_foot_ft_offsets_in_air = true;
      // Now reset the ft sensors for ros
      for (unsigned int i = 0; i < MAXIMUM_NUMBER_OF_FT_SENSORS; i++)
      {
        resetFtSensor(i);
      }
    }
  }
}

void ThorMangHardwareInterface::compensate_force_torque(unsigned int ft_sensor_index) {
  FTCompensation::Vector6d ft_raw;
  FTCompensation::Vector6d ft_compensated;

  ft_raw[0] = force_raw[ft_sensor_index][0];
  ft_raw[1] = force_raw[ft_sensor_index][1];
  ft_raw[2] = force_raw[ft_sensor_index][2];
  ft_raw[3] = torque_raw[ft_sensor_index][0];
  ft_raw[4] = torque_raw[ft_sensor_index][1];
  ft_raw[5] = torque_raw[ft_sensor_index][2];

  ft_compensation[ft_sensor_index].zeroAndCompensate(ft_raw, ft_compensated);

  force_compensated[ft_sensor_index][0] = ft_compensated[0];
  force_compensated[ft_sensor_index][1] = ft_compensated[1];
  force_compensated[ft_sensor_index][2] = ft_compensated[2];
  torque_compensated[ft_sensor_index][0] = ft_compensated[3];
  torque_compensated[ft_sensor_index][1] = ft_compensated[4];
  torque_compensated[ft_sensor_index][2] = ft_compensated[5];

  // check if ft sensors are being reset
  if (!has_ft_offsets[ft_sensor_index] && getFTSensorInstance(ft_sensor_index).hasBias())
  {
    // accumulate values and divide them later by num of measurements
    force_torque_offset[ft_sensor_index] += ft_compensated;

    if (num_ft_measurements[ft_sensor_index]++ > 500)
    {
      force_torque_offset[ft_sensor_index] = (force_torque_offset[ft_sensor_index] / (double) num_ft_measurements[ft_sensor_index]).eval();
      has_ft_offsets[ft_sensor_index] = true;
      // set offset
      ft_compensation[ft_sensor_index].setBias(force_torque_offset[ft_sensor_index]);
      ROS_INFO_STREAM("FT bias for " << ftSensorUIDs[ft_sensor_index] << ": " <<
                      force_torque_offset[ft_sensor_index](0) << ", " << force_torque_offset[ft_sensor_index](1) << ", " <<
                      force_torque_offset[ft_sensor_index](2) << ", " << force_torque_offset[ft_sensor_index](3) << ", " <<
                      force_torque_offset[ft_sensor_index](4) << ", " << force_torque_offset[ft_sensor_index](5));
          ROS_INFO_THROTTLE(1, "Robot setup finished! You can place the robot on ground now.");
    }
  }
}

void ThorMangHardwareInterface::dynRecParamCallback(thor_mang_ros_control::HardwareInterfaceConfig &config, uint32_t /*level*/)
{
  calibration_joint_offsets[0] = config.r_shoulder_pitch;
  calibration_joint_offsets[2] = config.r_shoulder_roll;
  calibration_joint_offsets[4] = config.r_shoulder_yaw;
  calibration_joint_offsets[6] = config.r_elbow;
  calibration_joint_offsets[8] = config.r_wrist_yaw1;
  calibration_joint_offsets[10] = config.r_wrist_roll;
  calibration_joint_offsets[12] = config.r_wrist_yaw2;

  calibration_joint_offsets[1] = config.l_shoulder_pitch;
  calibration_joint_offsets[3] = config.l_shoulder_roll;
  calibration_joint_offsets[5] = config.l_shoulder_yaw;
  calibration_joint_offsets[7] = config.l_elbow;
  calibration_joint_offsets[9] = config.l_wrist_yaw1;
  calibration_joint_offsets[11] = config.l_wrist_roll;
  calibration_joint_offsets[13] = config.r_wrist_yaw2;

  calibration_joint_offsets[14] = config.r_hip_yaw;
  calibration_joint_offsets[16] = config.r_hip_roll;
  calibration_joint_offsets[18] = config.r_hip_pitch;
  calibration_joint_offsets[20] = config.r_knee;
  calibration_joint_offsets[22] = config.r_ankle_pitch;
  calibration_joint_offsets[24] = config.r_ankle_roll;

  calibration_joint_offsets[15] = config.l_hip_yaw;
  calibration_joint_offsets[17] = config.l_hip_roll;
  calibration_joint_offsets[19] = config.l_hip_pitch;
  calibration_joint_offsets[21] = config.l_knee;
  calibration_joint_offsets[23] = config.l_ankle_pitch;
  calibration_joint_offsets[25] = config.l_ankle_roll;

  calibration_joint_offsets[26] = config.waist_pan;
  calibration_joint_offsets[27] = config.waist_tilt;

  calibration_joint_offsets[28] = config.head_pan;
  calibration_joint_offsets[29] = config.head_tilt;

  calibration_joint_offsets[30] = config.r_hand_thumb;
  calibration_joint_offsets[32] = config.r_hand_index_finger;
  calibration_joint_offsets[34] = config.r_hand_middle_finger;

  calibration_joint_offsets[31] = config.l_hand_thumb;
  calibration_joint_offsets[33] = config.l_hand_index_finger;
  calibration_joint_offsets[35] = config.l_hand_middle_finger;

  calibration_joint_offsets[36] = config.waist_lidar;
}

void ThorMangHardwareInterface::dynRecServoGainsConfigCallback(thor_mang_ros_control::ServoGainsConfig &config, uint32_t level){
    if(level == 0){
        ROS_ERROR("[HardwareInterface] Could not set joint gains because level was 0.");
        return;
    }
    int index = 0;

    while(level > 1){
        level >>= 1;
        index++;
    }

    if(index > 29){
        ROS_ERROR("[HardwareInterface] Could not set joint gains.");
        return;
    }



}

void ThorMangHardwareInterface::reinitializeMotion() {
  ROS_WARN("Reinitializing motion!");
  if(MotionManager::GetInstance()->Reinitialize()) {
    ROS_WARN("Reinitialization successful!");
  } else {
    ROS_WARN("Reinitialization failed!");
  }
}

}

