#include <thor_mang_ros_control/thor_mang_falling_controller.h>

#include "tf/tf.h"
#include "motion/motionmanager.h"
#include <thor_mang_ros_control/thor_mang_hardware_interface.h>

namespace Thor
{

const int ros_joint_offsets[MotionStatus::MAXIMUM_NUMBER_OF_JOINTS-1] =
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
  -504,     //r_f0_j0	// r_hand_thumb
  -751,     //l_f0_j0	// l_hand_thumb
  -434,     //r_f1_j0	// r_hand_index_finger
  -621,     //l_f1_j0	// l_hand_index_finger
  0,        // r_hand_middle_finger
  0,        // l_hand_middle_finger
  0         // waist_lidar
};


ThorMangFallingController::ThorMangFallingController() : fallState(Disabled)
{
  uID = const_cast<char*>("thor_mang_falling_controller");
  torqueTestCounter = 0;
  lightOn = false;
  stateTransitionCounter = 0;
  rollOffset = 0;
  pitchOffset = 0;
}


bool ThorMangFallingController::init(hardware_interface::ImuSensorInterface *hw, ros::NodeHandle& nh)
{
  nh_ = nh;
  nh.param("fallDetectionAngleThreshold", fallDetectionAngleThreshold, 32.0);
  nh.param("fallRelaxAngleThreshold", fallRelaxAngleThreshold, 66.0);
  nh.param("rollOffset", rollOffset, 0.02);
  nh.param("pitchOffset", pitchOffset, 0.15);
  nh.param("vel_goal", vel_goal, 2000);

  nh.param<std::string>("control_mode_switch_name", control_mode_switch_name, "/mode_controllers/control_mode_controller/change_control_mode");

  imu_rpy_pub = nh.advertise<geometry_msgs::Vector3Stamped>("falling_controller/imu_rpy", 1);

  action_client.reset(new ChangeControlModeActionClient(control_mode_switch_name, true));

  imu_sensor_handle = hw->getHandle("pelvis_imu");

  fallState = Disabled;

  if (MotionStatus::m_CurrentJoints.size() != 0)
  {
    m_RobotInfo = MotionStatus::m_CurrentJoints;
  }
  else
  {
    ROS_WARN("MotionStatus is not initialized");
  }

  initJoints();

  MotionManager::GetInstance()->EnableLights(false);

  return true;
}

// Called in every tick, after starting has been called.
void ThorMangFallingController::update(const ros::Time& time, const ros::Duration& period)
{
  double roll = 0, pitch = 0, yaw = 0;
  const double* imu_orientation = imu_sensor_handle.getOrientation();
  tf::Quaternion orientation(imu_orientation[0] , imu_orientation[1], imu_orientation[2], imu_orientation[3] );
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  roll -= rollOffset;
  pitch -= pitchOffset;

  if (imu_rpy_pub.getNumSubscribers() > 0)
  {
    geometry_msgs::Vector3Stamped rpy;
    rpy.header.frame_id = "/world";
    rpy.header.stamp = ros::Time::now();
    rpy.vector.x = roll;
    rpy.vector.y = pitch;
    rpy.vector.z = yaw;

    imu_rpy_pub.publish(rpy);
  }

  switch(fallState)
  {
    case Ready:
      if (detectAndDecide())
      {
        MotionManager::GetInstance()->EnableLights(true);
        fallState = Falling;
      }
      break;
    case Falling:
      fallPose();
      sendInfoToControlModeSwitcher();
      if (checkTorqueOff())
      {
        fallState = TorqueOff;
      }
      break;
    case TorqueOff:
      disableTorque();
      return;
    default:
      ROS_ERROR("Unknown state");
      break;
  }
}

void ThorMangFallingController::starting(const ros::Time& time)
{
  if (MotionStatus::m_CurrentJoints.size() != 0)
  {
    m_RobotInfo = MotionStatus::m_CurrentJoints;
  }
  else
  {
    ROS_WARN("MotionStatus is not initialized");
  }

  MotionManager::GetInstance()->AddModule(this);

  fallState = Ready;
  ROS_INFO("[thor_mang_falling_controller] Starting...");
}

void ThorMangFallingController::stopping(const ros::Time& time)
{
  unclaimJoints();
  unlimitSpeed();
  MotionManager::GetInstance()->EnableLights(false);
  Thor::MotionManager::GetInstance()->RemoveModule(this);
  fallState = Disabled;
  ROS_INFO("ThorMangFallingController::stopping");
}


// Don't use!
void ThorMangFallingController::Initialize()
{
}

// Don't use!
void ThorMangFallingController::Process()
{
}

bool ThorMangFallingController::detectAndDecide()
{
  double roll = 0, pitch = 0, yaw = 0;
  const double* imu_orientation = imu_sensor_handle.getOrientation();
  tf::Quaternion orientation(imu_orientation[0] , imu_orientation[1], imu_orientation[2], imu_orientation[3] );
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  roll -= rollOffset;
  pitch -= pitchOffset;

  double fallDirection = atan2(-roll, pitch);

  ROS_DEBUG("Checking attitude roll: %f pitch: %f direction: %f", roll, pitch, fallDirection);

  if ((std::max(fabs(roll), fabs(pitch))*180.0/M_PI) > fallDetectionAngleThreshold)
    stateTransitionCounter++;
  else
    stateTransitionCounter = 0;

  // Fall detection.
  if (stateTransitionCounter > 5)
  {
    stateTransitionCounter = 0;

    fallingPose = PoseFront;
    if (fabs(fallDirection) < M_PI/4.0)
      fallingPose = PoseFront; // front
    else if (fabs(fallDirection) > 3.0*M_PI/4.0)
      fallingPose = PoseBack; // back
    else if (fallDirection > 0)
      fallingPose = PoseLeft;
    else if (fallDirection < 0)
      fallingPose = PoseRight;

    return true;
  }

  return false;
}

void ThorMangFallingController::fallPose()
{
  limitSpeed();
  claimJoints();

  if (fallingPose == PoseFront)
  {
    ROS_INFO_THROTTLE(5.0, "Falling pose FRONT");
    fallPoseFront();
  }
  else if (fallingPose == PoseBack)
  {
    ROS_INFO_THROTTLE(5.0, "Falling pose BACK");
    fallPoseBack();
  }
  else if (fallingPose == PoseLeft)
  {
    ROS_INFO_THROTTLE(5.0, "Falling pose LEFT");
    fallPoseLeft();
  }
  else if (fallingPose == PoseRight)
  {
    ROS_INFO_THROTTLE(5.0, "Falling pose RIGHT");
    fallPoseRight();
  }
}

void ThorMangFallingController::fallPoseFront()
{
  //ARMS

  setJoint(1, 1.43); //r_shoulder_pitch
  setJoint(2, -1.43); //l_shoulder_pitch

  setJoint(3, -0.085); //r_shoulder_roll
  setJoint(4, 0.085); //l_shoulder_roll

  setJoint(5, -0.036); //r_shoulder_yaw
  setJoint(6, 0.036); //l_shoulder_yaw

  setJoint(7, 2.16); //r_elbow
  setJoint(8, -2.16); //l_elbow

  setJoint(9, -3.22); //r_wrist_yaw_1
  setJoint(10, 3.22); //l_wrist_yaw_1

  setJoint(11, -0.9311); //r_wrist_roll
  setJoint(12, 0.9311); //l_wrist_roll

  setJoint(13, -0.02); //r_wrist_yaw_2
  setJoint(14, 0.02); //l_wrist_yaw_2

  // Torso
  setJoint(27, 0); // waist pan
  setJoint(28, 0); // waist tilt

  //LEGS

  setJoint(15, 0); //r_hip_yaw
  setJoint(16, 0); //l_hip_yaw

  setJoint(17, 0.04); //r_hip_roll
  setJoint(18, -0.04); //l_hip_roll

  setJoint(19, 0.83); //r_hip_pitch
  setJoint(20, -0.83); //l_hip_pitch

  setJoint(21, -1.41); //r_knee
  setJoint(22, 1.41); //l_knee

  setJoint(23, -0.64);  //r_ankle_pitch
  setJoint(24, 0.64); //l_ankle_pitch

  setJoint(25, 0.03);  //r_ankle_roll
  setJoint(26, -0.03); //l_ankle_roll
}

void ThorMangFallingController::fallPoseBack()
{
  //ARMS
  setJoint(1, -1); //r_shoulder_pitch
  setJoint(2, 1); //l_shoulder_pitch

  setJoint(3, -0.0101); //r_shoulder_roll
  setJoint(4, 0.0101); //l_shoulder_roll

  setJoint(5, 0.03); //r_shoulder_yaw
  setJoint(6, -0.03); //l_shoulder_yaw

  setJoint(7, 2.5851); //r_elbow
  setJoint(8, -2.5851); //l_elbow

  setJoint(9, -1.55); //r_wrist_yaw_1
  setJoint(10, 1.55); //l_wrist_yaw_1

  setJoint(11, -0.21); //r_wrist_roll
  setJoint(12, 0.21); //l_wrist_roll

  setJoint(13, 0.0); //r_wrist_yaw_2
  setJoint(14, 0.0); //l_wrist_yaw_2

  // Torso
  setJoint(27, 0); // waist pan
  setJoint(28, 0.697); // waist tilt

  //LEGS
  setJoint(15, -0.0); //r_hip_yaw
  setJoint(16, 0.0); //l_hip_yaw

  setJoint(17, 0.0948); //r_hip_roll
  setJoint(18, -0.0948); //l_hip_roll

  setJoint(19,  0.174); //r_hip_pitch
  setJoint(20,  -0.174); //l_hip_pitch

  setJoint(21, -0.0544); //r_knee
  setJoint(22, 0.0544); //l_knee

  setJoint(23, 0.096);  //r_ankle_pitch
  setJoint(24, -0.096); //l_ankle_pitch

  setJoint(25, 0.0783);  //r_ankle_roll
  setJoint(26, -0.0783); //l_ankle_roll

}

void ThorMangFallingController::fallPoseLeft()
{
  //ARMS
  setJoint(1, -0.79); //r_shoulder_pitch
  setJoint(2, -0.2123); //l_shoulder_pitch

  setJoint(3, 0.2691); //r_shoulder_roll
  setJoint(4, 0.0842); //l_shoulder_roll

  setJoint(5, 0.0); //r_shoulder_yaw
  setJoint(6, 0.0); //l_shoulder_yaw

  setJoint(7, 1.5174); //r_elbow
  setJoint(8, -1.3463); //l_elbow

  setJoint(9, -1.55); //r_wrist_yaw_1
  setJoint(10, 1.478); //l_wrist_yaw_1

  setJoint(11, 0.0163); //r_wrist_roll
  setJoint(12, -0.4534); //l_wrist_roll

  setJoint(13, 0.0); //r_wrist_yaw_2
  setJoint(14, -0.0283); //l_wrist_yaw_2

  // Torso
  setJoint(27, 0); // waist pan
  setJoint(28, 0); // waist tilt

  //LEGS stay as they are
}

void ThorMangFallingController::fallPoseRight()
{
    //ARMS
    setJoint(1, 0.2123); //r_shoulder_pitch
    setJoint(2, 0.79); //l_shoulder_pitch

    setJoint(3, -0.0842); //r_shoulder_roll
    setJoint(4, -0.2691); //l_shoulder_roll

    setJoint(5, 0.0); //r_shoulder_yaw
    setJoint(6, 0.0); //l_shoulder_yaw

    setJoint(7, 1.3463); //r_elbow
    setJoint(8, -1.5174); //l_elbow

    setJoint(9, -1.478); //r_wrist_yaw_1
    setJoint(10, 1.55); //l_wrist_yaw_1

    setJoint(11, 0.4534); //r_wrist_roll
    setJoint(12, -0.0163); //l_wrist_roll

    setJoint(13, 0.0283); //r_wrist_yaw_2
    setJoint(14, 0.0); //l_wrist_yaw_2

    // Torso
    setJoint(27, 0); // waist pan
    setJoint(28, 0); // waist tilt

    //LEGS stay as they are
}

bool ThorMangFallingController::checkTorqueOff()
{
    double roll = 0, pitch = 0, yaw = 0;
    const double* imu_orientation = imu_sensor_handle.getOrientation();
    tf::Quaternion orientation(imu_orientation[0] , imu_orientation[1], imu_orientation[2], imu_orientation[3] );
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    roll -= rollOffset;
    pitch -= pitchOffset;

    if (std::max(fabs(roll), fabs(pitch))*180.0/M_PI > fallRelaxAngleThreshold)
        stateTransitionCounter++;
    else
        stateTransitionCounter = 0;

    if (stateTransitionCounter > 5)
    {
        stateTransitionCounter = 0;
        ROS_INFO("Torque off!");
        return true;
    }

    return false;
}

void ThorMangFallingController::disableTorque()
{
  ROS_INFO_THROTTLE(5.0,"disable torque!");

  torqueTestCounter++;
  if (torqueTestCounter % 20 == 0)
  {
    lightOn = !lightOn;
    MotionManager::GetInstance()->EnableLights(lightOn);
  }

  MotionManager::GetInstance()->EnableLights(false);

  for (unsigned int joint_index = 0; joint_index < m_RobotInfo.size(); joint_index++)
  {
    if (m_RobotInfo[joint_index].m_ID != 27 && m_RobotInfo[joint_index].m_ID != 28) // Except torso pitch and torso yaw!
      MotionManager::GetInstance()->SetTorqueOn(m_RobotInfo[joint_index], false);
  }
}


void ThorMangFallingController::sendInfoToControlModeSwitcher(){
  ROS_INFO("sending falling");
  if (action_client->isServerConnected()){
    vigir_humanoid_control_msgs::ChangeControlModeGoal goal;

    goal.mode_request="falling";

    action_client->sendGoal(goal, boost::bind(&ThorMangFallingController::modeSwitchDoneCallback, this, _1, _2), boost::bind(&ThorMangFallingController::modeSwitchActiveCallback, this), boost::bind(&ThorMangFallingController::modeSwitchFeedbackCallback, this, _1));
    ROS_INFO("falling sent");
  }else{
    ROS_ERROR("Unable to calll control mode switcher.");
  }
}

void ThorMangFallingController::claimJoints()
{
  for (unsigned int joint_index = 0; joint_index < m_RobotInfo.size(); joint_index++)
  {
    if (m_RobotInfo[joint_index].m_ID > MotionStatus::MAXIMUM_NUMBER_OF_JOINTS-1)
    {
      ROS_WARN("Robot has joint with invalid id: %u", m_RobotInfo[joint_index].m_ID);
      continue;
    }

    unsigned int id_index = m_RobotInfo[joint_index].m_ID-1;

    // activate control
    MotionStatus::m_EnableList[id_index].uID = uID;
  }
}

void ThorMangFallingController::unclaimJoints() {
  for (unsigned int joint_index = 0; joint_index < m_RobotInfo.size(); joint_index++)
  {
    if (m_RobotInfo[joint_index].m_ID > MotionStatus::MAXIMUM_NUMBER_OF_JOINTS-1)
    {
      ROS_WARN("Robot has joint with invalid id: %u", m_RobotInfo[joint_index].m_ID);
      continue;
    }

    unsigned int id_index = m_RobotInfo[joint_index].m_ID-1;

    // activate control
    MotionStatus::m_EnableList[id_index].uID = const_cast<char*>("thor_mang_hardware_interface");
  }
}

void ThorMangFallingController::setJointsToPose(){
  for (unsigned int joint_index = 0; joint_index < m_RobotInfo.size(); joint_index++)
  {
    int id_index = m_RobotInfo[joint_index].m_ID-1;

    if (m_RobotInfo[joint_index].m_ID < 1 || m_RobotInfo[joint_index].m_ID > MotionStatus::MAXIMUM_NUMBER_OF_JOINTS-1)
      continue;

    double value = 0;

    m_RobotInfo[joint_index].m_Value = m_RobotInfo[joint_index].m_DXLInfo->Rad2Value(value) + ros_joint_offsets[id_index];
  }
}

void ThorMangFallingController::initJoints(){
  for (unsigned int joint_index = 0; joint_index < m_RobotInfo.size(); joint_index++)
  {
    servo_id_mapping[ m_RobotInfo[joint_index].m_ID ] = joint_index;
  }
}

void ThorMangFallingController::setJoint(unsigned int servo_id, double value){
  unsigned int joint_index = servo_id_mapping[servo_id];

  if (m_RobotInfo[joint_index].m_ID < 1 || m_RobotInfo[joint_index].m_ID > MotionStatus::MAXIMUM_NUMBER_OF_JOINTS-1){
    ROS_ERROR("Trying to operate on invalid joint: %d", joint_index);
    return;
  }

  if(m_RobotInfo[joint_index].m_ID == servo_id){
    int id_index = m_RobotInfo[joint_index].m_ID-1;
    m_RobotInfo[joint_index].m_Value = m_RobotInfo[joint_index].m_DXLInfo->Rad2Value(value) + ros_joint_offsets[id_index];
    // ROS_INFO("Setting joint value for %d (%d): %f", servo_id, joint_index, value);
    return;
  }
}

void ThorMangFallingController::limitSpeed(){
  ThorMangHardwareInterface::Instance()->limitJointSpeed(vel_goal);
}

void ThorMangFallingController::unlimitSpeed() {
  ThorMangHardwareInterface::Instance()->unlimitJointSpeed();
}

void ThorMangFallingController::modeSwitchDoneCallback(const actionlib::SimpleClientGoalState& state,  const vigir_humanoid_control_msgs::ChangeControlModeResultConstPtr& result){
}

void ThorMangFallingController::modeSwitchActiveCallback(){
}

void ThorMangFallingController::modeSwitchFeedbackCallback(const vigir_humanoid_control_msgs::ChangeControlModeFeedbackConstPtr &feedback){
}
}

PLUGINLIB_EXPORT_CLASS(Thor::ThorMangFallingController, controller_interface::ControllerBase)
