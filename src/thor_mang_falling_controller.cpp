#include <thor_mang_ros_control/thor_mang_falling_controller.h>

#include "tf/tf.h"
#include "motion/motionmanager.h"

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


ThorMangFallingController::ThorMangFallingController() : falling_state(Disabled)
{
	uID = const_cast<char*>("thor_mang_falling_controller");
}


bool ThorMangFallingController::init(hardware_interface::ImuSensorInterface *hw, ros::NodeHandle& nh)
{
    nh.param("rollThresholdPositive", rollThresholdPositive, 0.245);
    nh.param("rollThresholdNegative", rollThresholdNegative, -0.325);
    nh.param("pitchThresholdPositive", pitchThresholdPositive, 0.3);
    nh.param("pitchThresholdNegative", pitchThresholdNegative, -0.15);
    nh.param("fallPoseTime", fallPoseTime, 0.500);

    nh.param<std::string>("control_mode_switch_name", control_mode_switch_name, "/mode_controllers/control_mode_controller/change_control_mode");

    action_client.reset(new ChangeControlModeActionClient(control_mode_switch_name, true));

    imu_sensor_handle = hw->getHandle("pelvis_imu");

    falling_state = Disabled;

    if (MotionStatus::m_CurrentJoints.size() != 0)
    {
      m_RobotInfo = MotionStatus::m_CurrentJoints;
    }
    else
    {
      ROS_WARN("MotionStatus is not initialized");
    }

    return true;
}

void ThorMangFallingController::update(const ros::Time& time, const ros::Duration& period)
{
    ROS_INFO_THROTTLE(100,"[thor_mang_falling_controller] in state %d", falling_state);
    switch(falling_state){
        case Disabled:
            return;
        case Ready:
            if(checkFalling()){
                goIntoFallPose();
                falling_state = FallPose;
		sendInfoToControlModeSwitcher();            
}
            break;
        case FallPose:
            if(checkTorqueOff()){
                disableTorque();
                falling_state = TorqueOff;
            }
            break;
        case TorqueOff:
            return;
        default:
            ROS_ERROR("Unknown state");
    }
    ROS_INFO_THROTTLE(100,"[thor_mang_falling_controller] END update");

}

void ThorMangFallingController::starting(const ros::Time& time)
{
    MotionManager::GetInstance()->AddModule(this);

    falling_state = Ready;
    ROS_INFO("[thor_mang_falling_controller] Starting...");

}

void ThorMangFallingController::stopping(const ros::Time& time)
{
    Thor::MotionManager::GetInstance()->RemoveModule(this);
    falling_state = Disabled;
}

void ThorMangFallingController::Initialize()
{
}

void ThorMangFallingController::Process()
{
}

bool ThorMangFallingController::checkFalling()
{
    const double* imu_orientation = imu_sensor_handle.getOrientation();

    double roll = 0, pitch = 0, yaw = 0;

    tf::Quaternion orientation(imu_orientation[0] , imu_orientation[1], imu_orientation[2], imu_orientation[3] );

    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    if (roll > rollThresholdPositive || roll < rollThresholdNegative
            || pitch < pitchThresholdNegative || pitch > pitchThresholdPositive){
        if (pitch > pitchThresholdPositive)
            fallingPose = PoseFront;
        else
            fallingPose = PoseRear;
        return true;
    }
    return false;
}

void ThorMangFallingController::goIntoFallPose(){
    ROS_INFO("Going into FALL!");
    if(fallingPose == PoseFront){
        ROS_INFO("Falling pose FRONT");
    }else{
        ROS_INFO("Falling pose REAR");
    }

    fallPoseDoneTime = (ros::WallTime::now() + ros::WallDuration(fallPoseTime));
    MotionManager::GetInstance()->EnableLights(true);

    //TODO impl
    //claimJoints

    //set Joint Values
}

bool ThorMangFallingController::checkTorqueOff(){
    ros::WallTime current = ros::WallTime::now();
    if( current > fallPoseDoneTime ){
        return true;
    }
}

void ThorMangFallingController::disableTorque(){
    ROS_INFO("disable torque!");
    claimJoints();
    MotionManager::GetInstance()->EnableLights(false);

    for (unsigned int joint_index = 0; joint_index < m_RobotInfo.size(); joint_index++)
    {
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


void ThorMangFallingController::modeSwitchDoneCallback(const actionlib::SimpleClientGoalState& state,  const vigir_humanoid_control_msgs::ChangeControlModeResultConstPtr& result){
    if(state == actionlib::SimpleClientGoalState::SUCCEEDED){
        //Good
    }else{
        //Need to try again
    }
}

void ThorMangFallingController::modeSwitchActiveCallback(){

}

void ThorMangFallingController::modeSwitchFeedbackCallback(const vigir_humanoid_control_msgs::ChangeControlModeFeedbackConstPtr &feedback){

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

}

PLUGINLIB_EXPORT_CLASS(Thor::ThorMangFallingController, controller_interface::ControllerBase)
