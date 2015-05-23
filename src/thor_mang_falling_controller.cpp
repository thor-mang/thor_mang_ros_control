#include <thor_mang_ros_control/thor_mang_falling_controller.h>

#include "tf/tf.h"
#include "motion/motionmanager.h"

namespace Thor
{

const unsigned int joint_count = 34;

const std::string jointUIDs[] =
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
  "l_f1_j0"        // l_hand_index_finger

};



ThorMangFallingController::ThorMangFallingController() : falling_state(Disabled)
{
	uID = const_cast<char*>("thor_mang_falling_controller");
}


bool ThorMangFallingController::init(hardware_interface::PositionJointInterface *hw, ros::NodeHandle& nh)
{
    nh.param("rollThresholdPositive", rollThresholdPositive, 0.245);
    nh.param("rollThresholdNegative", rollThresholdNegative, -0.325);
    nh.param("pitchThresholdPositive", pitchThresholdPositive, 0.3);
    nh.param("pitchThresholdNegative", pitchThresholdNegative, -0.15);
    nh.param("fallPoseTime", fallPoseTime, 0.500);

    nh.getParam("jointIds", jointIds);
    nh.getParam("jointValues", jointValues);

    nh.param<std::string>("control_mode_switch_name", control_mode_switch_name, "/mode_controllers/control_mode_controller/change_control_mode");

    action_client.reset(new ChangeControlModeActionClient(control_mode_switch_name, true));

    //imu_sensor_handle = hw->getHandle("pelvis_imu");

    falling_state = Disabled;

for (unsigned int id = 1; id < joint_count+1; id++)
  {
      try
      {
        hw->getHandle(jointUIDs[id-1]);
      }
      catch (hardware_interface::HardwareInterfaceException e)
      {
        ROS_ERROR_STREAM("[PreviewWalking] Can't find joint '" << jointUIDs[id-1] << "' in hardware interface. Error: " << e.what());
        return false;
      }
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
	return false;
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
      ROS_INFO("Torque off: %d", joint_index);
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
  for(unsigned int jointIndex = 0; jointIndex < 35; jointIndex++)
  {
    int id = jointIndex;
    if(id >= 15 && id <=26)
    {
      MotionStatus::m_EnableList[id-1].uID = uID;
    }
    if((id == 1 || id ==2 || id == 7|| id == 8))
      MotionStatus::m_EnableList[id-1].uID = uID;
  }
}
}

PLUGINLIB_EXPORT_CLASS(Thor::ThorMangFallingController, controller_interface::ControllerBase)
