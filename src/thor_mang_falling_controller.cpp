#include <thor_mang_ros_control/thor_mang_falling_controller.h>

#include "tf/tf.h"
#include "motion/motionmanager.h"

namespace Thor
{

ThorMangFallingController::ThorMangFallingController() : falling_state(Disabled)
{

}


bool ThorMangFallingController::init(hardware_interface::ImuSensorInterface *hw, ros::NodeHandle& nh)
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

    imu_sensor_handle = hw->getHandle("pelvis_imu");

    falling_state = Disabled;

    return true;
}

void ThorMangFallingController::update(const ros::Time& time, const ros::Duration& period)
{
    switch(falling_state){
        case Disabled:
            return;
        case Ready:
            if(checkFalling()){
                goIntoFallPose();
                falling_state = FallPose;
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

}

void ThorMangFallingController::starting(const ros::Time& time)
{
    MotionManager::GetInstance()->AddModule(this);

    falling_state = Ready;

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
    MotionManager::GetInstance()->EnableLights(false);
    for (unsigned int joint_index = 0; joint_index < m_RobotInfo.size(); joint_index++)
    {
      MotionManager::GetInstance()->SetTorqueOn(m_RobotInfo[joint_index], false);
    }
}


void ThorMangFallingController::sendInfoToControlModeSwitcher(){
    if (action_client->isServerConnected()){
        vigir_humanoid_control_msgs::ChangeControlModeGoal goal;

        goal.mode_request="falling";

        action_client->sendGoal(goal, boost::bind(&ThorMangFallingController::modeSwitchDoneCallback, this, _1, _2), boost::bind(&ThorMangFallingController::modeSwitchActiveCallback, this), boost::bind(&ThorMangFallingController::modeSwitchFeedbackCallback, this, _1));
        action_client->waitForResult(ros::Duration(5));

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
}

PLUGINLIB_EXPORT_CLASS(Thor::ThorMangFallingController, controller_interface::ControllerBase)
