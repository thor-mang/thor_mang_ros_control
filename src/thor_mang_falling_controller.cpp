#include <thor_mang_ros_control/thor_mang_falling_controller.h>

#include "tf/tf.h"
#include "motion/motionmanager.h"

namespace Thor
{

const bool debugMode = true; //TODO remove

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
}


bool ThorMangFallingController::init(hardware_interface::ImuSensorInterface *hw, ros::NodeHandle& nh)
{

    nh.param("fallDetectionAngleThreshold", fallDetectionAngleThreshold, 40.0);
    nh.param("fallRelaxAngleThreshold", fallRelaxAngleThreshold, 66.0);


    nh.param<std::string>("control_mode_switch_name", control_mode_switch_name, "/mode_controllers/control_mode_controller/change_control_mode");

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


    return true;
}

// Called in every tick, after starting has been called.
void ThorMangFallingController::update(const ros::Time& time, const ros::Duration& period)
{
    switch(fallState)
    {
        case Ready:
            if (detectAndDecide())
            {
                MotionManager::GetInstance()->EnableLights(true);
                fallState = Falling;
            }
            else
            {
                MotionManager::GetInstance()->EnableLights(false);
            }
            break;
        case Falling:
            fallPose();
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
    MotionManager::GetInstance()->AddModule(this);

    fallState = Ready;
    ROS_INFO("[thor_mang_falling_controller] Starting...");

    testing_fall_timer = ros::WallTime::now() + ros::WallDuration(5.0); //TODO remove me. Important
    ROS_INFO("Testing fall!");
}

void ThorMangFallingController::stopping(const ros::Time& time)
{
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

    double fallDirection = atan2(-roll, pitch);

    ROS_DEBUG("Checking attitude roll: %f pitch: %f direction: %f", roll, pitch, fallDirection);

    // Fall detection.
    if ((max(fabs(roll), fabs(pitch))*180.0/M_PI) > fallDetectionAngleThreshold)
    {
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
//    limitSpeed();
//    claimJoints();

    if(fallingPose == PoseFront)
    {
        ROS_INFO("Falling pose FRONT");
//        fallPoseFront();
    }
    else if (fallingPose == PoseBack)
    {
        ROS_INFO("Falling pose BACK");
//        fallPoseBack();
    }
    else if (fallingPose == PoseLeft)
    {
        ROS_INFO("Falling pose LEFT");
//        fallPoseLeft();
    }
    else if (fallingPose == PoseRight)
    {
        ROS_INFO("Falling pose RIGHT");
//        fallPoseRight();
    }
}

void ThorMangFallingController::fallPoseFront()
{
    //ARMS

    setJoint(1, 1.78); //r_shoulder_pitch
    setJoint(2, -1.78); //l_shoulder_pitch

    setJoint(3, 0.33); //r_shoulder_roll
    setJoint(4, -0.33); //l_shoulder_roll

    setJoint(5, 0.53); //r_shoulder_yaw
    setJoint(6, -0.53); //l_shoulder_yaw

    setJoint(7, 2.03); //r_elbow
    setJoint(8, -2.03); //l_elbow

    setJoint(9, 0.71); //r_wrist_yaw_1
    setJoint(10, -0.71); //l_wrist_yaw_1

    setJoint(11, 0.97); //r_wrist_roll
    setJoint(12, -0.97); //l_wrist_roll

    setJoint(13, -0.90); //r_wrist_yaw_2
    setJoint(14, 0.90); //l_wrist_yaw_2

    //LEGS

    setJoint(15, -0.01); //r_hip_yaw
    setJoint(16, 0.01); //l_hip_yaw

    setJoint(17, 0.09); //r_hip_roll
    setJoint(18, -0.09); //l_hip_roll

    setJoint(19, 1.57); //r_hip_pitch
    setJoint(20, -1.57); //l_hip_pitch

    setJoint(21, -2.49); //r_knee
    setJoint(22, 2.49); //l_knee

    setJoint(23, -0.91);  //r_ankle_pitch
    setJoint(24, 0.91); //l_ankle_pitch

    setJoint(25, -0.09);  //r_ankle_roll
    setJoint(26, 0.09); //l_ankle_roll
}

void ThorMangFallingController::fallPoseBack()
{
    //ARMS

    setJoint(1, -0.79); //r_shoulder_pitch
    setJoint(2, 0.79); //l_shoulder_pitch

    setJoint(3, 0.44); //r_shoulder_roll
    setJoint(4, -0.44); //l_shoulder_roll

    setJoint(5, 0.57); //r_shoulder_yaw
    setJoint(6, -0.57); //l_shoulder_yaw

    setJoint(7, 2.49); //r_elbow
    setJoint(8, -2.49); //l_elbow

    setJoint(9, -1.55); //r_wrist_yaw_1
    setJoint(10, 1.55); //l_wrist_yaw_1

    setJoint(11, 0.0); //r_wrist_roll
    setJoint(12, 0.0); //l_wrist_roll

    setJoint(13, 0.0); //r_wrist_yaw_2
    setJoint(14, 0.0); //l_wrist_yaw_2

    //LEGS

    setJoint(15, -0.01); //r_hip_yaw
    setJoint(16, 0.01); //l_hip_yaw

    setJoint(17, 0.09); //r_hip_roll
    setJoint(18, -0.09); //l_hip_roll

    setJoint(19, 1.57); //r_hip_pitch
    setJoint(20, -1.57); //l_hip_pitch

    setJoint(21, -2.49); //r_knee
    setJoint(22, 2.49); //l_knee

    setJoint(23, -0.91);  //r_ankle_pitch
    setJoint(24, 0.91); //l_ankle_pitch

    setJoint(25, -0.09);  //r_ankle_roll
    setJoint(26, 0.09); //l_ankle_roll

}

void ThorMangFallingController::fallPoseLeft()
{
    //ARMS

    setJoint(1, -0.79); //r_shoulder_pitch
    setJoint(2, 0.79); //l_shoulder_pitch

    setJoint(3, 0.44); //r_shoulder_roll
    setJoint(4, -0.44); //l_shoulder_roll

    setJoint(5, 0.57); //r_shoulder_yaw
    setJoint(6, -0.57); //l_shoulder_yaw

    setJoint(7, 2.49); //r_elbow
    setJoint(8, -2.49); //l_elbow

    setJoint(9, -1.55); //r_wrist_yaw_1
    setJoint(10, 1.55); //l_wrist_yaw_1

    setJoint(11, 0.0); //r_wrist_roll
    setJoint(12, 0.0); //l_wrist_roll

    setJoint(13, 0.0); //r_wrist_yaw_2
    setJoint(14, 0.0); //l_wrist_yaw_2

    //LEGS

    setJoint(15, -0.01); //r_hip_yaw
    setJoint(16, 0.01); //l_hip_yaw

    setJoint(17, 0.09); //r_hip_roll
    setJoint(18, -0.09); //l_hip_roll

    setJoint(19, 1.57); //r_hip_pitch
    setJoint(20, -1.57); //l_hip_pitch

    setJoint(21, -2.49); //r_knee
    setJoint(22, 2.49); //l_knee

    setJoint(23, -0.91);  //r_ankle_pitch
    setJoint(24, 0.91); //l_ankle_pitch

    setJoint(25, -0.09);  //r_ankle_roll
    setJoint(26, 0.09); //l_ankle_roll

}

void ThorMangFallingController::fallPoseRight()
{
    //ARMS

    setJoint(1, -0.79); //r_shoulder_pitch
    setJoint(2, 0.79); //l_shoulder_pitch

    setJoint(3, 0.44); //r_shoulder_roll
    setJoint(4, -0.44); //l_shoulder_roll

    setJoint(5, 0.57); //r_shoulder_yaw
    setJoint(6, -0.57); //l_shoulder_yaw

    setJoint(7, 2.49); //r_elbow
    setJoint(8, -2.49); //l_elbow

    setJoint(9, -1.55); //r_wrist_yaw_1
    setJoint(10, 1.55); //l_wrist_yaw_1

    setJoint(11, 0.0); //r_wrist_roll
    setJoint(12, 0.0); //l_wrist_roll

    setJoint(13, 0.0); //r_wrist_yaw_2
    setJoint(14, 0.0); //l_wrist_yaw_2

    //LEGS

    setJoint(15, -0.01); //r_hip_yaw
    setJoint(16, 0.01); //l_hip_yaw

    setJoint(17, 0.09); //r_hip_roll
    setJoint(18, -0.09); //l_hip_roll

    setJoint(19, 1.57); //r_hip_pitch
    setJoint(20, -1.57); //l_hip_pitch

    setJoint(21, -2.49); //r_knee
    setJoint(22, 2.49); //l_knee

    setJoint(23, -0.91);  //r_ankle_pitch
    setJoint(24, 0.91); //l_ankle_pitch

    setJoint(25, -0.09);  //r_ankle_roll
    setJoint(26, 0.09); //l_ankle_roll

}

bool ThorMangFallingController::checkTorqueOff()
{
    double roll = 0, pitch = 0, yaw = 0;
    const double* imu_orientation = imu_sensor_handle.getOrientation();
    tf::Quaternion orientation(imu_orientation[0] , imu_orientation[1], imu_orientation[2], imu_orientation[3] );
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    if (max(fabs(roll), fabs(pitch))*180.0/M_PI > fallRelaxAngleThreshold)
    {
        ROS_INFO("Torque off!");
        return true;
    }
    return false;
}

void ThorMangFallingController::disableTorque()
{
    ROS_INFO_THROTTLE(5.0,"disable torque!");

    torqueTestCounter++;
    if (torqueTestCounter % 40 == 0)
    {
        lightOn = !lightOn;
        MotionManager::GetInstance()->EnableLights(lightOn);
    }

    return;

    claimJoints();
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
            ROS_INFO("Setting koint value for %d (%d): %f", servo_id, joint_index, value);
            return;
        }
}

void ThorMangFallingController::limitSpeed(){
    for (unsigned int joint_index = 0; joint_index < m_RobotInfo.size(); joint_index++)
    {
        if (m_RobotInfo[joint_index].m_DXLInfo->MODEL_NUM != 42 && m_RobotInfo[joint_index].m_DXLInfo->MODEL_NUM != 54)
            continue;

        int id = m_RobotInfo[joint_index].m_ID;

        int error = 0;
        m_RobotInfo[joint_index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_ACCELATION_LL, 4, &error); //0 -> unlimited
        m_RobotInfo[joint_index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_VELOCITY_LL, 2000, &error); //0 -> unlimited
        ROS_INFO("Setting speed limit for %d", joint_index);
        ROS_ERROR_COND(error, "Error %d occured on ID %d", error, id);
    }
}

void ThorMangFallingController::modeSwitchDoneCallback(const actionlib::SimpleClientGoalState& state,  const vigir_humanoid_control_msgs::ChangeControlModeResultConstPtr& result){
}

void ThorMangFallingController::modeSwitchActiveCallback(){
}

void ThorMangFallingController::modeSwitchFeedbackCallback(const vigir_humanoid_control_msgs::ChangeControlModeFeedbackConstPtr &feedback){
}
}

PLUGINLIB_EXPORT_CLASS(Thor::ThorMangFallingController, controller_interface::ControllerBase)
