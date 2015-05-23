#ifndef THOR_MANG_FALLING_CONTROLLER_H
#define THOR_MANG_FALLING_CONTROLLER_H

#include "ros/ros.h"

// ros control
#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <motion/motionmodule.h>

// THOR-OP includes
#include <framework/Thor.h>

//Mode switcher action client
#include <actionlib/client/simple_action_client.h>
#include <vigir_humanoid_control_msgs/ChangeControlModeAction.h>

#include <map>

namespace Thor
{

typedef actionlib::SimpleActionClient<vigir_humanoid_control_msgs::ChangeControlModeAction> ChangeControlModeActionClient;

typedef enum {
    Disabled,
    Ready,
    FallPose,
    TorqueOff
}State;

typedef enum {
    PoseFront,
    PoseRear
}FallingPose;

class ThorMangFallingController:
        public controller_interface::Controller<hardware_interface::ImuSensorInterface>
        , public MotionModule
{
public:
    ThorMangFallingController();

    // ros control
    bool init(hardware_interface::ImuSensorInterface *hw, ros::NodeHandle& nh);

    void update(const ros::Time& time, const ros::Duration& period);

    void starting(const ros::Time& time);
    void stopping(const ros::Time& time);

    // THOR-OP framework
    void Initialize();
    void Process();

protected:
    bool checkFalling();
    void sendInfoToControlModeSwitcher();
    void goIntoFallPose();
    bool checkTorqueOff();
    void disableTorque();
    void setJoint(unsigned int servo_id, double value);
    void fallPose();
    void fallPoseFront();
    void fallPoseRear();
    void limitSpeed(); //Only for test!
    ros::WallTime testing_fall_timer;

private:
    void initJoints();
    void claimJoints();
    void setJointsToPose();
    double rollThresholdPositive;
    double rollThresholdNegative;
    double pitchThresholdPositive;
    double pitchThresholdNegative;

    double fallPoseTime;

    std::map<unsigned int, unsigned int> servo_id_mapping;

    State falling_state;

    FallingPose fallingPose;

    std::string control_mode_switch_name;
    boost::shared_ptr<ChangeControlModeActionClient> action_client;

    hardware_interface::ImuSensorHandle imu_sensor_handle;

    ros::WallTime fallPoseDoneTime;

    ros::WallTime torqueOffDoneTime;

    void modeSwitchDoneCallback(const actionlib::SimpleClientGoalState& state,  const vigir_humanoid_control_msgs::ChangeControlModeResultConstPtr& result);
    void modeSwitchActiveCallback();
    void modeSwitchFeedbackCallback(const vigir_humanoid_control_msgs::ChangeControlModeFeedbackConstPtr &feedback);

};

}
#endif // THOR_MANG_FALLING_CONTROLLER_H
