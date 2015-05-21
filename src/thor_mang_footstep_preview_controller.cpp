#include <thor_mang_ros_control/thor_mang_footstep_preview_controller.h>

const unsigned int joint_count = 37;

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
  "l_f1_j0",        // l_hand_index_finger
  "r_hand_middle_finger",
  "l_hand_middle_finger",
  "waist_lidar"
};

namespace Thor
{
ThorMangFootstepPreviewController::ThorMangFootstepPreviewController()
  : system_control_unit_time_sec(MotionModule::TIME_UNIT),
    claim_arms(true),
    imu_resetted(true),
    ft_resetted(true)
{
  uID = const_cast<char*>("thor_mang_footstep_preview_controller");
}

bool ThorMangFootstepPreviewController::init(hardware_interface::PositionJointInterface *hw, ros::NodeHandle& nh)
{
  // Load params
  std::string execute_step_plan_topic;
  nh.param("execute_step_plan_topic", execute_step_plan_topic, std::string("execute_step_plan"));
  nh.param("arms", claim_arms, true);

  dyn_rec_server_.reset(new FootstepPreviewConfigServer(nh));
  dyn_rec_server_->setCallback(boost::bind(&ThorMangFootstepPreviewController::dynRecParamCallback, this, _1, _2));

  // Pre-claim all joints needed by robotis
  for (unsigned int id = 1; id < joint_count+1; id++)
  {
    if((id >= 15 && id <=26) || claim_arms && (id == 1 || id ==2 || id == 7|| id == 8))
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
  }

  // Init walking
  nh.param("hip_pitch_offset", hip_pitch_offset, 10.2);
  nh.param("ankle_pitch_offset", ankle_pitch_offset, -1.08);
  nh.param("walk_stabilizer_gain_ratio", walk_stabilizer_gain_ratio, 3.0);
  nh.param("imu_gyro_gain_ratio", hip_pitch_offset, 0.0731);
  nh.param("force_moment_distribution_ratio", force_moment_distribution_ratio, 0.4);
  nh.param("balance_hip_pitch_gain", balance_hip_pitch_gain, 1.0);
  nh.param("balance_z_gain_by_ft", balance_z_gain_by_ft, 0.05);
  nh.param("balance_right_roll_gain_by_ft", balance_right_roll_gain_by_ft, 0.001);
  nh.param("balance_right_pitch_gain_by_ft", balance_right_pitch_gain_by_ft, -0.0005);
  nh.param("balance_left_roll_gain_by_ft", balance_left_roll_gain_by_ft, 0.001);
  nh.param("balance_left_pitch_gain_by_ft", balance_left_pitch_gain_by_ft, -0.0005);
  nh.param("foot_landing_offset_gain", foot_landing_offset_gain, 1.0);
  nh.param("foot_landing_detect_n", foot_landing_detect_n, 50.0);

  initWalkingParameters();
  PreviewControlWalking::GetInstance()->SetInitialPose(0.0, -125.0,   0.0, 0.0, 0.0, 0.0,
                                                       0.0,  125.0,   0.0, 0.0, 0.0, 0.0,
                                                       0.0,    0.0, 650.0, 0.0, 0.0, 0.0);

  // Init action servers
  execute_step_plan_as = ActionServer::create(nh, execute_step_plan_topic, true, boost::bind(&ThorMangFootstepPreviewController::executeStepPlanAction, this, boost::ref(execute_step_plan_as)), boost::bind(&ThorMangFootstepPreviewController::stepPlanPreempted, this));
  return true;
}

void ThorMangFootstepPreviewController::update(const ros::Time& time, const ros::Duration& period)
{
  addFtData();
  addImuData();
  if (goal_waiting & ft_resetted & imu_resetted) {
    goal_waiting = false;
    executeStepPlanAction(execute_step_plan_as);
  }
  // Nothing to do here since robotis has its own process function
  if (PreviewControlWalking::GetInstance()->IsRunning()) {
    if (PreviewControlWalking::GetInstance()->GetNumofRemainingUnreservedStepData() != remaining_steps) {
      remaining_steps = PreviewControlWalking::GetInstance()->GetNumofRemainingUnreservedStepData();
      ROS_INFO_STREAM("[PreviewWalking] Remaining steps: " << remaining_steps << "/" << total_steps);
      ROS_INFO_STREAM("[PreviewWalking] Currently executing: " << total_steps - remaining_steps << ", Last executed: " << total_steps - remaining_steps -1);
      ROS_INFO_STREAM("[PreviewWalking] -----------------------------------------------------");
      vigir_footstep_planning::msgs::ExecuteStepPlanFeedback feedback;
      feedback.currently_executing_step_index = total_steps - remaining_steps;
      feedback.first_changeable_step_index = total_steps - remaining_steps +1;
      feedback.last_performed_step_index = total_steps - remaining_steps -1;
      execute_step_plan_as->publishFeedback(feedback);
    }
  } else {
    if (execute_step_plan_as->isActive()) {
      ROS_INFO("[PreviewWalking] Walking finished!");

      vigir_footstep_planning::msgs::ExecuteStepPlanResult result;
      execute_step_plan_as->setSucceeded(result);
    }
  }
}

void ThorMangFootstepPreviewController::starting(const ros::Time& time)
{
  claimJoints();

  ROS_INFO("[PreviewWalking] Controller '%s' is starting.", uID);
  MotionManager::GetInstance()->AddModule(this);
  ROS_INFO("[PreviewWalking] Init IMU data");
  InitImuData();
  ROS_INFO("[PreviewWalking] Init FT data");
  InitFtDataOnGround();
  goal_waiting = false;

  PreviewControlWalking::GetInstance()->Start();
  ROS_INFO("[PreviewWalking] Footstep controller init successful.");
}

void ThorMangFootstepPreviewController::stopping(const ros::Time& time)
{
  ROS_INFO("[PreviewWalking] Controller '%s' stopping.", uID);
  PreviewControlWalking::GetInstance()->Stop();
  Thor::MotionManager::GetInstance()->RemoveModule(this);
  unclaimJoints();
  imu_resetted = false;
  ft_resetted = false;
}

void ThorMangFootstepPreviewController::Initialize()
{
  PreviewControlWalking::GetInstance()->Initialize();
  last_call = ros::Time::now();
  system_control_unit_time_sec = MotionModule::TIME_UNIT;
}

void ThorMangFootstepPreviewController::Process()
{
  PreviewControlWalking::GetInstance()->Process();
  m_RobotInfo = PreviewControlWalking::GetInstance()->m_RobotInfo;

  ros::Time current_time = ros::Time::now();
  system_control_unit_time_sec += ((current_time - last_call).toSec() - system_control_unit_time_sec) * 0.01; // low pass filter
  last_call = current_time;
}

void ThorMangFootstepPreviewController::InitImuData()
{
  imu_resetted = false;
  imu_bias[0] = 0;
  imu_bias[1] = 0;
}

void ThorMangFootstepPreviewController::InitFtDataOnGround()
{
  for (unsigned int i = 0; i < 12; i++) {
    ft_bias_on_ground[i] = 0;
  }
  ft_resetted = false;
}

void ThorMangFootstepPreviewController::addImuData() {
  if (!imu_resetted) {
    if (current_imu_measurements++ < max_imu_measurements) {
      imu_bias[0] += MotionStatus::EulerAngleX; // Roll
      imu_bias[1] += MotionStatus::EulerAngleY; // Pitch
    } else {
      for (unsigned int i = 0; i < 2; i++) {
        imu_bias[i] = imu_bias[i]/(double)max_imu_measurements;
      }
      ROS_INFO_STREAM("[PreviewWalking] Imu initialized with: " << imu_bias[0] << ", " << imu_bias[1]);
      PreviewControlWalking::GetInstance()->SetInitAngleinRad(imu_bias[0], imu_bias[1]);
      imu_resetted = true;
    }
  }
}

void ThorMangFootstepPreviewController::addFtData() {
  if (!ft_resetted) {
    if (current_ft_measurements++ < max_ft_measurements) {
      ft_bias_on_ground[0]  += MotionStatus::R_LEG_FX;
      ft_bias_on_ground[1]  += MotionStatus::R_LEG_FY;
      ft_bias_on_ground[2]  += MotionStatus::R_LEG_FZ;
      ft_bias_on_ground[3] += MotionStatus::R_LEG_TX;
      ft_bias_on_ground[4] += MotionStatus::R_LEG_TY;
      ft_bias_on_ground[5] += MotionStatus::R_LEG_TZ;

      ft_bias_on_ground[6]  += MotionStatus::L_LEG_FX;
      ft_bias_on_ground[7]  += MotionStatus::L_LEG_FY;
      ft_bias_on_ground[8]  += MotionStatus::L_LEG_FZ;
      ft_bias_on_ground[9] += MotionStatus::L_LEG_TX;
      ft_bias_on_ground[10] += MotionStatus::L_LEG_TY;
      ft_bias_on_ground[11] += MotionStatus::L_LEG_TZ;
    } else {
      ft_resetted = true;
      std::stringstream ss;
      for (unsigned int i = 0; i < 12; i++) {
        ft_bias_on_ground[i] = ft_bias_on_ground[i]/(double)max_ft_measurements;
        ss << ft_bias_on_ground[i];
        if (i == 5) ss << "\n";
        else if (i != 11) ss << ", ";
      }
      ROS_INFO_STREAM("[PreviewWalking] Ft initialized with: " << ss.str());
      PreviewControlWalking::GetInstance()->SetInitForceOntheGround(ft_bias_on_ground[0], ft_bias_on_ground[1], ft_bias_on_ground[2],
                                                                    ft_bias_on_ground[3], ft_bias_on_ground[4], ft_bias_on_ground[5],
                                                                    ft_bias_on_ground[6], ft_bias_on_ground[7], ft_bias_on_ground[8],
                                                                    ft_bias_on_ground[9], ft_bias_on_ground[10], ft_bias_on_ground[11]);
    }
  }
}

void ThorMangFootstepPreviewController::initWalkingParameters()
{
  ROS_INFO("[PreviewWalking] Estimating system control time: %f", system_control_unit_time_sec);

  ROS_INFO_STREAM("[PreviewWalking] Setting hip pitch offset to: " << hip_pitch_offset);

  // init walking lib
  PreviewControlWalking::GetInstance()->BALANCE_ENABLE = true;
  PreviewControlWalking::GetInstance()->DEBUG_PRINT = false;

  PreviewControlWalking::GetInstance()->HIP_PITCH_OFFSET = hip_pitch_offset;//7.0;//6.0
  PreviewControlWalking::GetInstance()->ANKLE_PITCH_OFFSET = ankle_pitch_offset;//0.0;
  //PreviewControlWalking::GetInstance()->Initialize(); /// TODO: Shouldn't be necessary

  PreviewControlWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO = walk_stabilizer_gain_ratio;//3.0;//3.5;
  PreviewControlWalking::GetInstance()->IMU_GYRO_GAIN_RATIO = imu_gyro_gain_ratio; // 7.31*0.01;
  PreviewControlWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO = force_moment_distribution_ratio;//0.4;

  PreviewControlWalking::GetInstance()->BALANCE_X_GAIN     = +1.0*20.30*0.625*(PreviewControlWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO)*PreviewControlWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO;
  PreviewControlWalking::GetInstance()->BALANCE_Y_GAIN     = -1.0*20.30*0.75*(PreviewControlWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO)*PreviewControlWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO;
  PreviewControlWalking::GetInstance()->BALANCE_Z_GAIN     =    0.0*2.0*(PreviewControlWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO)*PreviewControlWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO;

  PreviewControlWalking::GetInstance()->BALANCE_PITCH_GAIN = -1.0*0.06*0.625*(1.0 - PreviewControlWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO)*PreviewControlWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO;
  PreviewControlWalking::GetInstance()->BALANCE_ROLL_GAIN  = -1.0*0.10*0.75*(1.0 - PreviewControlWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO)*PreviewControlWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO;

  PreviewControlWalking::GetInstance()->BALANCE_HIP_PITCH_GAIN = balance_hip_pitch_gain;//1.0;
  PreviewControlWalking::GetInstance()->BALANCE_Z_GAIN_BY_FT = balance_z_gain_by_ft ;//0.1*0.5;

  PreviewControlWalking::GetInstance()->BALANCE_RIGHT_ROLL_GAIN_BY_FT = balance_right_roll_gain_by_ft;//0.01*0.1;
  PreviewControlWalking::GetInstance()->BALANCE_RIGHT_PITCH_GAIN_BY_FT = balance_right_pitch_gain_by_ft;//-0.01*0.1*0.5;

  PreviewControlWalking::GetInstance()->BALANCE_LEFT_ROLL_GAIN_BY_FT = balance_left_roll_gain_by_ft;//0.01*0.1;
  PreviewControlWalking::GetInstance()->BALANCE_LEFT_PITCH_GAIN_BY_FT = balance_left_pitch_gain_by_ft;//-0.01*0.1*0.5;

  PreviewControlWalking::GetInstance()->AXIS_CONTROLLER_GAIN = 0.0;

  PreviewControlWalking::GetInstance()->BALANCE_HIP_PITCH_TIME_CONSTANT = 1.0*0.01;
  PreviewControlWalking::GetInstance()->BALANCE_Z_TIME_CONSTANT = 1.0*0.5;

  PreviewControlWalking::GetInstance()->BALANCE_RIGHT_ANKLE_ROLL_TIME_CONSTANT = 1.0*0.5;
  PreviewControlWalking::GetInstance()->BALANCE_RIGHT_ANKLE_PITCH_TIME_CONSTANT = 1.0*0.5;

  PreviewControlWalking::GetInstance()->BALANCE_LEFT_ANKLE_ROLL_TIME_CONSTANT = 1.0*0.5;
  PreviewControlWalking::GetInstance()->BALANCE_LEFT_ANKLE_PITCH_TIME_CONSTANT = 1.0*0.5;

  PreviewControlWalking::GetInstance()->AXIS_CONTROLLER_TIME_CONSTANT = 0.01;

  PreviewControlWalking::GetInstance()->FOOT_LANDING_OFFSET_GAIN =   foot_landing_offset_gain;//+1.0*0;
  PreviewControlWalking::GetInstance()->FOOT_LANDING_DETECT_N = foot_landing_detect_n;//50;

  PreviewControlWalking::GetInstance()->SYSTEM_CONTROL_UNIT_TIME_SEC = MotionModule::TIME_UNIT / 1000.0;
  PreviewControlWalking::GetInstance()->FOOT_LANDING_DETECTION_TIME_MAX_SEC = 10.0;

  PreviewControlWalking::GetInstance()->FOOT_ROLL_ADJUSTMENT_ABS_MAX_RAD = 10.0*M_PI/180;
  PreviewControlWalking::GetInstance()->FOOT_PITCH_ADJUSTMENT_ABS_MAX_RAD = 10.0*M_PI/180;

  PreviewControlWalking::GetInstance()->COB_X_ADJUSTMENT_ABS_MAX_MM = 50;
  PreviewControlWalking::GetInstance()->COB_Y_ADJUSTMENT_ABS_MAX_MM = 50;

  PreviewControlWalking::GetInstance()->P_GAIN = 32;
  PreviewControlWalking::GetInstance()->I_GAIN = 0;
  PreviewControlWalking::GetInstance()->D_GAIN = 0;

  //PreviewControlWalking::GetInstance()->SetRefZMPDecisionParameter(0.0, 0.0, 0.0);

  // init servo gains
  for(unsigned int index = 0; index < MotionStatus::m_CurrentJoints.size(); index++)
  {
    int id = MotionStatus::m_CurrentJoints[index].m_ID;

    int err;
    MotionManager::GetInstance()->WriteDWord(id, PRO54::P_GOAL_ACCELATION_LL, 0, &err);
    MotionManager::GetInstance()->WriteDWord(id, PRO54::P_GOAL_VELOCITY_LL, 0, &err);
  }
}

void ThorMangFootstepPreviewController::StartWalking()
{
  if(!PreviewControlWalking::GetInstance()->IsRunning())
    PreviewControlWalking::GetInstance()->Start();
}

void ThorMangFootstepPreviewController::claimJoints()
{
  for(unsigned int jointIndex = 0; jointIndex < 35; jointIndex++)
  {
    int id = jointIndex;
    if(id >= 15 && id <=26)
    {
      MotionStatus::m_EnableList[id-1].uID = uID;
    }
    if(claim_arms && (id == 1 || id ==2 || id == 7|| id == 8))
      MotionStatus::m_EnableList[id-1].uID = uID;
  }
}

void ThorMangFootstepPreviewController::unclaimJoints()
{
  for (unsigned int id = 1; id < joint_count+1; id++)
  {
    if((id >= 15 && id <=26) || id == 1 || id ==2 || id == 7|| id == 8)
      MotionStatus::m_EnableList[id-1].uID = const_cast<char*>("thor_mang_hardware_interface");
  }
}

void ThorMangFootstepPreviewController::executeStepPlanAction(ActionServer::Ptr& as)
{
  if (!(imu_resetted && ft_resetted)) {
    goal_waiting = true;
    ROS_INFO("[PreviewWalking] Received new step plan but waiting for sensor reset.");
    return;
  }
  const vigir_footstep_planning::msgs::ExecuteStepPlanGoalConstPtr& goal(as->acceptNewGoal());
  vigir_footstep_planning::msgs::StepPlan step_plan = goal->step_plan;

  initWalkingParameters();

  Thor::StepData ref_step_data;
  PreviewControlWalking::GetInstance()->GetReferenceStepDatafotAddition(&ref_step_data);

  ROS_INFO("[PreviewWalking] Converting step plan...");
  std::vector<StepData> step_data_list;
  step_data_list.push_back(ref_step_data);
  if (!thor_mang_footstep_planning::operator<<(step_data_list, step_plan)) {
    ROS_ERROR("[PreviewWalking] Error during step plan conversion!");
    return;
  }

  ROS_INFO("[PreviewWalking] Spooling step plan...");
  for (std::vector<StepData>::iterator itr = step_data_list.begin(); itr != step_data_list.end(); itr++)
  {
    StepData step_data = *itr;

    ROS_INFO("%s", thor_mang_footstep_planning::toString(step_data).c_str());
    ROS_INFO("------------------------------------");

    PreviewControlWalking::GetInstance()->AddStepData(step_data);
  }

  ROS_INFO("[PreviewWalking] Start walking!");
  StartWalking();
  remaining_steps = PreviewControlWalking::GetInstance()->GetNumofRemainingUnreservedStepData();
  total_steps = step_plan.steps.size();
}

void ThorMangFootstepPreviewController::stepPlanPreempted() {
  ROS_INFO("[PreviewWalking] Step plan preempting was requested.");
}

void ThorMangFootstepPreviewController::dynRecParamCallback(thor_mang_ros_control::FootstepPreviewControllerConfig &config, uint32_t level)
{
  hip_pitch_offset = config.hip_pitch_offset;
  ankle_pitch_offset = config.ankle_pitch_offset;
  walk_stabilizer_gain_ratio = config.walk_stabilizer_gain_ratio;
  imu_gyro_gain_ratio = config.imu_gyro_gain_ratio;
  force_moment_distribution_ratio = config.force_moment_distribution_ratio;
  balance_hip_pitch_gain = config.balance_hip_pitch_gain;
  balance_z_gain_by_ft = config.balance_z_gain_by_ft;
  balance_right_roll_gain_by_ft = config.balance_right_roll_gain_by_ft;
  balance_right_pitch_gain_by_ft = config.balance_right_pitch_gain_by_ft;
  balance_left_roll_gain_by_ft = config.balance_left_roll_gain_by_ft;
  balance_left_pitch_gain_by_ft = config.balance_left_pitch_gain_by_ft;
  foot_landing_offset_gain = config.foot_landing_offset_gain;
  foot_landing_detect_n = config.foot_landing_detect_n;

  initWalkingParameters();
}
}

PLUGINLIB_EXPORT_CLASS(Thor::ThorMangFootstepPreviewController, controller_interface::ControllerBase)
