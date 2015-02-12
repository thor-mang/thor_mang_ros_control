#include <thor_mang_ros_control/thor_mang_footstep_recursive_controller.h>

namespace Thor
{
ThorMangFootstepRecursiveController::ThorMangFootstepRecursiveController()
  : system_control_unit_time_sec(MotionModule::TIME_UNIT)
{
  uID = const_cast<char*>("thor_mang_footstep_recursive_controller");
}

bool ThorMangFootstepRecursiveController::init(hardware_interface::ThorMangFootstepInterface* hw, ros::NodeHandle& nh)
{
  footsteps_handle = hw->getHandle("footsteps_handle");

  std::string execute_step_plan_topic;
  nh.param("execute_step_plan_topic", execute_step_plan_topic, std::string("execute_step_plan"));

  // init action servers
  execute_step_plan_as = vigir_footstep_planning::SimpleActionServer<vigir_footstep_planning::msgs::ExecuteStepPlanAction>::create(nh, execute_step_plan_topic, boost::bind(&ThorMangFootstepRecursiveController::executeStepPlanAction, this, boost::ref(execute_step_plan_as)), true);
  return true;
}

void ThorMangFootstepRecursiveController::update(const ros::Time& time, const ros::Duration& period)
{

}

void ThorMangFootstepRecursiveController::starting(const ros::Time& time)
{
  ROS_WARN_NAMED(footsteps_handle.getName(), "Controller '%s' is starting.", uID);
  Thor::MotionManager::GetInstance()->AddModule(this);
}

void ThorMangFootstepRecursiveController::stopping(const ros::Time& time)
{
  ROS_WARN_NAMED(footsteps_handle.getName(), "Controller '%s' stopping.", uID);
  RecursiveWalking::GetInstance()->Stop();
  Thor::MotionManager::GetInstance()->RemoveModule(this);
}

void ThorMangFootstepRecursiveController::Initialize()
{
  RecursiveWalking::GetInstance()->Initialize();
  last_call = ros::Time::now();
  system_control_unit_time_sec = MotionModule::TIME_UNIT;
}

void ThorMangFootstepRecursiveController::Process()
{
  RecursiveWalking::GetInstance()->Process();
  m_RobotInfo = RecursiveWalking::GetInstance()->m_RobotInfo;

  ros::Time current_time = ros::Time::now();
  system_control_unit_time_sec += ((current_time - last_call).toSec() - system_control_unit_time_sec) * 0.01; // low pass filter
  last_call = current_time;
}

void ThorMangFootstepRecursiveController::InitImuData()
{
  double RollInitAngleRad = 0.0, PitchInitAngleRad = 0.0;
  int InitAngleWindowSize = 50;
  for(int index = 0; index < InitAngleWindowSize; index++)
  {
    RollInitAngleRad += MotionStatus::EulerAngleX;
    PitchInitAngleRad += MotionStatus::EulerAngleY;
    usleep(4000);
  }
  RecursiveWalking::GetInstance()->SetInitAngleinRad(RollInitAngleRad/((double)InitAngleWindowSize), PitchInitAngleRad/((double)InitAngleWindowSize));
}

void ThorMangFootstepRecursiveController::InitFtDataOnGround()
{
  double right_fz_on_gnd_N = 0, left_fz_on_gnd_N = 0;
  int InitAngleWindowSize = 125;
  for(int count = 0; count < InitAngleWindowSize; count++)	{
    right_fz_on_gnd_N  += MotionStatus::R_LEG_FZ;
    left_fz_on_gnd_N  += MotionStatus::L_LEG_FZ;
    usleep(8000);
  }

  right_fz_on_gnd_N = right_fz_on_gnd_N/(double)InitAngleWindowSize;
  left_fz_on_gnd_N = left_fz_on_gnd_N/(double)InitAngleWindowSize;

  RecursiveWalking::GetInstance()->SetInitForceOntheGround(right_fz_on_gnd_N, left_fz_on_gnd_N);
}

void ThorMangFootstepRecursiveController::InitWalking()
{
  // init imu data
  ROS_INFO("ThorMangFootstepRecursiveController: Init IMU");
  double RollInitAngleRad = 0.0, PitchInitAngleRad = 0.0;
  int InitAngleWindowSize = 50;
  for(int index =0; index < InitAngleWindowSize; index++)
  {
    RollInitAngleRad += MotionStatus::EulerAngleX;
    PitchInitAngleRad += MotionStatus::EulerAngleY;
    usleep(4000);
  }
  RecursiveWalking::GetInstance()->SetInitAngleinRad(RollInitAngleRad/((double)InitAngleWindowSize), PitchInitAngleRad/((double)InitAngleWindowSize));

  ROS_INFO("Estimating system control time: %f", system_control_unit_time_sec);

  // init walking lib
  RecursiveWalking::GetInstance()->BALANCE_ENABLE = true;
  RecursiveWalking::GetInstance()->DEBUG_PRINT = false;

  RecursiveWalking::GetInstance()->HIP_PITCH_OFFSET = 7.0;//6.0
  RecursiveWalking::GetInstance()->ANKLE_PITCH_OFFSET = 0.0;
  RecursiveWalking::GetInstance()->Initialize();

  RecursiveWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO = 3.5;
  RecursiveWalking::GetInstance()->IMU_GYRO_GAIN_RATIO = 0.0;//7.31*0.01;
  RecursiveWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO = 0.4;

  RecursiveWalking::GetInstance()->BALANCE_X_GAIN     = +1.0*20.30*0.625*(RecursiveWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO)*RecursiveWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO;
  RecursiveWalking::GetInstance()->BALANCE_Y_GAIN     = -1.0*20.30*0.75*(RecursiveWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO)*RecursiveWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO;
  RecursiveWalking::GetInstance()->BALANCE_Z_GAIN     =    0.0*2.0*(RecursiveWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO)*RecursiveWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO;

  RecursiveWalking::GetInstance()->BALANCE_PITCH_GAIN = -1.0*0.06*0.625*(1-RecursiveWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO)*RecursiveWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO;
  RecursiveWalking::GetInstance()->BALANCE_ROLL_GAIN  = -1.0*0.10*0.75*(1-RecursiveWalking::GetInstance()->FORCE_MOMENT_DISTRIBUTION_RATIO)*RecursiveWalking::GetInstance()->WALK_STABILIZER_GAIN_RATIO;


  RecursiveWalking::GetInstance()->BALANCE_HIP_PITCH_GAIN = 1.0;
  RecursiveWalking::GetInstance()->BALANCE_Z_GAIN_BY_FT = 0.1;

  RecursiveWalking::GetInstance()->BALANCE_RIGHT_ROLL_GAIN_BY_FT = 0.01*0.1*2.0;
  RecursiveWalking::GetInstance()->BALANCE_RIGHT_PITCH_GAIN_BY_FT = -0.01*0.1*2.0;

  RecursiveWalking::GetInstance()->BALANCE_LEFT_ROLL_GAIN_BY_FT = 0.01*0.1*2.0;
  RecursiveWalking::GetInstance()->BALANCE_LEFT_PITCH_GAIN_BY_FT = -0.01*0.1*2.0;

  RecursiveWalking::GetInstance()->BALANCE_HIP_PITCH_SETTLING_TIME = 1.0;
  RecursiveWalking::GetInstance()->BALANCE_Z_SETTLING_TIME = 1.0;

  RecursiveWalking::GetInstance()->BALANCE_RIGHT_ANKLE_ROLL_SETTLING_TIME = 1.0;
  RecursiveWalking::GetInstance()->BALANCE_RIGHT_ANKLE_PITCH_SETTLING_TIME = 1.0;

  RecursiveWalking::GetInstance()->BALANCE_LEFT_ANKLE_ROLL_SETTLING_TIME = 1.0;
  RecursiveWalking::GetInstance()->BALANCE_LEFT_ANKLE_PITCH_SETTLING_TIME = 1.0;

  RecursiveWalking::GetInstance()->FOOT_LANDING_OFFSET_GAIN =   +1.0*0;
  RecursiveWalking::GetInstance()->FOOT_LANDING_DETECT_N = 50;

  RecursiveWalking::GetInstance()->SYSTEM_CONTROL_UNIT_TIME_SEC = system_control_unit_time_sec;
  RecursiveWalking::GetInstance()->FOOT_LANDING_DETECTION_TIME_MAX_SEC = 10.0;

  RecursiveWalking::GetInstance()->FOOT_ROLL_ADJUSTMENT_ABS_MAX_RAD = 10.0*M_PI/180;
  RecursiveWalking::GetInstance()->FOOT_PITCH_ADJUSTMENT_ABS_MAX_RAD = 10.0*M_PI/180;

  RecursiveWalking::GetInstance()->COB_X_ADJUSTMENT_ABS_MAX_MM = 50;
  RecursiveWalking::GetInstance()->COB_Y_ADJUSTMENT_ABS_MAX_MM = 50;

  RecursiveWalking::GetInstance()->P_GAIN = 32;
  RecursiveWalking::GetInstance()->I_GAIN = 0;
  RecursiveWalking::GetInstance()->D_GAIN = 0;

  //RecursiveWalking::GetInstance()->SetRefZMPDecisionParameter(0.0, 0.0, 10.0);
  //RecursiveWalking::GetInstance()->m_ReferenceGtoBodyPosition.z = 650.0;

  boost::mutex::scoped_lock lock(footsteps_handle.getDynamixelMutex());

  // init servo gains
  for(unsigned int index = 0; index < MotionStatus::m_CurrentJoints.size(); index++)
  {
    int id = MotionStatus::m_CurrentJoints[index].m_ID;

    int err;
    MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_ACCELATION_LL, 0, &err);
    MotionStatus::m_CurrentJoints[index].m_DXL_Comm->GetDXLInstance()->WriteDWord(id, PRO54::P_GOAL_VELOCITY_LL, 0, &err);
  }
}

void ThorMangFootstepRecursiveController::StartWalking()
{
  SetMotionEnableList();
  RecursiveWalking::GetInstance()->Start();
}

void ThorMangFootstepRecursiveController::SetMotionEnableList()
{
  for(unsigned int jointIndex = 0; jointIndex < 35; jointIndex++)
  {
    int id = jointIndex;

    //not be fixed code
    if(id >= 15 && id <=26)
    {
      MotionStatus::m_EnableList[id-1].uID = uID;
    }
    if(id == 1 || id ==2 || id == 7|| id == 8)
      MotionStatus::m_EnableList[id-1].uID = uID;
  }

  MotionStatus::m_EnableList[27 - 1].uID = const_cast<char*>("Action");
  MotionStatus::m_EnableList[28 - 1].uID = const_cast<char*>("Action");
  MotionStatus::m_EnableList[29 - 1].uID = const_cast<char*>("Action");
  MotionStatus::m_EnableList[30 - 1].uID = const_cast<char*>("Action");
}

void ThorMangFootstepRecursiveController::executeStepPlanAction(vigir_footstep_planning::SimpleActionServer<vigir_footstep_planning::msgs::ExecuteStepPlanAction>::Ptr& as)
{
  const vigir_footstep_planning::msgs::ExecuteStepPlanGoalConstPtr& goal(as->acceptNewGoal());

  ROS_INFO("Preparing for start walking...");
  InitWalking();

  ROS_INFO("Init IMU data");
  InitImuData();

  ROS_INFO("Init FT data");
  InitFtDataOnGround();

  ROS_INFO("Converting step plan...");
  std::vector<StepData> step_data_list;
  thor_mang_footstep_planner::operator<<(step_data_list, goal->step_plan);

  if (thor_mang_footstep_planner::operator==(last_step_data_list, step_data_list))
  {
    ROS_INFO("Received same plan, reusing last pattern.");
  }
  else
  {
    ROS_INFO("Spooling step plan...");
    for (std::vector<StepData>::iterator itr = step_data_list.begin(); itr != step_data_list.end(); itr++)
    {
      ROS_INFO("%s", thor_mang_footstep_planner::toString(*itr).c_str());
      ROS_INFO("------------------------------------");
      RecursiveWalking::GetInstance()->AddStepData(*itr);
    }

    ROS_INFO("Calculate pattern...");
    if (!RecursiveWalking::GetInstance()->CalcWalkingPattern())
    {
      ROS_ERROR("Can't execute step plan!");

  //    std::vector<StepData> step_data_vec = RecursiveWalking::GetInstance()->GetStepData();

  //    for (std::vector<StepData>::const_iterator itr = step_data_vec.begin(); itr != step_data_vec.end(); itr++)
  //    {
  //      ROS_INFO("%s", thor_mang_footstep_planner::toString(*itr).c_str());
  //      ROS_INFO("------------------------------------");
  //    }

      return;
    }
  }

  last_step_data_list = step_data_list;

  ROS_INFO("Start walking!");
  ThorMangFootstepRecursiveController::StartWalking();

  while(RecursiveWalking::GetInstance()->IsRunning())
  {
    usleep(8000);
  }

  ROS_INFO("Walking finished!");

  vigir_footstep_planning::msgs::ExecuteStepPlanResult result;
  as->setSucceeded(result);
}
}

PLUGINLIB_EXPORT_CLASS(Thor::ThorMangFootstepRecursiveController, controller_interface::ControllerBase)
