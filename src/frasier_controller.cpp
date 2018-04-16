#include <frasier_openrave/frasier_controller.h>


FRASIERController::FRASIERController(ros::NodeHandle n) : nh_(n),
                    arm_cli_("/hsrb/arm_trajectory_controller/follow_joint_trajectory", true),
                    base_cli_("/hsrb/omni_base_controller/follow_joint_trajectory", true){

  arm_cli_.waitForServer();
  base_cli_.waitForServer();

  filter_traj_srv_ = nh_.serviceClient<tmc_manipulation_msgs::FilterJointTrajectory>("/filter_hsrb_trajectory");
//  joint_state_sub_ = nh_.subscribe("/hsrb/joint_states", 1, &FRASIERController::jointSensorCb, this);

//  joint_state_flag_ = false;
}


// FRASIERController::~FRASIERController(){
//
// }

//void FRASIERController::jointSensorCb(const sensor_msgs::JointState::ConstPtr &msg){
//  boost::mutex::scoped_lock lock(joint_state_mutex_);
//  joint_state_flag_ = true;
//  joints_ = *msg;
//
//}

bool FRASIERController::filterTrajectory(Eigen::MatrixXd& traj,
                                         trajectory_msgs::JointTrajectory& whole_body_traj){

//  boost::mutex::scoped_lock lock(joint_state_mutex_);

  tmc_manipulation_msgs::FilterJointTrajectoryRequest filter_req;
  tmc_manipulation_msgs::FilterJointTrajectoryResponse filter_res;
  trajectory_msgs::JointTrajectory arm_traj;


  arm_traj.joint_names.push_back("odom_x");
  arm_traj.joint_names.push_back("odom_y");
  arm_traj.joint_names.push_back("odom_t");
  arm_traj.joint_names.push_back("arm_lift_joint");
  arm_traj.joint_names.push_back("arm_flex_joint");
  arm_traj.joint_names.push_back("arm_roll_joint");
  arm_traj.joint_names.push_back("wrist_flex_joint");
  arm_traj.joint_names.push_back("wrist_roll_joint");


  arm_traj.points.resize(traj.rows());

  for (int i = 0; i < traj.rows(); i++) {
    arm_traj.points[i].positions.resize(8);

    for (int j = 0; j < 8; j++) {
      arm_traj.points[i].positions[j] = traj(i, j);

    }
  }


  filter_req.trajectory = arm_traj;
  filter_req.allowed_time = ros::Duration(30.0);

//  sensor_msgs::JointState start_state_;
//  start_state_.name.push_back("odom_x");
//  start_state_.name.push_back("odom_y");
//  start_state_.name.push_back("odom_t");
//  start_state_.name.push_back("arm_lift_joint");
//  start_state_.name.push_back("arm_flex_joint");
//  start_state_.name.push_back("arm_roll_joint");
//  start_state_.name.push_back("wrist_flex_joint");
//  start_state_.name.push_back("wrist_roll_joint");
//  start_state_.name.push_back("base_roll_joint");
//
//
//  start_state_.position.push_back(0.0);
//  start_state_.position.push_back(0.0);
//  start_state_.position.push_back(0.0);
//  start_state_.position.push_back(0.0);
//  start_state_.position.push_back(0.0);
//  start_state_.position.push_back(0.0);
//  start_state_.position.push_back(0.0);
//  start_state_.position.push_back(0.0);
//  start_state_.position.push_back(0.0);

  filter_req.start_state.joint_state = start_state_;

  std::cout << "calling the filter service..." << std::endl;

  bool result = filter_traj_srv_.call(filter_req, filter_res);

  if (filter_res.error_code.val != tmc_manipulation_msgs::ArmNavigationErrorCodes::SUCCESS) {
      std::cout << "cannot filter the trajectory, error code: " << filter_res.error_code.val << std::endl;
      return false;
  }else{
    std::cout << "trajecory filtering is successfull!" << std::endl;

    whole_body_traj = filter_res.trajectory;

    return true;
  }


}

void FRASIERController::extractArmBaseTraj(trajectory_msgs::JointTrajectory whole_body_traj,
                                           trajectory_msgs::JointTrajectory& base_traj,
                                           trajectory_msgs::JointTrajectory& arm_traj) {

    int n_waypoints = whole_body_traj.points.size();
    base_traj.points.resize(n_waypoints);
    arm_traj.points.resize(n_waypoints);

    base_traj.joint_names.push_back("odom_x");
    base_traj.joint_names.push_back("odom_y");
    base_traj.joint_names.push_back("odom_t");

    arm_traj.joint_names.push_back("arm_lift_joint");
    arm_traj.joint_names.push_back("arm_flex_joint");
    arm_traj.joint_names.push_back("arm_roll_joint");
    arm_traj.joint_names.push_back("wrist_flex_joint");
    arm_traj.joint_names.push_back("wrist_roll_joint");

    for (int i = 0; i < n_waypoints; i++) {
        for (int j = 0; j < 3; j++){
            base_traj.points[i].positions.push_back(whole_body_traj.points[i].positions[j]);
            base_traj.points[i].velocities.push_back(whole_body_traj.points[i].velocities[j]);
            base_traj.points[i].accelerations.push_back(whole_body_traj.points[i].accelerations[j]);
            base_traj.points[i].time_from_start = whole_body_traj.points[i].time_from_start;
        }
        for (int k = 3; k < 8 ; k++) {
            arm_traj.points[i].positions.push_back(whole_body_traj.points[i].positions[k]);
            arm_traj.points[i].velocities.push_back(whole_body_traj.points[i].velocities[k]);
            arm_traj.points[i].accelerations.push_back(whole_body_traj.points[i].accelerations[k]);
            arm_traj.points[i].time_from_start = whole_body_traj.points[i].time_from_start;
        }
    }




}

void FRASIERController::executeWholeBodyTraj(Eigen::MatrixXd& traj) {
    trajectory_msgs::JointTrajectory whole_body_traj;
    if (filterTrajectory(traj, whole_body_traj)) {

        std::cout << "executing the trajectory..." << std::endl;

        executeWholeBodyTraj(whole_body_traj);
    }
}

void FRASIERController::executeWholeBodyTraj(trajectory_msgs::JointTrajectory whole_body_traj) {


    control_msgs::FollowJointTrajectoryGoal base_goal, arm_goal;
    trajectory_msgs::JointTrajectory base_traj, arm_traj;
    extractArmBaseTraj(whole_body_traj, base_traj, arm_traj);


    arm_goal.trajectory = arm_traj;
    base_goal.trajectory = base_traj;

    arm_cli_.sendGoal(arm_goal);
    base_cli_.sendGoal(base_goal);
}


void FRASIERController::sendWholeBodyTraj(Eigen::MatrixXd& traj){

  /////// BASE TRAJECTORY ///////
  control_msgs::FollowJointTrajectoryGoal base_goal;
  trajectory_msgs::JointTrajectory base_traj;

  base_traj.header.stamp = ros::Time::now();

  base_traj.joint_names.push_back("odom_x");
  base_traj.joint_names.push_back("odom_y");
  base_traj.joint_names.push_back("odom_t");

  base_traj.points.resize(traj.rows());

  ros::Duration duration_base(0.2);

  for (int i = 0; i < traj.rows(); i++) {
    base_traj.points[i].positions.resize(3);
    base_traj.points[i].velocities.resize(3);
    for (int j = 0; j < 3; j++) {
      base_traj.points[i].positions[j] = traj(i, j);
      base_traj.points[i].velocities[j] = 0.0;
    }

    base_traj.points[i].time_from_start = duration_base;
    duration_base += ros::Duration(0.2);
  }

  base_goal.trajectory = base_traj;


  /////// ARM TRAJECTORY ///////
  control_msgs::FollowJointTrajectoryGoal arm_goal;
  trajectory_msgs::JointTrajectory arm_traj;

  arm_traj.header.stamp = ros::Time::now();

  arm_traj.joint_names.push_back("arm_lift_joint");
  arm_traj.joint_names.push_back("arm_flex_joint");
  arm_traj.joint_names.push_back("arm_roll_joint");
  arm_traj.joint_names.push_back("wrist_flex_joint");
  arm_traj.joint_names.push_back("wrist_roll_joint");

  arm_traj.points.resize(traj.rows());

  ros::Duration duration_arm(0.2);

  for (int i = 0; i < traj.rows(); i++) {
    arm_traj.points[i].positions.resize(5);
    arm_traj.points[i].velocities.resize(5);
    for (int j = 0; j < 5; j++) {
      arm_traj.points[i].positions[j] = traj(i, j+3);
      arm_traj.points[i].velocities[j] = 0.0;
    }

    arm_traj.points[i].time_from_start = duration_arm;
    duration_arm += ros::Duration(0.2);
  }


  arm_goal.trajectory = arm_traj;

  std::cout << "sending arm trajectory... "  << std::endl;
  std::cout << "sending base trajectory... "  << std::endl;
  arm_cli_.sendGoal(arm_goal);
  base_cli_.sendGoal(base_goal);
  // arm_cli_.waitForResult(ros::Duration(3.0));
  // base_cli_.waitForResult(ros::Duration(3.0));


}

void FRASIERController::moveToStartState(MOVE_STATE state){

  std::cout << "moving to start state... "  << std::endl;

  /////// BASE TRAJECTORY ///////
  control_msgs::FollowJointTrajectoryGoal base_goal;
  trajectory_msgs::JointTrajectory base_traj;

  base_traj.joint_names.push_back("odom_x");
  base_traj.joint_names.push_back("odom_y");
  base_traj.joint_names.push_back("odom_t");

  base_traj.points.resize(1);

  base_traj.points[0].positions.resize(3);
  if (state == MOVE_STATE::HOME){
      base_traj.points[0].positions[0] = 0.0;
      base_traj.points[0].positions[1] = 0.0;
      base_traj.points[0].positions[2] = 0.0;
  }

  base_traj.points[0].time_from_start = ros::Duration(5.0);

  base_goal.trajectory = base_traj;

  /////// ARM TRAJECTORY ///////
  control_msgs::FollowJointTrajectoryGoal arm_goal;
  trajectory_msgs::JointTrajectory arm_traj;

  arm_traj.joint_names.push_back("arm_lift_joint");
  arm_traj.joint_names.push_back("arm_flex_joint");
  arm_traj.joint_names.push_back("arm_roll_joint");
  arm_traj.joint_names.push_back("wrist_flex_joint");
  arm_traj.joint_names.push_back("wrist_roll_joint");

  arm_traj.points.resize(1);

  arm_traj.points[0].positions.resize(5);
  if (state == MOVE_STATE::HOME) {

      arm_traj.points[0].positions[0] = 0.0;
      arm_traj.points[0].positions[1] = 0.0;
      arm_traj.points[0].positions[2] = 0.0;
      arm_traj.points[0].positions[3] = 0.0;
      arm_traj.points[0].positions[4] = 0.0;
  }
  else if(state == MOVE_STATE::PICK){
      arm_traj.points[0].positions[0] = 0.60;
      arm_traj.points[0].positions[1] = -2.60;
      arm_traj.points[0].positions[2] = 0.0;
      arm_traj.points[0].positions[3] = -0.54;
      arm_traj.points[0].positions[4] = 0.0;
  }

  arm_traj.points[0].time_from_start = ros::Duration(5.0);

  arm_goal.trajectory = arm_traj;

  if (state == MOVE_STATE::HOME){
      base_cli_.sendGoal(base_goal);
      arm_cli_.sendGoal(arm_goal);
      base_cli_.waitForResult(ros::Duration(6.0));
      arm_cli_.waitForResult(ros::Duration(6.0));
  }
  else if(state == MOVE_STATE::PICK){
      arm_cli_.sendGoal(arm_goal);
      arm_cli_.waitForResult(ros::Duration(6.0));
  }

}

void FRASIERController::setStartState(sensor_msgs::JointState &start_state) {
    start_state_ = start_state;
}
