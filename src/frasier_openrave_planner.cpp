#include <frasier_openrave/frasier_openrave.h>

void FRASIEROpenRAVE::initRRTPlanner(){
  manip_ = hsr_->GetManipulator("whole_body");
  planner_ = OpenRAVE::RaveCreatePlanner(env_, "birrt");
  hsr_->SetActiveDOFs(manip_->GetArmIndices());
}


void FRASIEROpenRAVE::planToConf(std::vector<double>& q){

  OpenRAVE::PlannerBase::PlannerParametersPtr params(new OpenRAVE::PlannerBase::PlannerParameters());
  params->_nMaxIterations = 1000; // max iterations before failure
  params->SetRobotActiveJoints(hsr_); // set planning configuration space to current active dofs
  params->vgoalconfig.resize(hsr_->GetActiveDOF());
  std::cout << "active dofs: " << hsr_->GetActiveDOF() << std::endl;

  {
    OpenRAVE::RobotBase::RobotStateSaver saver(hsr_); // save the state

    for (int i = 0; i < hsr_->GetActiveDOF(); i++) {
      params->vgoalconfig[i] = q[i];
    }

    hsr_->SetActiveDOFValues(params->vgoalconfig);

    if (hsr_->CheckSelfCollision()) {
      std::cout << "self collision detected!" << std::endl;
      return;
    }

    if(env_->CheckCollision(hsr_)) {
      std::cout << "env collision detected!" << std::endl;
      return;
    }

  }

  hsr_->GetActiveDOFValues(params->vinitialconfig);

  if( !planner_->InitPlan(hsr_, params) ) {
      std::cout << "couldn't init the plan" << std::endl;
      return;
  }

  OpenRAVE::TrajectoryBasePtr traj = OpenRAVE::RaveCreateTrajectory(env_);
  OpenRAVE::PlannerStatus status = planner_->PlanPath(traj);

  std::cout << "plan status : " << status << std::endl;
  if(status & OpenRAVE::PlannerStatus::PS_HasSolution) {
      RAVELOG_WARN("Found plan\n");
  }
  std::cout << "traj time " << traj->GetDuration()<< std::endl;

  std::vector<double> traj_0;
  traj->Sample(traj_0, 0.1);
  for (size_t i = 0; i < traj_0.size(); i++) {
    std::cout << traj_0[i] << '\n';
  }


}

void FRASIEROpenRAVE::computeIK(Eigen::Affine3d& eef_pose, Eigen::VectorXd&  q_sol, bool check_coll){

  Json::Value opt_j = createJsonValueIK(eef_pose, check_coll);

  updatePlanningEnv();

  trajopt::TrajOptProbPtr traj_prob = trajopt::ConstructProblem(opt_j, planning_env_);
  trajopt::TrajOptResultPtr traj_result = trajopt::OptimizeProblem(traj_prob, false);

  q_sol = traj_result->traj.row(0);

}

void FRASIEROpenRAVE::computeTrajectory(Eigen::MatrixXd& traj, EEFPoseGoals eef_goals){ //TODO: Should I set active dofs???


  Json::Value opt_j = createJsonValueTraj(10, eef_goals);

  updatePlanningEnv();

  trajopt::TrajOptProbPtr traj_prob = trajopt::ConstructProblem(opt_j, planning_env_);
  trajopt::TrajOptResultPtr result = trajopt::OptimizeProblem(traj_prob, false);

  traj = result->traj;

}

void FRASIEROpenRAVE::playTrajectory(Eigen::MatrixXd& traj){
  std::vector<int> q_index;
  getWholeBodyJointIndex(q_index);

  for (int i = 0; i < traj.rows(); i++) {
    hsr_->SetDOFValues(trajopt::trajToDblVec(traj.row(i)), 1, q_index);
    ros::Duration(0.5).sleep();
  }
}

Json::Value FRASIEROpenRAVE::createJsonValueTraj(int n_steps, EEFPoseGoals eef_goals){
  json opt_j, disc_coll_j, joint_vel_j, basic_info_j, init_info_j;

  std::vector<json> eef_pose_j;
  eef_pose_j.resize(eef_goals.n_goals);

  int first_step = 0; int last_step = n_steps-1;

  std::vector<double> joint_vel_coeffs(8, 1.0);
  std::vector<int> disc_coll_coeffs(n_steps, 50);
  std::vector<double> disc_coll_pen(n_steps, 0.040);

  basic_info_j = { {"n_steps", n_steps}, {"manip", "whole_body"}, {"start_fixed", true} };

  joint_vel_j = { {"type", "joint_vel"}, {"params", { {"coeffs", joint_vel_coeffs} } } };

  disc_coll_j = { {"type", "collision"}, {"params", { {"coeffs", disc_coll_coeffs}, {"continuous", false}, {"dist_pen", disc_coll_pen}, {"first_step", first_step}, {"last_step", last_step} } } };

  for (int i = 0; i < eef_goals.n_goals; i++) {
    Eigen::Quaterniond rot(eef_goals.poses[i].linear());
    Eigen::Vector3d pos = eef_goals.poses[i].translation();
    eef_pose_j[i] = { {"type", "pose"}, {"params", { {"xyz", {pos(0), pos(1), pos(2)}}, {"wxyz", {rot.w(), rot.x(), rot.y(), rot.z()}}, {"link", "hand_palm_link"}, {"timestep", eef_goals.timesteps[i]}} } };
  }

  init_info_j = { {"type", "stationary"} };

  opt_j["basic_info"] = basic_info_j;
  opt_j["costs"] = {joint_vel_j, disc_coll_j};
  opt_j["constraints"] = eef_pose_j;
  opt_j["init_info"] = init_info_j;


  Json::Value v;
  Json::Reader r;

  std::string opt_j_str = opt_j.dump();

  if (!r.parse(opt_j_str, v, false)) {
    std::cout << "can't read json!"  << std::endl;
    std::cout << r.getFormatedErrorMessages() << std::endl;
    return 0;
  }

  return v;
}

Json::Value FRASIEROpenRAVE::createJsonValueIK(Eigen::Affine3d& eef_pose, bool check_coll){
  json opt_j, disc_coll_j, basic_info_j, init_info_j, eef_pose_j;

  Eigen::Quaterniond rot(eef_pose.linear());
  Eigen::Vector3d pos = eef_pose.translation();

  std::vector<int> disc_coll_coeffs(1, 50);
  std::vector<double> disc_coll_pen(1, 0.040);

  basic_info_j = { {"n_steps", 1}, {"manip", "whole_body"}, {"start_fixed", false} };
  disc_coll_j = { {"type", "collision"}, {"params", { {"coeffs", disc_coll_coeffs}, {"continuous", false}, {"dist_pen", disc_coll_pen} } } };
  eef_pose_j = { {"type", "pose"}, {"params", { {"xyz", {pos(0), pos(1), pos(2)}}, {"wxyz", {rot.w(), rot.x(), rot.y(), rot.z()}}, {"link", "hand_palm_link"}} } };
  init_info_j = { {"type", "stationary"} };

  opt_j["basic_info"] = basic_info_j;
  opt_j["costs"] = {disc_coll_j};
  opt_j["constraints"] = {eef_pose_j};
  opt_j["init_info"] = init_info_j;

  Json::Value v;
  Json::Reader r;

  std::string opt_j_str = opt_j.dump();
  std::cout << "here" << '\n';
  if (!r.parse(opt_j_str, v, false)) {
    std::cout << "can't read json!"  << std::endl;
    std::cout << r.getFormatedErrorMessages() << std::endl;
    return 0;
  }

  return v;
}
