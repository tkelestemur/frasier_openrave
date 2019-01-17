#include <frasier_openrave/frasier_openrave.h>
#include <geometry_msgs/Pose.h>

////////////////////////////// RRT //////////////////////////////
//void FRASIEROpenRAVE::planToConf(std::vector<double>& q){
//
//  planner_ = OpenRAVE::RaveCreatePlanner(env_, "birrt");
//  hsr_->SetActiveDOFs(manip_->GetArmIndices());
//
//  OpenRAVE::PlannerBase::PlannerParametersPtr params(new OpenRAVE::PlannerBase::PlannerParameters());
//  params->_nMaxIterations = 1000; // max iterations before failure
//  params->SetRobotActiveJoints(hsr_); // set planning configuration space to current active dofs
//  params->vgoalconfig.resize(hsr_->GetActiveDOF());
//  std::cout << "active dofs: " << hsr_->GetActiveDOF() << std::endl;
//
//  {
//    OpenRAVE::RobotBase::RobotStateSaver saver(hsr_); // save the state
//
//    for (int i = 0; i < hsr_->GetActiveDOF(); i++) {
//      params->vgoalconfig[i] = q[i];
//    }
//
//    hsr_->SetActiveDOFValues(params->vgoalconfig);
//
//    if (hsr_->CheckSelfCollision()) {
//      std::cout << "self collision detected!" << std::endl;
//      return;
//    }
//
//    if(env_->CheckCollision(hsr_)) {
//      std::cout << "env collision detected!" << std::endl;
//      return;
//    }
//
//  }
//
//  hsr_->GetActiveDOFValues(params->vinitialconfig);
//
//  if( !planner_->InitPlan(hsr_, params) ) {
//      std::cout << "couldn't init the plan" << std::endl;
//      return;
//  }
//
//  OpenRAVE::TrajectoryBasePtr traj = OpenRAVE::RaveCreateTrajectory(env_);
//  OpenRAVE::PlannerStatus status = planner_->PlanPath(traj);
//
//  std::cout << "plan status : " << status << std::endl;
//  if(status & OpenRAVE::PlannerStatus::PS_HasSolution) {
//      RAVELOG_WARN("Found plan\n");
//  }
//  std::cout << "traj time " << traj->GetDuration()<< std::endl;

//}

////////////////////////////// TRAJOPT //////////////////////////////
trajectory_msgs::JointTrajectory FRASIEROpenRAVE::computeTrajectory(JointPosGoals& joint_goals, bool plot){
  std::cout << "RAVE: planning trajectory..." << std::endl;
  OpenRAVE::EnvironmentBasePtr planning_env = env_->CloneSelf(OpenRAVE::Clone_Bodies);

  Json::Value opt_j = createJsonValueTraj(joint_goals);

  trajopt::TrajOptProbPtr traj_prob = trajopt::ConstructProblem(opt_j, planning_env);
  trajopt::TrajOptResultPtr result = trajopt::OptimizeProblem(traj_prob, plot);

  planning_env->Destroy();
  Eigen::MatrixXd traj = result->traj;

  return eigenMatrixToTraj(traj);

}

trajectory_msgs::JointTrajectory FRASIEROpenRAVE::computeTrajectory(EEFPoseGoals& eef_goals, bool plot){
  std::cout << "RAVE: planning trajectory..." << std::endl;
  OpenRAVE::EnvironmentBasePtr planning_env = env_->CloneSelf(OpenRAVE::Clone_Bodies);

  std::vector<OpenRAVE::KinBodyPtr> bodies;
  planning_env->GetBodies(bodies);

  // disabled links needs to be removed from the env because trajopt doesn't check it
  for(const auto body : bodies){
    if(!body->IsEnabled()){
      planning_env->Remove(body);
    }
  }

  Json::Value opt_j = createJsonValueTraj(eef_goals);

  boost::posix_time::ptime start_time;
  boost::posix_time::time_duration diff;
  start_time = boost::posix_time::microsec_clock::local_time();

  trajopt::TrajOptProbPtr traj_prob = trajopt::ConstructProblem(opt_j, planning_env);
  trajopt::TrajOptResultPtr result = trajopt::OptimizeProblem(traj_prob, plot);

  diff = boost::posix_time::microsec_clock::local_time() - start_time;
  double elapsed = diff.total_nanoseconds() /1e9;

  std::cout << "RAVE: trajectory optimized in " << elapsed << " seconds!" << std::endl;

//  for(int i = 0; i < result->cost_names.size(); i++){
//    std::cout << result->cost_names[i] << " : " << result->cost_vals[i] << std::endl;
//  }


  planning_env->Destroy();
  Eigen::MatrixXd traj = result->traj;
  return eigenMatrixToTraj(traj);

}

void FRASIEROpenRAVE::computeIK(OpenRAVE::Transform& eef_pose, Eigen::VectorXd&  q_sol, bool check_coll){
  std::cout << "RAVE: computing IK..." << std::endl;

  Json::Value opt_j = createJsonValueIK(eef_pose, check_coll);
  OpenRAVE::EnvironmentBasePtr planning_env = env_->CloneSelf(OpenRAVE::Clone_Bodies);

  trajopt::TrajOptProbPtr traj_prob = trajopt::ConstructProblem(opt_j, planning_env);
  trajopt::TrajOptResultPtr traj_result = trajopt::OptimizeProblem(traj_prob, plan_plotter_);
  trajopt::ConstraintPtr ineq;
  
  q_sol = traj_result->traj.row(0);

}

Json::Value FRASIEROpenRAVE::createJsonValueTraj(EEFPoseGoals& eef_goals){

  int n_steps = eef_goals.no_waypoints;
  int first_step = 0; int last_step = n_steps-1;

  json opt_j, opt_costs_j, disc_coll_j, cont_coll_j, joint_vel_j, basic_info_j, init_info_j;

  OpenRAVE::Transform hsr_pose = hsr_->GetLink("base_link")->GetTransform();
  int n_joints = manip_->GetArmDOF();

//  std::vector<json> eef_pose_cost_j;
//  json eef_pose_cost_j;
  json eef_pose_const_j;
//  std::vector<json> eef_pose_const_j;
//  eef_pose_j.resize(eef_goals.n_goals);


  std::vector<double> joint_vel_coeffs(n_joints, 1.0);
  std::vector<int> disc_coll_coeffs(n_steps, COLLISION_COEFF);
  std::vector<double> disc_coll_pen(n_steps, COLLISION_PENALTY);

  basic_info_j = { {"n_steps", n_steps}, {"manip", "whole_body"}, {"start_fixed", true} };

  joint_vel_j = { {"type", "joint_vel"}, {"params", { {"coeffs", joint_vel_coeffs} } } };
  opt_costs_j.push_back(joint_vel_j);

  disc_coll_j = { {"type", "collision"},
                  {"params", {
                             {"coeffs", disc_coll_coeffs},
                             {"continuous", false},
                             {"dist_pen", disc_coll_pen},
                             {"first_step", first_step},
                             {"last_step", last_step} } }
  };
  opt_costs_j.push_back(disc_coll_j);

  cont_coll_j = { {"type", "collision"},
                  {"params", {
                             {"coeffs", disc_coll_coeffs},
                             {"continuous", true},
                             {"dist_pen", disc_coll_pen},
                             {"first_step", first_step},
                             {"last_step", last_step} } }
  };
  opt_costs_j.push_back(cont_coll_j);


//  json pose_j;
  for (int i = 0; i < eef_goals.n_goals; i++) {
    OpenRAVE::Transform eef_pose_goal = eef_goals.poses[i];
    OpenRAVE::Transform eef_goal;

    if (eef_goals.wrt_world){
      eef_goal = eef_pose_goal;
    }
    else{
      // trasform from robot fixed frame to world frame
      eef_goal = hsr_pose * eef_pose_goal;
    }

    const json pose_j = { {"type", "pose"},
                      {"params", {
                                 {"xyz", {eef_goal.trans[0], eef_goal.trans[1], eef_goal.trans[2]}},
                                 {"wxyz", {eef_goal.rot[0], eef_goal.rot[1], eef_goal.rot[2], eef_goal.rot[3]}},
                                 {"link", "hand_palm_link"},
                                 {"timestep", eef_goals.timesteps[i]}
                               } }
    };


    if(eef_goals.pose_types[i] == CONSTRAINT){
//      eef_pose_cost_j.
      eef_pose_const_j.push_back(pose_j);
      std::cout << "RAVE: pose type: constraint" << std::endl;
//      std::cout << eef_pose_const_j.dump(4) << std::endl;
    }

    else if(eef_goals.pose_types[i] == COST){
      opt_costs_j.push_back(pose_j);
      std::cout << "RAVE: pose type: cost" << std::endl;
//      std::cout << eef_pose_cost_j.dump(4) << std::endl;
    }

//    std::cout << eef_pose_const_j.dump(4) << std::endl;
//    pose_j.clear();

  }



  init_info_j = { {"type", "stationary"} };

  opt_j["basic_info"] = basic_info_j;
//  opt_j["costs"] = {joint_vel_j, disc_coll_j, cont_coll_j, eef_pose_cost_j};
  opt_j["costs"] = opt_costs_j;
  opt_j["constraints"] = eef_pose_const_j;
  opt_j["init_info"] = init_info_j;


  Json::Value v;
  Json::Reader r;

  std::string opt_j_str = opt_j.dump();
//  std::cout << opt_j.dump(4) << std::endl;

  if (!r.parse(opt_j_str, v, false)) {
    std::cout << "can't read json!"  << std::endl;
    std::cout << r.getFormatedErrorMessages() << std::endl;
    return 0;
  }

  return v;
}

Json::Value FRASIEROpenRAVE::createJsonValueTraj(JointPosGoals& joint_goals){
  json opt_j, joint_target_j, disc_coll_j, cont_coll_j, joint_vel_j, basic_info_j, init_info_j;

  int n_steps = joint_goals.no_waypoints;
  int n_joints = manip_->GetArmDOF();

  int first_step = 0; int last_step = n_steps-1;

  std::vector<double> joint_vel_coeffs(n_joints, 1.0);
  std::vector<int> disc_coll_coeffs(n_steps, COLLISION_COEFF);
  std::vector<double> disc_coll_pen(n_steps, COLLISION_PENALTY);
  std::vector<double> joint_coeffs(n_joints, 1.0);

  basic_info_j = { {"n_steps", n_steps}, {"manip", "whole_body"}, {"start_fixed", true} };

  joint_vel_j = { {"type", "joint_vel"}, {"params", { {"coeffs", joint_vel_coeffs} } } };

  disc_coll_j = { {"type", "collision"},
                  {"params", {
                             {"coeffs", disc_coll_coeffs},
                             {"continuous", false},
                             {"dist_pen", disc_coll_pen},
                             {"first_step", first_step},
                             {"last_step", last_step} } }
  };

  cont_coll_j = { {"type", "collision"},
                  {"params", {
                             {"coeffs", disc_coll_coeffs},
                             {"continuous", true},
                             {"dist_pen", disc_coll_pen},
                             {"first_step", first_step},
                             {"last_step", last_step} } }
  };


  joint_target_j = { {"type", "joint"}, // TODO: add timesteps
                     {"params", {
                                  {"vals", joint_goals.q},
                                  {"coeffs", joint_coeffs} } }
  };

  init_info_j = { {"type", "stationary"} };

  opt_j["basic_info"] = basic_info_j;
  opt_j["costs"] = {joint_vel_j, disc_coll_j, cont_coll_j};
  opt_j["constraints"] = {joint_target_j};
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

Json::Value FRASIEROpenRAVE::createJsonValueIK(OpenRAVE::Transform& eef_pose, bool check_coll){
  json opt_j, disc_coll_j, basic_info_j, init_info_j, eef_pose_j;

  std::vector<int> disc_coll_coeffs(1, 50);
  std::vector<double> disc_coll_pen(1, 0.040);

  basic_info_j = { {"n_steps", 1}, {"manip", "whole_body"}, {"start_fixed", false} };
  disc_coll_j = { {"type", "collision"}, {"params", { {"coeffs", disc_coll_coeffs}, {"continuous", false}, {"dist_pen", disc_coll_pen} } } };
  eef_pose_j = { {"type", "pose"}, {"params", { {"xyz", {eef_pose.trans[0], eef_pose.trans[1], eef_pose.trans[2]}}, {"wxyz", {eef_pose.rot[0], eef_pose.rot[1], eef_pose.rot[2], eef_pose.rot[3]}}, {"link", "hand_palm_link"}} } };
  init_info_j = { {"type", "stationary"} };

  opt_j["basic_info"] = basic_info_j;
  opt_j["costs"] = {disc_coll_j};
  opt_j["constraints"] = {eef_pose_j};
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

trajectory_msgs::JointTrajectory FRASIEROpenRAVE::computeOnTheFlyTraj(EEFPoseGoals &eef_goals, bool plot) {

  int n_steps = eef_goals.no_waypoints;
  int first_step = 0; int last_step = n_steps-1;

  json opt_j, disc_coll_j, cont_coll_j, joint_vel_j, basic_info_j, init_info_j;

  OpenRAVE::Transform hsr_pose = hsr_->GetLink("base_link")->GetTransform();
  int n_joints = manip_->GetArmDOF();

  std::vector<json> eef_pose_j;
  eef_pose_j.resize(eef_goals.n_goals);

  std::vector<json> joint_target_j;
  joint_target_j.resize(n_steps);

  std::vector<double> joint_vel_coeffs(n_joints, 1.0);
  std::vector<int> disc_coll_coeffs(n_steps, 50);
  std::vector<double> disc_coll_pen(n_steps, 0.050);

  basic_info_j = { {"n_steps", n_steps}, {"manip", "whole_body"}, {"start_fixed", true} };

  joint_vel_j = { {"type", "joint_vel"}, {"params", { {"coeffs", joint_vel_coeffs} } } };

  disc_coll_j = { {"type", "collision"},
                  {"params", {
                             {"coeffs", disc_coll_coeffs},
                             {"continuous", false},
                             {"dist_pen", disc_coll_pen},
                             {"first_step", first_step},
                             {"last_step", last_step} } }
  };

  cont_coll_j = { {"type", "collision"},
                  {"params", {
                             {"coeffs", disc_coll_coeffs},
                             {"continuous", true},
                             {"dist_pen", disc_coll_pen},
                             {"first_step", first_step},
                             {"last_step", last_step} } }
  };

  for (int i = 0; i < eef_goals.n_goals; i++) {
    OpenRAVE::Transform eef_pose_goal = eef_goals.poses[i];
    OpenRAVE::Transform eef_goal;

    if (eef_goals.wrt_world){
      eef_goal = eef_pose_goal;
    }
    else{
      eef_goal = hsr_pose * eef_pose_goal; // trasform from robot fixed frame to world frame
    }


    eef_pose_j[i] = { {"type", "pose"},
                      {"params", {
                                 {"xyz", {eef_goal.trans[0], eef_goal.trans[1], eef_goal.trans[2]}},
                                 {"wxyz", {eef_goal.rot[0], eef_goal.rot[1], eef_goal.rot[2], eef_goal.rot[3]}},
                                 {"link", "hand_palm_link"},
                                 {"timestep", eef_goals.timesteps[i]}
                               } }
    };


  }
  std::vector<double> q(n_joints, 0.0);

  std::vector<double> joint_coeffs(n_joints, 0.0);
  joint_coeffs[8] = 1.0;

  for (int j = 0; j < n_steps; j++) {
    if (j >= eef_goals.timesteps[0]){
      q[8] = eef_goals.aperture;
    }else{
      q[8] = hsr_->GetJoint("hand_motor_joint")->GetValue(0);
    }

    joint_target_j[j] = { {"type", "joint"},
                          {"params", {
                                     {"vals", q},
                                     {"coeffs", joint_coeffs},
                                     {"timestep", j}
                                   } }
    };
  }


  init_info_j = { {"type", "stationary"} };

  opt_j["basic_info"] = basic_info_j;
  opt_j["costs"] = {joint_vel_j, disc_coll_j, cont_coll_j};
  opt_j["constraints"] = {eef_pose_j[0], eef_pose_j[1],  joint_target_j[1], joint_target_j[2], joint_target_j[3],
                          joint_target_j[4], joint_target_j[5], joint_target_j[6], joint_target_j[7], joint_target_j[8], joint_target_j[9]};
//  opt_j["constraints"] = {eef_pose_j[0], eef_pose_j[1]};
//  opt_j["constraints"] = {eef_pose_j[0], eef_pose_j[1], joint_target_j[1]};
  opt_j["init_info"] = init_info_j;


  Json::Value v;
  Json::Reader r;

  std::string opt_j_str = opt_j.dump();
  std::cout << opt_j.dump(4) << std::endl;

  if (!r.parse(opt_j_str, v, false)) {
    std::cout << "can't read json!"  << std::endl;
    std::cout << r.getFormatedErrorMessages() << std::endl;
  }
}
// TODO: Try velocity limits
void FRASIEROpenRAVE::smoothTrajectory(trajectory_msgs::JointTrajectory &traj,
                                         trajectory_msgs::JointTrajectory &traj_smoothed) {

  int no_dof = manip_->GetArmDOF();;
  int no_waypoints = static_cast<int>(traj.points.size());
  ecl::Trajectory<ecl::JointAngles> ecl_traj(no_dof);
  ecl::WayPoint<ecl::JointAngles> waypoint(no_dof);
  ecl::Array<double> acc_limits(no_dof);

  acc_limits << 0.3-0.1, 0.3-0.1, 0.5-0.2, 0.15, 1.0, 1.0, 1.0, 1.0;

  for (int i = 0; i < no_waypoints; i++) {

    ecl::Array<double> q(no_dof);

    for (int j = 0; j < no_dof; j++) {
      q[j] = traj.points[i].positions[j];
    }
    waypoint.angles() = q;
    waypoint.nominalRates(1.0);
    ecl_traj.append(waypoint);
  }

  std::cout << "RAVE: started smoothing the trajectory..." << std::endl;
  ecl_traj.maxAccelerations() = acc_limits;
  try {
    ecl_traj.tensionSplineInterpolation(4.0);
  }
  catch (ecl::StandardException &e){
    std::cout << e.what() << std::endl;
  }


  std::cout << "RAVE: smoothing is done! total duration : " << ecl_traj.duration() << std::endl;
  int N = 100;
  traj_smoothed.header.stamp = ros::Time::now();
  traj_smoothed.joint_names.resize(traj.joint_names.size());
  traj_smoothed.joint_names = traj.joint_names;
  traj_smoothed.points.resize(N);

  for (int k = 0; k <N; k++) {

    double t = k*(ecl_traj.duration())/N;

    traj_smoothed.points[k].positions.resize(no_dof);
    traj_smoothed.points[k].velocities.resize(no_dof);
    traj_smoothed.points[k].accelerations.resize(no_dof);
    traj_smoothed.points[k].time_from_start = ros::Duration(t);
    for (int x = 0; x < no_dof ; x++) {

      traj_smoothed.points[k].positions[x] = ecl_traj(x, t);
      traj_smoothed.points[k].velocities[x] = ecl_traj.derivative(x, t);
      traj_smoothed.points[k].accelerations[x] = ecl_traj.dderivative(x, t);

    }
  }

}

// TODO: Impelemt this
void FRASIEROpenRAVE::checkCollisions(trajectory_msgs::JointTrajectory& traj) {
//   OpenRAVE::EnvironmentMutex::scoped_lock lockenv(env_->GetMutex());
   OpenRAVE::CollisionReportPtr report(new OpenRAVE::CollisionReport());
  {
    OpenRAVE::RobotBase::RobotStateSaver saver(hsr_); // save the state
    for(int i=0; i < traj.points.size(); i++){

    }

  }

}

void FRASIEROpenRAVE::getJacobian() {
//  boost::multi_array<double, 2> jac;
//  OpenRAVE::Vector offset;
//  std::vector<double> jac_;
//  manip_->CalculateJacobian(jac_);
//  int eef_index = hsr_->GetActiveManipulator()->GetEndEffector()->GetIndex();
//  hsr_->SetActiveDOFs()
//  hsr_->CalculateActiveJacobian(eef_index, offset, jac);
//
//  for(auto j : jac_){
//    std::cout << j << std::endl;
//  }

//  std::cout << "jac size: " << jac.size() << std::endl;
//  std::cout << "jac shape: " << jac.shape()[2] << std::endl;

}