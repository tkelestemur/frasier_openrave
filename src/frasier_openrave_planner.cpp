#include <frasier_openrave/frasier_openrave.h>

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
//TODO: Should I set active dofs???
trajectory_msgs::JointTrajectory FRASIEROpenRAVE::computeTrajectory(EEFPoseGoals& eef_goals, bool plot){
  std::cout << "RAVE: planning trajectory..." << std::endl;
  OpenRAVE::EnvironmentBasePtr planning_env = env_->CloneSelf(OpenRAVE::Clone_Bodies);

  Json::Value opt_j = createJsonValueTraj(eef_goals);

  trajopt::TrajOptProbPtr traj_prob = trajopt::ConstructProblem(opt_j, planning_env);
  trajopt::TrajOptResultPtr result = trajopt::OptimizeProblem(traj_prob, plot);

  planning_env->Destroy();
  Eigen::MatrixXd traj = result->traj;

  return eigenMatrixToTraj(traj);

}

// TODO: create planning env and destroy it insetad of updating planning_env_
void FRASIEROpenRAVE::computeIK(OpenRAVE::Transform& eef_pose, Eigen::VectorXd&  q_sol, bool check_coll){
  std::cout << "RAVE: computing IK..." << std::endl;

  Json::Value opt_j = createJsonValueIK(eef_pose, check_coll);

  updatePlanningEnv();

  trajopt::TrajOptProbPtr traj_prob = trajopt::ConstructProblem(opt_j, planning_env_);
  trajopt::TrajOptResultPtr traj_result = trajopt::OptimizeProblem(traj_prob, plan_plotter_);
  trajopt::ConstraintPtr ineq;
  
  q_sol = traj_result->traj.row(0);

}

Json::Value FRASIEROpenRAVE::createJsonValueTraj(EEFPoseGoals& eef_goals){

  int n_steps = eef_goals.no_waypoints;
  int first_step = 0; int last_step = n_steps-1;

  json opt_j, disc_coll_j, cont_coll_j, joint_vel_j, basic_info_j, init_info_j;

  OpenRAVE::Transform hsr_pose = hsr_->GetLink("base_link")->GetTransform();
  int n_joints = manip_->GetArmDOF();

  std::vector<json> eef_pose_j;
  eef_pose_j.resize(eef_goals.n_goals);


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



  init_info_j = { {"type", "stationary"} };

  opt_j["basic_info"] = basic_info_j;
  opt_j["costs"] = {joint_vel_j, disc_coll_j, cont_coll_j};
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

Json::Value FRASIEROpenRAVE::createJsonValueTraj(JointPosGoals& joint_goals){
  json opt_j, joint_target_j, disc_coll_j, cont_coll_j, joint_vel_j, basic_info_j, init_info_j;

  int n_steps = joint_goals.no_waypoints;
  int n_joints = manip_->GetArmDOF();

  int first_step = 0; int last_step = n_steps-1;

  std::vector<double> joint_vel_coeffs(n_joints, 1.0);
  std::vector<int> disc_coll_coeffs(n_steps, 50);
  std::vector<double> disc_coll_pen(n_steps, 0.050);
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
  int no_waypoints = traj.points.size();
  ecl::Trajectory<ecl::JointAngles> ecl_traj(no_dof);
  ecl::WayPoint<ecl::JointAngles> waypoint(no_dof);
  ecl::Array<double> acc_limits(no_dof);

  acc_limits << 0.3, 0.3, 0.5, 0.15, 1.0, 1.0, 1.0, 1.0;

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


void FRASIEROpenRAVE::grabObject(std::string& obj_name) {
  OpenRAVE::KinBodyPtr grabbed_object = env_->GetKinBody(obj_name);
  hsr_->Grab(grabbed_object);
}

void FRASIEROpenRAVE::releaseObject(std::string& obj_name) {
  OpenRAVE::KinBodyPtr released_object = env_->GetKinBody(obj_name);
  hsr_->Release(released_object);
}

////////////////////////////// UTILITIES //////////////////////////////
trajectory_msgs::JointTrajectory FRASIEROpenRAVE::eigenMatrixToTraj(Eigen::MatrixXd &traj) {

  int n_joints = manip_->GetArmDOF();

  trajectory_msgs::JointTrajectory traj_ros;

  traj_ros.joint_names.push_back("odom_x");
  traj_ros.joint_names.push_back("odom_y");
  traj_ros.joint_names.push_back("odom_t");
  traj_ros.joint_names.push_back("arm_lift_joint");
  traj_ros.joint_names.push_back("arm_flex_joint");
  traj_ros.joint_names.push_back("arm_roll_joint");
  traj_ros.joint_names.push_back("wrist_flex_joint");
  traj_ros.joint_names.push_back("wrist_roll_joint");


  traj_ros.points.resize(traj.rows());

  for (int i = 0; i < traj.rows(); i++) {
    traj_ros.points[i].positions.resize(n_joints);
    traj_ros.points[i].velocities.resize(n_joints);
    traj_ros.points[i].accelerations.resize(n_joints);

    for (int j = 0; j < n_joints; j++) {
      traj_ros.points[i].positions[j] = traj(i, j);
      traj_ros.points[i].velocities[j] = 0.0;
      traj_ros.points[i].accelerations[j] = 0.0;

    }
  }

  return traj_ros;
}

void FRASIEROpenRAVE::drawTransform(OpenRAVE::Transform &T) {
  OpenRAVE::TransformMatrix M;
  OpenRAVE::geometry::matrixFromQuat(M, T.rot);
  double arrow_length = 0.1;
  float arrow_width = 0.1;
  OpenRAVE::Vector origin(T.trans);
  OpenRAVE::Vector x_axis(M.rot(0, 0), M.rot(1, 0), M.rot(2, 0));
//  std::cout << "origin: " << origin << std::endl;
//  std::cout << "vector: " << arrow_length*origin+x_axis << std::endl;
  OpenRAVE::GraphHandlePtr arrow_x = env_->drawarrow(origin, arrow_length*origin+x_axis, arrow_width, OpenRAVE::Vector(1, 0, 0));
  arrow_x->SetShow(true);
}

void FRASIEROpenRAVE::playTrajectory(trajectory_msgs::JointTrajectory& traj){
  std::vector<int> q_index;
  getWholeBodyJointIndex(q_index);

  for (int i = 0; i < traj.points.size(); i++) {
    hsr_->SetDOFValues(traj.points[i].positions, 1, q_index);
    ros::Duration(traj.points[1].time_from_start).sleep();
  }
}

////////////////////////////// GRASPING //////////////////////////////



OpenRAVE::Transform FRASIEROpenRAVE::generateGraspPose(std::string &obj_name) {
  OpenRAVE::Transform hsr_pose = hsr_->GetLink(base_link_)->GetTransform();
  OpenRAVE::KinBodyPtr object = env_->GetKinBody(obj_name);

  OpenRAVE::Transform grasp_pose = hsr_pose.inverse() * object->GetTransform();
  grasp_pose.rot = OpenRAVE::Vector(0.5, -0.5, -0.5, -0.5);
  grasp_pose.trans.y = grasp_pose.trans.y - 0.03;

  return grasp_pose;
}

OpenRAVE::Transform FRASIEROpenRAVE::generatePlacePose(std::string &obj_name) {
  OpenRAVE::Transform place_pose;
  place_pose.rot = OpenRAVE::Vector(0.0, 0.707, 0.0, 0.707);

  OpenRAVE::KinBodyPtr object = env_->GetKinBody(obj_name);
  OpenRAVE::Transform object_pose = object->GetTransform();
  std::string shelf_name = "shelf_1";
  OpenRAVE::KinBodyPtr shelf = env_->GetKinBody(shelf_name);
  OpenRAVE::Transform shelf_pose = shelf->GetTransform();

  if(object_pose.trans.y < shelf_pose.trans.y){
    place_pose.trans = object_pose.trans;
    place_pose.trans.y = place_pose.trans.y + 0.2;
    place_pose.trans.x = place_pose.trans.x + 0.1;
  }else{
    place_pose.trans = object_pose.trans;
    place_pose.trans.y = place_pose.trans.y - 0.2;
    place_pose.trans.x = place_pose.trans.x + 0.1;
  }

  return place_pose;
}

Grasp FRASIEROpenRAVE::generateGraspPose(){
  OpenRAVE::Transform hsr_pose = hsr_->GetLink(base_link_)->GetTransform();

  Grasp grasp;
  std::vector <OpenRAVE::KinBodyPtr> bodies;
  env_->GetBodies(bodies);
  double closest_distance = std::numeric_limits<double>::max();
  for (int i = 0; i < bodies.size(); i++) {
    std::string body_name = bodies[i]->GetName();

    if (body_name.substr(0,3) == "obj"){
      std::cout << "RAVE: creating a grasp pose for " << body_name << std::endl;

      OpenRAVE::Transform obj_pose = hsr_pose.inverse() * bodies[i]->GetTransform();

      double distance = std::sqrt(std::pow(obj_pose.trans.x, 2) + std::pow(obj_pose.trans.y, 2));
      if (distance < closest_distance){

        obj_pose.rot = OpenRAVE::Vector(0.5, -0.5, -0.5, -0.5);
        obj_pose.trans.y = obj_pose.trans.y - 0.03;

        grasp.pose = obj_pose;
        grasp.obj_name = body_name;
        closest_distance = distance;

      }
    }

  }

  return grasp;
}

std::vector<OpenRAVE::Transform> FRASIEROpenRAVE::generatePlacePoses(){
  OpenRAVE::Transform hsr_pose = hsr_->GetLink(base_link_)->GetTransform();

  std::vector<OpenRAVE::Transform> place_poses;

  std::vector <OpenRAVE::KinBodyPtr> bodies;
  env_->GetBodies(bodies);
  for (int i = 0; i < bodies.size(); i++) {
    std::string body_name = bodies[i]->GetName();

    if (body_name.substr(0,5) == "shelf"){
      std::cout << "RAVE: creating a place pose for " << body_name << std::endl;

//      OpenRAVE::Transform place_pose = hsr_pose.inverse() * bodies[i]->GetTransform();
      OpenRAVE::Transform place_pose = bodies[i]->GetTransform();
      place_pose.rot = OpenRAVE::Vector(0.0, 0.707, 0.0, 0.707);
      place_pose.trans.z = place_pose.trans.z + 0.1;

      place_poses.push_back(place_pose);

    }

  }
  return place_poses;
}