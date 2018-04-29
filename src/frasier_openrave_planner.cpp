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
//
//  std::vector<double> traj_0;
//  traj->Sample(traj_0, 0.1);
//  for (size_t i = 0; i < traj_0.size(); i++) {
//    std::cout << traj_0[i] << '\n';
//  }
//
//
//}

////////////////////////////// TRAJOPT //////////////////////////////
trajectory_msgs::JointTrajectory FRASIEROpenRAVE::computeTrajectory(std::vector<double>& q){

  OpenRAVE::EnvironmentBasePtr planning_env = env_->CloneSelf(OpenRAVE::Clone_Bodies);

  Json::Value opt_j = createJsonValueTraj(q, 10);

  trajopt::TrajOptProbPtr traj_prob = trajopt::ConstructProblem(opt_j, planning_env);
  trajopt::TrajOptResultPtr result = trajopt::OptimizeProblem(traj_prob, plan_plotter_);

  planning_env->Destroy();
  Eigen::MatrixXd traj = result->traj;

  return eigenMatrixToTraj(traj);

}
//TODO: Should I set active dofs???
trajectory_msgs::JointTrajectory FRASIEROpenRAVE::computeTrajectory(EEFPoseGoals& eef_goals){

  OpenRAVE::EnvironmentBasePtr planning_env = env_->CloneSelf(OpenRAVE::Clone_Bodies);

  Json::Value opt_j = createJsonValueTraj(eef_goals);

  trajopt::TrajOptProbPtr traj_prob = trajopt::ConstructProblem(opt_j, planning_env);
  trajopt::TrajOptResultPtr result = trajopt::OptimizeProblem(traj_prob, plan_plotter_);

  planning_env->Destroy();
  Eigen::MatrixXd traj = result->traj;

  return eigenMatrixToTraj(traj);

}

// TODO: create planning env and destroy it insetad of updating planning_env_
void FRASIEROpenRAVE::computeIK(OpenRAVE::Transform& eef_pose, Eigen::VectorXd&  q_sol, bool check_coll){

  Json::Value opt_j = createJsonValueIK(eef_pose, check_coll);

  updatePlanningEnv();

  trajopt::TrajOptProbPtr traj_prob = trajopt::ConstructProblem(opt_j, planning_env_);
  trajopt::TrajOptResultPtr traj_result = trajopt::OptimizeProblem(traj_prob, plan_plotter_);

  q_sol = traj_result->traj.row(0);

}

Json::Value FRASIEROpenRAVE::createJsonValueTraj(EEFPoseGoals& eef_goals){
  json opt_j, disc_coll_j, cont_coll_j, joint_vel_j, basic_info_j, init_info_j;

  OpenRAVE::Transform hsr_pose = hsr_->GetLink("base_link")->GetTransform();

  std::vector<json> eef_pose_j;
  eef_pose_j.resize(eef_goals.n_goals);

  int n_steps = eef_goals.no_waypoints;
  int first_step = 0; int last_step = n_steps-1;

  std::vector<double> joint_vel_coeffs(8, 1.0);
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

Json::Value FRASIEROpenRAVE::createJsonValueTraj(std::vector<double>& q, int n_steps){
  json opt_j, joint_target_j, disc_coll_j, cont_coll_j, joint_vel_j, basic_info_j, init_info_j;


  int first_step = 0; int last_step = n_steps-1;

  std::vector<double> joint_vel_coeffs(8, 1.0);
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

  joint_target_j = { {"type", "joint"},
                     {"params", {
                                  {"vals", q} } }
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
    traj_ros.points[i].positions.resize(8);

    for (int j = 0; j < 8; j++) {
      traj_ros.points[i].positions[j] = traj(i, j);

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

void FRASIEROpenRAVE::playTrajectory(Eigen::MatrixXd& traj){
  std::vector<int> q_index;
  getWholeBodyJointIndex(q_index);

  for (int i = 0; i < traj.rows(); i++) {
    hsr_->SetDOFValues(trajopt::trajToDblVec(traj.row(i)), 1, q_index);
    ros::Duration(0.5).sleep();
  }
}

////////////////////////////// GRASPING //////////////////////////////
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


OpenRAVE::Transform FRASIEROpenRAVE::generateGraspPose(std::string &obj_name) {
  OpenRAVE::Transform hsr_pose = hsr_->GetLink(base_link_)->GetTransform();
  OpenRAVE::KinBodyPtr object = env_->GetKinBody(obj_name);

  OpenRAVE::Transform grasp_pose = hsr_pose.inverse() * object->GetTransform();
  grasp_pose.rot = OpenRAVE::Vector(0.5, -0.5, -0.5, -0.5);
  grasp_pose.trans.y = grasp_pose.trans.y - 0.03;

  return grasp_pose;
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

      OpenRAVE::Transform place_pose = hsr_pose.inverse() * bodies[i]->GetTransform();

      place_pose.rot = OpenRAVE::Vector(0.5, 0.5, -0.5, 0.5);
      place_pose.trans.z = place_pose.trans.z + 0.12;

      place_poses.push_back(place_pose);

    }

  }
  return place_poses;
}