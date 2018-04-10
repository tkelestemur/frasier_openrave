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

  // hsr_->GetController()->SetPath(traj);d


}

void FRASIEROpenRAVE::computeIK(Eigen::VectorXd&  q_sol){
  std::string json_path;
  json_path = config_path_ + "whole_body_ik.json";

  Json::Value whole_body_ik_json;
  Json::Reader reader;
  std::ifstream i(json_path);

  if (!reader.parse(i, whole_body_ik_json, false)) {
    std::cout << "can't read json!"  << std::endl;
    return;
  }

  trajopt::TrajOptProbPtr traj_prob = trajopt::ConstructProblem(whole_body_ik_json, env_);

  trajopt::TrajOptResultPtr traj_result = trajopt::OptimizeProblem(traj_prob, false);
  std::cout << "traj : " << std::endl << traj_result->traj << std::endl;

  q_sol = traj_result->traj.row(0);

}

void FRASIEROpenRAVE::computeTrajectory(Eigen::MatrixXd& traj){ //TODO: Should I set active dofs???

  std::string json_path;
  json_path = config_path_ + "table_shelf.json";

  Json::Value whole_body_traj_json;
  Json::Reader reader;
  std::ifstream i(json_path);

  if (!reader.parse(i, whole_body_traj_json, true)) {
    std::cout << "can't read json!"  << std::endl;
    std::cout << reader.getFormatedErrorMessages() << std::endl;
    return;
  }


    updatePlanningEnv();

    manip_ = hsr_->GetManipulator("whole_body");
    hsr_->SetActiveDOFs(manip_->GetArmIndices());


    trajopt::ProblemConstructionInfo pci(planning_env_);
    pci.fromJson(whole_body_traj_json);
    // pci.basic_info.start_fixed = true;
    // pci.basic_info.n_steps = 10;
    // pci.basic_info.manip = "whole_body";
    //
    // pci.init_info.type = trajopt::InitInfo::STATIONARY;
    //
    // trajopt::TermInfo pose_cost;

    trajopt::TrajOptProbPtr traj_prob = trajopt::ConstructProblem(pci);

    trajopt::TrajOptResultPtr result = trajopt::OptimizeProblem(traj_prob, false);

    traj = result->traj;

    OpenRAVE::TrajectoryBasePtr or_traj = OpenRAVE::RaveCreateTrajectory(env_);
    std::vector<double> traj_0;
    traj_0 = trajopt::trajToDblVec(traj.row(0));

    // or_traj->Insert(1, traj_0, true);


}

void FRASIEROpenRAVE::playTrajectory(Eigen::MatrixXd& traj){
  std::vector<int> q_index;
  getWholeBodyJointIndex(q_index);

  for (int i = 0; i < traj.rows(); i++) {
    hsr_->SetDOFValues(trajopt::trajToDblVec(traj.row(i)), 1, q_index);
    ros::Duration(0.5).sleep();
  }
}
