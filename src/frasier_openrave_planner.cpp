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

  hsr_->GetController()->SetPath(traj);


}

void FRASIEROpenRAVE::solveIK(){
  std::string json_path;
  json_path = config_path_ + "whole_body_ik.json";

  Json::Value whole_body_ik_json;
  Json::Reader reader;
  std::ifstream i(json_path);

  bool json_parsing_success = reader.parse(i, whole_body_ik_json, false);

  if (!json_parsing_success) {
    std::cout << "can't read json!"  << std::endl;
  }

  trajopt::TrajOptProbPtr traj_prob = trajopt::ConstructProblem(whole_body_ik_json, env_);

  trajopt::TrajOptResultPtr traj_result = trajopt::OptimizeProblem(traj_prob, false);
  std::cout << "traj : " << std::endl << traj_result->traj << std::endl;


}

void FRASIEROpenRAVE::computeTrajectory(){
  std::string json_path;
  json_path = config_path_ + "whole_body_traj.json";

  Json::Value whole_body_traj_json;
  Json::Reader reader;
  std::ifstream i(json_path);

  bool json_parsing_success = reader.parse(i, whole_body_traj_json, true);

  if (!json_parsing_success) {
    std::cout << "can't read json!"  << std::endl;
    std::cout << reader.getFormatedErrorMessages() << std::endl;
  }

  trajopt::TrajOptProbPtr traj_prob = trajopt::ConstructProblem(whole_body_traj_json, env_);

  trajopt::TrajOptResultPtr traj_result = trajopt::OptimizeProblem(traj_prob, false);
  std::cout << "traj : " << std::endl << traj_result->traj << std::endl;

}
