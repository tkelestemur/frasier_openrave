#include <frasier_openrave/frasier_openrave.h>

void FRASIEROpenRAVE::initPlanner(){
  manip_ = hsr_->GetManipulator("whole_body");
  planner_ = OpenRAVE::RaveCreatePlanner(env_, "birrt");
  hsr_->SetActiveDOFs(manip_->GetArmIndices());
}


void FRASIEROpenRAVE::planToConf(){

  OpenRAVE::PlannerBase::PlannerParametersPtr params(new OpenRAVE::PlannerBase::PlannerParameters());
  params->_nMaxIterations = 4000; // max iterations before failure
  params->SetRobotActiveJoints(hsr_); // set planning configuration space to current active dofs
  params->vgoalconfig.resize(hsr_->GetActiveDOF());
  std::cout << "active dofs: " << hsr_->GetActiveDOF() << std::endl;

  {
    OpenRAVE::RobotBase::RobotStateSaver saver(hsr_); // save the state

    params->vgoalconfig[0] = 0.0;
    params->vgoalconfig[1] = 0.0;
    params->vgoalconfig[2] = 0.0;
    params->vgoalconfig[3] = 0.4;
    params->vgoalconfig[4] = -0.3;
    params->vgoalconfig[5] = 0.0;
    params->vgoalconfig[6] = 0.0;
    params->vgoalconfig[7] = 0.0;

    if (hsr_->CheckSelfCollision()) {
      std::cout << "self collision detected!" << std::endl;
    }

    if(env_->CheckCollision(hsr_)) {
      std::cout << "env collision detected!" << std::endl;
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
