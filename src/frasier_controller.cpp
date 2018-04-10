#include <frasier_openrave/frasier_controller.h>

FRASIERController::FRASIERController(ros::NodeHandle n) : nh_(n),
                    arm_cli_("/hsrb/arm_trajectory_controller/follow_joint_trajectory", true),
                    base_cli_("/hsrb/omni_base_controller/follow_joint_trajectory", true){

  arm_cli_.waitForServer();
  base_cli_.waitForServer();

  ros::ServiceClient client = nh_.serviceClient<controller_manager_msgs::ListControllers>(
    "/hsrb/controller_manager/list_controllers");
    controller_manager_msgs::ListControllers list_controllers;
    bool running = false;
    while (running == false) {
      ros::Duration(0.1).sleep();
      if (client.call(list_controllers)) {
        for (unsigned int i = 0; i < list_controllers.response.controller.size(); i++) {
          controller_manager_msgs::ControllerState c = list_controllers.response.controller[i];
          if (c.name == "arm_trajectory_controller" && c.state == "running") {
            running = true;
          }
        }
      }
    }

}

// FRASIERController::~FRASIERController(){
//
// }


void FRASIERController::sendWholeBodyTraj(Eigen::MatrixXd& traj){

  /////// BASE TRAJECTORY ///////
  control_msgs::FollowJointTrajectoryGoal base_goal;
  trajectory_msgs::JointTrajectory base_traj;

  base_traj.header.stamp = ros::Time().now();

  base_traj.joint_names.push_back("odom_x");
  base_traj.joint_names.push_back("odom_y");
  base_traj.joint_names.push_back("odom_t");

  base_traj.points.resize(traj.rows());

  ros::Duration duration_base(0.1);

  for (int i = 0; i < traj.rows(); i++) {
    base_traj.points[i].positions.resize(3);
    base_traj.points[i].velocities.resize(3);
    for (int j = 0; j < 3; j++) {
      base_traj.points[i].positions[j] = traj(i, j);
      base_traj.points[i].velocities[j] = 0.0;
    }

    base_traj.points[i].time_from_start = duration_base;
    duration_base += ros::Duration(0.1);
  }

  base_goal.trajectory = base_traj;


  /////// ARM TRAJECTORY ///////
  control_msgs::FollowJointTrajectoryGoal arm_goal;
  trajectory_msgs::JointTrajectory arm_traj;

  arm_traj.header.stamp = ros::Time().now();

  arm_traj.joint_names.push_back("arm_lift_joint");
  arm_traj.joint_names.push_back("arm_flex_joint");
  arm_traj.joint_names.push_back("arm_roll_joint");
  arm_traj.joint_names.push_back("wrist_flex_joint");
  arm_traj.joint_names.push_back("wrist_roll_joint");

  arm_traj.points.resize(traj.rows());

  ros::Duration duration_arm(0.1);

  for (int i = 0; i < traj.rows(); i++) {
    arm_traj.points[i].positions.resize(5);
    arm_traj.points[i].velocities.resize(5);
    for (int j = 0; j < 5; j++) {
      arm_traj.points[i].positions[j] = traj(i, j+3);
      arm_traj.points[i].velocities[j] = 0.0;
    }

    arm_traj.points[i].time_from_start = duration_arm;
    duration_arm += ros::Duration(0.1);
  }


  arm_goal.trajectory = arm_traj;

  std::cout << "sending arm trajectory... "  << std::endl;
  std::cout << "sending base trajectory... "  << std::endl;
  arm_cli_.sendGoal(arm_goal);
  base_cli_.sendGoal(base_goal);
  arm_cli_.waitForResult(ros::Duration(3.0));
  base_cli_.waitForResult(ros::Duration(3.0));


}

void FRASIERController::moveToStartState(){ //TODO: Move base too

  control_msgs::FollowJointTrajectoryGoal arm_goal;
  trajectory_msgs::JointTrajectory arm_traj;

  arm_traj.joint_names.push_back("arm_lift_joint");
  arm_traj.joint_names.push_back("arm_flex_joint");
  arm_traj.joint_names.push_back("arm_roll_joint");
  arm_traj.joint_names.push_back("wrist_flex_joint");
  arm_traj.joint_names.push_back("wrist_roll_joint");


  arm_traj.points.resize(1);

  arm_traj.points[0].positions.resize(5);
  arm_traj.points[0].positions[0] = 0.0;
  arm_traj.points[0].positions[1] = 0.0;
  arm_traj.points[0].positions[2] = 0.0;
  arm_traj.points[0].positions[3] = 0.0;
  arm_traj.points[0].positions[4] = 0.0;
  arm_traj.points[0].velocities.resize(5);

  for (size_t i = 0; i < 5; ++i) {
    arm_traj.points[0].velocities[i] = 0.0;
  }
  arm_traj.points[0].time_from_start = ros::Duration(1.0);

  arm_goal.trajectory = arm_traj;

  std::cout << "moving to start state... "  << std::endl;
  arm_cli_.sendGoal(arm_goal);
  arm_cli_.waitForResult(ros::Duration(3.0));

}
