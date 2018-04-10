#ifndef FRASIER_CONTROLLER_H_
#define FRASIER_CONTROLLER_H_

// ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/ListControllers.h>

// Other
#include <Eigen/Core>

class FRASIERController{

public:
  FRASIERController(ros::NodeHandle n);
  // ~FRASIERController();

  void sendWholeBodyTraj(Eigen::MatrixXd& traj);
  void moveToStartState();


private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_cli_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> base_cli_;

};


#endif
