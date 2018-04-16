#ifndef FRASIER_CONTROLLER_H_
#define FRASIER_CONTROLLER_H_

// ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/ListControllers.h>
#include <tmc_manipulation_msgs/FilterJointTrajectory.h>
#include <tmc_manipulation_msgs/ArmNavigationErrorCodes.h>

// Other
#include <Eigen/Core>

enum MOVE_STATE{
    PICK,
    HOME
};

class FRASIERController{



public:
  FRASIERController(ros::NodeHandle n);
  // ~FRASIERController();
  void jointSensorCb(const sensor_msgs::JointState::ConstPtr &msg);
  void sendWholeBodyTraj(Eigen::MatrixXd& traj);
  void moveToStartState(MOVE_STATE state);
  bool filterTrajectory(Eigen::MatrixXd& traj,
                        trajectory_msgs::JointTrajectory& whole_body_traj);
  void extractArmBaseTraj(trajectory_msgs::JointTrajectory whole_body_traj,
                           trajectory_msgs::JointTrajectory& base_traj,
                           trajectory_msgs::JointTrajectory& arm_traj);
  void executeWholeBodyTraj(Eigen::MatrixXd& traj);
  void executeWholeBodyTraj(trajectory_msgs::JointTrajectory whole_body_traj);
  void setStartState(sensor_msgs::JointState& start_state);

private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_cli_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> base_cli_;

  ros::ServiceClient filter_traj_srv_;
  ros::Subscriber joint_state_sub_;
  sensor_msgs::JointState start_state_;

  bool joint_state_flag_;

  boost::mutex joint_state_mutex_;

};


#endif
