#ifndef FRASIER_CONTROLLER_H_
#define FRASIER_CONTROLLER_H_

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/ListControllers.h>
#include <tmc_manipulation_msgs/FilterJointTrajectory.h>
#include <tmc_manipulation_msgs/ArmNavigationErrorCodes.h>
#include <tmc_control_msgs/GripperApplyEffortAction.h>
#include <tmc_control_msgs/GripperApplyEffortGoal.h>




// Other
#include <Eigen/Core>
#include <iostream>
#include <fstream>

enum MOVE_STATE{
    PICK,
    HOME,
    TABLE,
    SHELF
};

enum HEAD_STATE{
    LOOK_TABLE,
    LOOK_SHELF
};

enum GRIPPER_STATE{
    GRASP,
    RELEASE
};

class FRASIERController{

public:
  FRASIERController(ros::NodeHandle n);
  // ~FRASIERController();
  void jointSensorCb(const sensor_msgs::JointState::ConstPtr &msg);

  void moveToKnownState(MOVE_STATE state);
  void moveHeadToKnownState(HEAD_STATE state);

//  bool filterTrajectory(trajectory_msgs::JointTrajectory& traj,
//                        trajectory_msgs::JointTrajectory& traj_filtered);


  void extractArmBaseTraj(trajectory_msgs::JointTrajectory whole_body_traj,
                           trajectory_msgs::JointTrajectory& base_traj,
                           trajectory_msgs::JointTrajectory& arm_traj);

  void executeArmBaseTraj(trajectory_msgs::JointTrajectory& traj);
  void executeWholeBodyTraj(trajectory_msgs::JointTrajectory& traj,
                            bool execute_gripper=false);
  void executeGraspTraj(trajectory_msgs::JointTrajectory& traj);

  void graspOrRelease(GRIPPER_STATE state);



//  void setStartState(sensor_msgs::JointState& start_state);

private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_cli_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> base_cli_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> head_cli_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> whole_body_cli_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> gripper_pos_cli_;
  actionlib::SimpleActionClient<tmc_control_msgs::GripperApplyEffortAction> gripper_cli_;


  ros::ServiceClient filter_traj_srv_;
  sensor_msgs::JointState start_state_;

  bool joint_state_flag_;

  boost::mutex joint_state_mutex_;
  const double CONTROLLER_TIMEOUT = 20.0;
};


#endif
