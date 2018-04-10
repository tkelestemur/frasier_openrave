#ifndef FRASIER_OPENRAVE_H_
#define FRASIER_OPENRAVE_H_

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose2D.h>

// OpenRAVE
#include <openrave-0.9/openrave-core.h>
#include <openrave/openrave.h>
#include <openrave/kinbody.h>
#include <openrave/planningutils.h>
#include <openrave/geometry.h>
#include <openrave/trajectory.h>

#include <json/json.h>
#include <trajopt/problem_description.hpp>

// Other
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <Eigen/Core>


class FRASIEROpenRAVE{
public:
  FRASIEROpenRAVE(ros::NodeHandle n);
  ~FRASIEROpenRAVE();
  bool LoadHSR();

  void setViewer();
  void updateJointStates();
  void updateCollisionEnv(); //TODO: Anas
  void updatePlanningEnv();
  void getActiveJointIndex(std::vector<int>& q_index);
  void getWholeBodyJointIndex(std::vector<int>& q_index);

  void startThreads();
  void startROSSpinner();

  // Motion planning
  void initRRTPlanner();
  void planToConf(std::vector<double>& q);
  void computeIK(Eigen::VectorXd&  q_sol);
  void computeTrajectory(Eigen::MatrixXd& traj);
  void playTrajectory(Eigen::MatrixXd& traj);

  void jointSensorCb(const sensor_msgs::JointState::ConstPtr &msg);
  void pointCloudCb(); // TODO: Anas
  void baseStateCb(const geometry_msgs::Pose2D::ConstPtr &msg);


private:
  ros::NodeHandle nh_;
  ros::Subscriber joint_state_sub_, base_state_sub_;
  sensor_msgs::JointState joints_;
  geometry_msgs::Pose2D base_;
  std::string joint_state_topic_, base_state_topic_;
  std::string robot_name_, manip_name_, planner_name_;
  std::string package_path_, config_path_, worlds_path_;
  std::vector<std::string> joint_names_, base_names_, whole_body_joint_names_;

  bool joint_state_flag_, run_viewer_flag_, run_joint_updater_flag_;

  OpenRAVE::EnvironmentBasePtr env_, planning_env_;
  OpenRAVE::ViewerBasePtr viewer_;
  OpenRAVE::RobotBasePtr hsr_;
  OpenRAVE::RobotBase::ManipulatorPtr manip_;
  OpenRAVE::PlannerBasePtr planner_;
  OpenRAVE::ControllerBasePtr controller_;

  boost::thread viewer_thread_;
  boost::thread joint_state_thread_;
  boost::mutex joint_state_mutex_, base_state_mutex_;


};

#endif
