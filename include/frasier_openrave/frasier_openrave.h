#ifndef FRASIER_OPENRAVE_H_
#define FRASIER_OPENRAVE_H_

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>

// OpenRAVE
#include <openrave-0.9/openrave-core.h>
#include <openrave/openrave.h>
#include <openrave/kinbody.h>
#include <openrave/planningutils.h>
#include <openrave/geometry.h>

#include <json/json.h>
#include <trajopt/problem_description.hpp>
#include <trajopt/kinematic_terms.hpp>
#include <trajopt/trajectory_costs.hpp>

// Other
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>


// using namespace OpenRAVE;

class FRASIEROpenRAVE{
public:
  FRASIEROpenRAVE(ros::NodeHandle n);
  ~FRASIEROpenRAVE();
  bool LoadHSR();

  void setViewer();
  void updateJointStates();
  void getActiveJointIndex(std::vector<int>& q_index);

  void startThreads();
  void startROSSpinner();

  void initRRTPlanner();
  void planToConf(std::vector<double>& q);
  void solveIK();
  void computeTrajectory();


  void jointSensorCb(const sensor_msgs::JointState::ConstPtr &msg);

private:
  ros::NodeHandle nh_;
  ros::Subscriber joint_state_sub_;
  sensor_msgs::JointState joints_;
  std::string joint_state_topic_;
  std::string robot_name_, manip_name_, planner_name_;
  std::string package_path_, config_path_, worlds_path_;
  std::vector<std::string> joint_names_;

  bool joint_state_flag_, run_viewer_flag_, run_joint_updater_flag_;

  OpenRAVE::EnvironmentBasePtr env_, viewer_env;
  OpenRAVE::ViewerBasePtr viewer_;
  OpenRAVE::RobotBasePtr hsr_;
  OpenRAVE::RobotBase::ManipulatorPtr manip_;
  OpenRAVE::PlannerBasePtr planner_;



  boost::thread viewer_thread_;
  boost::thread joint_state_thread_;
  boost::mutex joint_state_mutex_;


};

#endif
