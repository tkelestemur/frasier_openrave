#ifndef FRASIER_OPENRAVE_H_
#define FRASIER_OPENRAVE_H_

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/manipulators.hpp>
#include <ecl/exceptions.hpp>

// OpenRAVE+TrajOpt
#include <openrave-0.9/openrave-core.h>
#include <openrave/openrave.h>
#include <openrave/kinbody.h>
#include <openrave/planningutils.h>
#include <openrave/geometry.h>
#include <openrave/trajectory.h>
#include <trajopt/problem_description.hpp>

// Other
#include <frasier_openrave/json.hpp>
//#include <HACD/hacdInterface.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <json/json.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

// PCL
//#include <pcl/PolygonMesh.h>

using json = nlohmann::json;

struct EEFPoseGoals{
  int n_goals;
  int no_waypoints;
  std::vector<OpenRAVE::Transform> poses;
  std::vector<int> timesteps;
  double aperture;
  bool wrt_world;
  EEFPoseGoals(int n) : n_goals(n){
    poses.resize(n_goals);
    timesteps.resize(n_goals);
  }
};

struct JointPosGoals{
    int no_waypoints;
    std::vector<double> q;
    std::vector<int> timesteps;
};

struct Grasp {
    std::string obj_name;
    OpenRAVE::Transform pose;
    bool graspable;
};


class FRASIEROpenRAVE{
public:
  FRASIEROpenRAVE(ros::NodeHandle n, bool run_viewer=false, bool real_robot=false);
  ~FRASIEROpenRAVE();


  // General
  bool loadHSR();
  bool loadCustomEnv(std::string& world_path);
  void viewerThread();
  void updateJointStatesThread();
  void updateCollisionEnv();
  void updatePlanningEnv();
  void getActiveJointIndex(std::vector<int>& q_index);
  void getWholeBodyJointIndex(std::vector<int>& q_index);
  OpenRAVE::Transform getRobotTransform();
  geometry_msgs::Pose2D getRobotPose();
  void startThreads();


  // Motion planning
  void planRRT(std::vector<double>& q);
  Json::Value createJsonValueTraj(EEFPoseGoals& eef_goals);
  Json::Value createJsonValueTraj(JointPosGoals& joint_goals);
  Json::Value createJsonValueIK(OpenRAVE::Transform& eef_pose, bool check_coll=true);
  trajectory_msgs::JointTrajectory computeTrajectory(EEFPoseGoals& eef_goals, bool plot=false);
  trajectory_msgs::JointTrajectory computeOnTheFlyTraj(EEFPoseGoals& eef_goals, bool plot=false);
  trajectory_msgs::JointTrajectory computeTrajectory(JointPosGoals& joint_goals, bool plot=false);
  void computeIK(OpenRAVE::Transform& eef_pose, Eigen::VectorXd& q_sol, bool check_coll=true);
  void grabObject(std::string& obj_name);
  void releaseObject(std::string& obj_name);
  void removeTableObjects();
  void smoothTrajectory(trajectory_msgs::JointTrajectory& traj,
                        trajectory_msgs::JointTrajectory& traj_smoothed);
  void checkCollisions(trajectory_msgs::JointTrajectory& traj);

  //  Utilities
  void playTrajectory(trajectory_msgs::JointTrajectory& traj);
  void drawTransform(OpenRAVE::Transform& T);
  trajectory_msgs::JointTrajectory eigenMatrixToTraj(Eigen::MatrixXd& traj);
  geometry_msgs::Pose orTransformToROSPose(OpenRAVE::Transform& transform);


  // Grasping
  std::vector<OpenRAVE::Transform> generatePlacePoses();
  Grasp generateGraspPose(int table_pose);
  Grasp generateGraspPose(int table_pose, std::string& obj_name);
  OpenRAVE::Transform generatePlacePose(std::string& obj_name);

  // ROS
  void jointSensorCb(const sensor_msgs::JointState::ConstPtr &msg);
  void baseStateCb(const geometry_msgs::Pose2D::ConstPtr &msg);
  sensor_msgs::JointState getWholeBodyState();

  // Perception
  void addBoxCollObj(OpenRAVE::Vector& size, OpenRAVE::Transform& pose, std::string& obj_name);
  void addCylinderCollObj(OpenRAVE::Vector& size, OpenRAVE::Transform& pose, std::string& obj_name);
  void removeCollisionObj(std::string& obj_name);
  void addMeshCollObj(pcl_msgs::PolygonMesh& mesh, std::string& obj_name);
  void getObjectPose(OpenRAVE::Transform& pose, std::string& obj_name);

private:
  ros::NodeHandle nh_;
  ros::Subscriber joint_state_sub_, base_state_sub_;
  geometry_msgs::Pose2D base_;
  sensor_msgs::JointState joints_;

  std::string joint_state_topic_, base_state_topic_, base_link_;
  std::string robot_name_, manip_name_, planner_name_;
  std::string package_path_, config_path_, worlds_path_;
  std::vector<std::string> joint_names_, base_names_, whole_body_joint_names_;

  bool joint_state_flag_, run_viewer_flag_, run_joint_updater_flag_;
  bool plan_plotter_;

  OpenRAVE::EnvironmentBasePtr env_, planning_env_;
  OpenRAVE::ViewerBasePtr viewer_;
  OpenRAVE::RobotBasePtr hsr_;
  OpenRAVE::RobotBase::ManipulatorPtr manip_;
  OpenRAVE::PlannerBasePtr planner_;
  OpenRAVE::ControllerBasePtr controller_;

  boost::thread viewer_thread_;
  boost::thread joint_state_thread_;
  boost::mutex joint_state_mutex_, base_state_mutex_;

  OpenRAVE::Vector FRONT_EEF_ROT, LEFT_EEF_ROT, RIGHT_EEF_ROT, BACK_EEF_ROT, FRONT_TOP_EEF_ROT;
  double COLLISION_PENALTY;
  int COLLISION_COEFF;
  double MAX_FINGER_APERTURE;
  int FRONT_TABLE = 0;
  int LEFT_TABLE = 1;
  int RIGHT_TABLE = 2;


};

#endif
