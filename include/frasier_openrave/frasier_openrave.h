#ifndef FRASIER_OPENRAVE_H_
#define FRASIER_OPENRAVE_H_

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>
#include <trajectory_msgs/JointTrajectory.h>
//#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/manipulators.hpp>
#include <ecl/exceptions.hpp>
#include <ecl/geometry.hpp>

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
#include <boost/date_time.hpp>
#include <boost/bind.hpp>
#include <json/json.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

// PCL
//#include <pcl/PolygonMesh.h>

using json = nlohmann::json;

enum POSE_TYPE {
    COST,
    CONSTRAINT
};


struct EEFPoseGoals {
    int n_goals;
    int no_waypoints;
    std::vector<OpenRAVE::Transform> poses;
    std::vector<int> timesteps;
    std::vector<POSE_TYPE> pose_types;
    double aperture;
    bool wrt_world;

    EEFPoseGoals(int n) : n_goals(n) {
        poses.resize(n_goals);
        timesteps.resize(n_goals);
//    pose_types.resize(n_goals);
        for (int i = 0; i < n_goals; i++) {
            pose_types.push_back(CONSTRAINT);
        }
    }
};

struct JointPosGoals {
    int no_waypoints;
    std::vector<double> q;
    std::vector<int> timesteps;
};

struct Grasp {
    std::string obj_name;
    OpenRAVE::Transform pose;
    bool graspable;
};


class FRASIEROpenRAVE {
public:
    FRASIEROpenRAVE(ros::NodeHandle n, bool run_viewer = false, bool real_robot = false);

    ~FRASIEROpenRAVE();


    // General
    bool loadHSR();

    bool loadCustomEnv(std::string &world_path);

    void viewerThread();

    void updateJointStatesThread();

    void getActiveJointIndex(std::vector<int> &q_index);

    void ResetEnv();

    // void getWholeBodyJointIndex(std::vector<int> &q_index);

    OpenRAVE::Transform getRobotTransform();
    OpenRAVE::Transform getEEFTransform();

    geometry_msgs::Pose2D getRobotPose();


    // Motion planning
    Json::Value createJsonValueTraj(EEFPoseGoals &eef_goals);

    Json::Value createJsonValueTraj(JointPosGoals &joint_goals);

    Json::Value createJsonValueIK(OpenRAVE::Transform &eef_pose, bool check_coll = true);

    trajectory_msgs::JointTrajectory computeTrajectory(EEFPoseGoals &eef_goals, bool plot = false);

    trajectory_msgs::JointTrajectory computeOnTheFlyTraj(EEFPoseGoals &eef_goals, bool plot = false);

    trajectory_msgs::JointTrajectory computeTrajectory(JointPosGoals &joint_goals, bool plot = false);

    void computeIK(OpenRAVE::Transform &eef_pose, Eigen::VectorXd &q_sol, bool check_coll = true);

    void grabObject(std::string &obj_name);

    void releaseObject(std::string &obj_name);

    void smoothTrajectory(trajectory_msgs::JointTrajectory &traj,
                          trajectory_msgs::JointTrajectory &traj_smoothed);

    void checkCollisions(trajectory_msgs::JointTrajectory &traj);

    // Kinetmatics
    void getJacobian();

    void playTrajectory(trajectory_msgs::JointTrajectory &traj);

    void moveToHomeState();

    void setEEFValue(double &v);

    //  Utilities
    void drawTransform(OpenRAVE::Transform &T, bool transparent = false);

    void drawArrow(OpenRAVE::Vector &origin, OpenRAVE::Vector &vector, bool transparent = false);

    void drawPoints(std::vector<OpenRAVE::Vector> &points);

    void drawPoint(OpenRAVE::Vector &point);

    void drawMesh(OpenRAVE::TriMesh& mesh);

    trajectory_msgs::JointTrajectory eigenMatrixToTraj(Eigen::MatrixXd &traj);

    geometry_msgs::Pose orTransformToROSPose(OpenRAVE::Transform &transform);
    OpenRAVE::Transform ROSTransformToORTransform(geometry_msgs::Transform &transform);

    // Grasping
    void sampleGraspPoses(std::string &obj_name);

    void generateEEFCurve();

    Grasp generateGraspPose();

    Grasp generateGraspPose(const std::string &obj_name);

//    OpenRAVE::Transform generatePlacePose(std::string &obj_name);

//    std::vector<OpenRAVE::Transform> generatePlacePoses();

    // ROS
    void jointSensorCb(const sensor_msgs::JointState::ConstPtr &msg);

    void baseStateCb(const geometry_msgs::Pose2D::ConstPtr &msg);

    sensor_msgs::JointState getWholeBodyState();

    // Perception
    void addBoxCollObj(OpenRAVE::Vector &size, OpenRAVE::Transform &pose,
                       std::string &obj_name, bool collision = true);

    void addCylinderCollObj(OpenRAVE::Vector &size, OpenRAVE::Transform &pose,
                            std::string &obj_name, bool collision = true);

    void removeCollisionObj(std::string &obj_name);

    void addMeshCollObj(pcl_msgs::PolygonMesh &mesh, std::string &obj_name);

    void getObjectPose(OpenRAVE::Transform &pose, std::string &obj_name);

    void removeTableObjects();

private:
    ros::NodeHandle nh_;
    ros::Subscriber joint_state_sub_, base_state_sub_;
    geometry_msgs::Pose2D base_;
    sensor_msgs::JointState joints_;

    std::string joint_state_topic_, base_state_topic_, base_link_, eef_link_;
    std::string robot_name_, manip_name_, planner_name_;
    std::string package_path_, config_path_, worlds_path_;
    std::vector<std::string> joint_names_, base_names_, whole_body_joint_names_;

    bool joint_state_flag_, base_state_flag_, run_viewer_flag_, run_joint_updater_flag_;
    bool plan_plotter_;
    std::vector<int> whole_body_joint_index_, eef_joint_index_;

    OpenRAVE::EnvironmentBasePtr env_;
    OpenRAVE::ViewerBasePtr viewer_;
    OpenRAVE::RobotBasePtr hsr_;
    OpenRAVE::RobotBase::ManipulatorPtr manip_;
    OpenRAVE::PlannerBasePtr planner_;
    OpenRAVE::ControllerBasePtr controller_;
    std::vector<OpenRAVE::GraphHandlePtr> graph_handles_;

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
