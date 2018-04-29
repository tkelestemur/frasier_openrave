#include <frasier_openrave/frasier_openrave.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "frasier_planner_test");
    ros::NodeHandle nh; //

    FRASIEROpenRAVE rave(nh);

    rave.loadHSR();

    rave.startThreads();
    ros::Duration(1.0).sleep();

    // Test trajectory optimization
    EEFPoseGoals eef_goals;
    eef_goals.n_goals = 2;
    eef_goals.poses.resize(2);
    eef_goals.timesteps.resize(2);

    Eigen::Vector3d p(0.4, 0.0, 1.0);
    Eigen::Quaterniond q(0.0, 0.707, 0.0, 0.707);
    Eigen::Affine3d pose_1;
    pose_1.translation() = p;
    pose_1.linear() = q.toRotationMatrix();
    eef_goals.poses[0] = pose_1;
    eef_goals.timesteps[0] = 5;

    Eigen::Vector3d p_2(1.2, -0.5, 1.1);
    Eigen::Quaterniond q_2(0.0, 0.707, 0.0, 0.707);
    Eigen::Affine3d pose_2;
    pose_2.translation() = p_2;
    pose_2.linear() = q_2.toRotationMatrix();
    eef_goals.poses[1] = pose_2;
    eef_goals.timesteps[1] = 9;

    trajectory_msgs::JointTrajectory traj = rave.computeTrajectory(eef_goals);
//    std::cout << "TRAJECTORY:  " << std::endl << traj << std::endl;

    // rave.playTrajectory(traj);

    // Test IK solver
    // Eigen::Affine3d eef_pose = Eigen::Affine3d::Identity();
    // eef_pose.translation() = Eigen::Vector3d(1.0, 0.0, 1.1);
    // Eigen::VectorXd q_sol;
    // rave.computeIK(eef_pose, q_sol);
    // std::cout << "IK SOLUTION:  " << std::endl << q_sol << std::endl;

    // Test RRT planner
    // rave.initRRTPlanner();
    // std::vector<double> q;
    // q = {1.0, 0.5, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0};
    // rave.planToConf(q);





    return 0;
}
