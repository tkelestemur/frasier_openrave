#include <frasier_openrave/frasier_openrave.h>
#include <frasier_openrave/frasier_controller.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "frasier_openrave_test");
    ros::NodeHandle nh; //

    FRASIEROpenRAVE rave(nh);
    FRASIERController controller(nh);
    rave.LoadHSR();

    rave.startThreads();
    ros::Duration(2.0).sleep();
    // controller.moveToStartState();
    // ros::Duration(2.0).sleep();

    // Test trajectory optimization
    EEFPoseGoals eef_goals;
    eef_goals.n_goals = 2;
    eef_goals.poses.resize(2);
    eef_goals.timesteps.resize(2);
    eef_goals.poses[0] = Eigen::Affine3d::Identity();
    eef_goals.poses[0].translation() = Eigen::Vector3d(1.0, 0.0, 1.1);
    eef_goals.timesteps[0] = 5;
    eef_goals.poses[1] = Eigen::Affine3d::Identity();
    eef_goals.poses[1].translation() = Eigen::Vector3d(2.0, 0.0, 1.4);
    eef_goals.timesteps[1] = 9;
    Eigen::MatrixXd traj;
    rave.computeTrajectory(traj, eef_goals);
    std::cout << "TRAJECTORY:  " << std::endl << traj << std::endl;
    bool play = false;
    std::string p;
    while (!play) {
      std::cin >> p;
      if (p == "p") {
        play = true;
      }
      ros::Duration(1.0).sleep();
    }
    rave.playTrajectory(traj);
    // controller.sendWholeBodyTraj(traj);

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

    // rave.startROSSpinner();



    return 0;
}
