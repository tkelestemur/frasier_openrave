#include <frasier_openrave/frasier_openrave.h>
#include <frasier_openrave/frasier_controller.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "frasier_openrave_test");
    ros::NodeHandle nh; //

    FRASIEROpenRAVE rave(nh);
    FRASIERController controller(nh);
    rave.LoadHSR();

    rave.startThreads();
    ros::Duration(1.0).sleep();
    // controller.moveToStartState();
    // ros::Duration(2.0).sleep();

    // Test trajectory optimization
    Eigen::MatrixXd traj;
    rave.computeTrajectory(traj);
    std::cout << "TRAJECTORY:  " << std::endl << traj << std::endl;
    ros::Duration(2.0).sleep();
    rave.playTrajectory(traj);
    // controller.sendWholeBodyTraj(traj);

    // Test IK solver
    // Eigen::VectorXd q_sol;
    // frasier_or.computeIK(q_sol);
    // std::cout << "IK SOLUTION:  " << std::endl << q_sol << std::endl;

    // Test RRT planner
    // rave.initRRTPlanner();
    // std::vector<double> q;
    // q = {1.0, 0.5, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0};
    // rave.planToConf(q);

    rave.startROSSpinner();

    //

    return 0;
}
