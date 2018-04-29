#include <frasier_openrave/frasier_openrave.h>
#include <frasier_openrave/frasier_controller.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "frasier_controller_test");
    ros::NodeHandle nh; //
    ros::AsyncSpinner spinner(4);
    FRASIEROpenRAVE rave(nh, true);
    FRASIERController controller(nh);

    spinner.start();
    rave.loadHSR();
    rave.startThreads();
    ros::Duration(1.0).sleep();


    controller.moveToKnownState(MOVE_STATE::HOME);

    EEFPoseGoals eef_grasp_goal(1);
    eef_grasp_goal.wrt_world = false;
    eef_grasp_goal.no_waypoints = 5;
    eef_grasp_goal.timesteps[0] = 4;
    eef_grasp_goal.poses[0] = OpenRAVE::Transform(OpenRAVE::Vector(0.0, 0.707, 0.0, 0.707), OpenRAVE::Vector(1.0, 0.0, 1.0));

    trajectory_msgs::JointTrajectory traj = rave.computeTrajectory(eef_grasp_goal);
//    controller.executeWholeBodyTraj(traj);
    trajectory_msgs::JointTrajectory smoothed_traj;
    controller.smoothTrajectory(traj, smoothed_traj);



    ros::waitForShutdown();
    return 0;
}
