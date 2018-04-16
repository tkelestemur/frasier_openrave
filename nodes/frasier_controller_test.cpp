#include <frasier_openrave/frasier_openrave.h>
#include <frasier_openrave/frasier_controller.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "frasier_controller_test");
    ros::NodeHandle nh; //

    FRASIEROpenRAVE rave(nh, false);
    FRASIERController controller(nh);
    rave.LoadHSR();

    rave.startThreads();

    ros::Duration(1.0).sleep();
    controller.moveToStartState(MOVE_STATE::HOME);

    EEFPoseGoals eef_goals;
    eef_goals.n_goals = 2;
    eef_goals.poses.resize(2);
    eef_goals.timesteps.resize(2);

    Eigen::Vector3d p(0.8, 0.0, 0.3);
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


    Eigen::MatrixXd traj;
    rave.computeTrajectory(traj, eef_goals);

    sensor_msgs::JointState start_state = rave.getWholeBodyState();
    controller.setStartState(start_state);
    controller.executeWholeBodyTraj(traj);


    rave.startROSSpinner();
    return 0;
}
