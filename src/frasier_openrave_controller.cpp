    #include <frasier_openrave/frasier_openrave_controller.h>


FRASIERController::FRASIERController(ros::NodeHandle n) : nh_(n),
                                                          arm_cli_("/hsrb/arm_trajectory_controller/follow_joint_trajectory", true),
                                                          base_cli_("/hsrb/omni_base_controller/follow_joint_trajectory", true),
                                                          head_cli_("/hsrb/head_trajectory_controller/follow_joint_trajectory", true),
                                                          whole_body_cli_("/hsrb/impedance_control/follow_joint_trajectory", true),
                                                          gripper_pos_cli_("/hsrb/gripper_controller/follow_joint_trajectory", true),
                                                          gripper_cli_("/hsrb/gripper_controller/grasp", true){

    bool arm_cli_running = arm_cli_.waitForServer(ros::Duration(3.0));
    bool base_cli_running = base_cli_.waitForServer(ros::Duration(3.0));
    bool head_cli_running = head_cli_.waitForServer(ros::Duration(3.0));
    bool gripper_cli_running = gripper_cli_.waitForServer(ros::Duration(3.0));
    bool wb_cli_running = whole_body_cli_.waitForServer(ros::Duration(3.0));

    if(arm_cli_running && base_cli_running && head_cli_running &&
       gripper_cli_running && wb_cli_running){
        std::cout << "CONTROL: controller is initialized!" << std::endl;
    }else{
        std::cout << "CONTROL: controller is NOT initialized! RESTART THE ROBOT!" << std::endl;
    }


}


void FRASIERController::extractArmBaseTraj(trajectory_msgs::JointTrajectory whole_body_traj,
                                           trajectory_msgs::JointTrajectory& base_traj,
                                           trajectory_msgs::JointTrajectory& arm_traj) {

    int n_waypoints = whole_body_traj.points.size();
    base_traj.points.resize(n_waypoints);
    arm_traj.points.resize(n_waypoints);

    base_traj.joint_names.push_back("odom_x");
    base_traj.joint_names.push_back("odom_y");
    base_traj.joint_names.push_back("odom_t");

    arm_traj.joint_names.push_back("arm_lift_joint");
    arm_traj.joint_names.push_back("arm_flex_joint");
    arm_traj.joint_names.push_back("arm_roll_joint");
    arm_traj.joint_names.push_back("wrist_flex_joint");
    arm_traj.joint_names.push_back("wrist_roll_joint");

    for (int i = 0; i < n_waypoints; i++) {
        for (int j = 0; j < 3; j++){
            base_traj.points[i].positions.push_back(whole_body_traj.points[i].positions[j]);
            base_traj.points[i].velocities.push_back(whole_body_traj.points[i].velocities[j]);
            base_traj.points[i].accelerations.push_back(whole_body_traj.points[i].accelerations[j]);
            base_traj.points[i].time_from_start = whole_body_traj.points[i].time_from_start;
        }
        for (int k = 3; k < 8 ; k++) {
            arm_traj.points[i].positions.push_back(whole_body_traj.points[i].positions[k]);
            arm_traj.points[i].velocities.push_back(whole_body_traj.points[i].velocities[k]);
            arm_traj.points[i].accelerations.push_back(whole_body_traj.points[i].accelerations[k]);
            arm_traj.points[i].time_from_start = whole_body_traj.points[i].time_from_start;
        }
    }


}


void FRASIERController::executeArmBaseTraj(trajectory_msgs::JointTrajectory& traj) {
    std::cout << "CONTROL: executing whole body trajectory..." << std::endl;

    control_msgs::FollowJointTrajectoryGoal base_goal, arm_goal;
    trajectory_msgs::JointTrajectory base_traj, arm_traj;
    extractArmBaseTraj(traj, base_traj, arm_traj);

    arm_goal.trajectory = arm_traj;
    base_goal.trajectory = base_traj;

    arm_cli_.sendGoal(arm_goal);
    base_cli_.sendGoal(base_goal);
    arm_cli_.waitForResult(ros::Duration(CONTROLLER_TIMEOUT));
    base_cli_.waitForResult(ros::Duration(CONTROLLER_TIMEOUT));

}

void FRASIERController::executeWholeBodyTraj(trajectory_msgs::JointTrajectory& traj, bool execute_gripper) {

    std::cout << "CONTROL: executing whole body impedance trajectory..." << std::endl;
    control_msgs::FollowJointTrajectoryGoal whole_body_goal, gripper_goal;

    whole_body_goal.trajectory = traj;

    gripper_goal.trajectory.joint_names.push_back("hand_motor_joint");
    gripper_goal.trajectory.points.resize(traj.points.size());

    for (int i = 0; i < traj.points.size(); i++) {
        gripper_goal.trajectory.points[i].positions.push_back(traj.points[i].positions[8]);
        gripper_goal.trajectory.points[i].velocities.push_back(traj.points[i].velocities[8]);
        gripper_goal.trajectory.points[i].time_from_start = traj.points[i].time_from_start;
    }

    if(execute_gripper){
        gripper_pos_cli_.sendGoal(gripper_goal);
        whole_body_cli_.sendGoal(whole_body_goal);
        whole_body_cli_.waitForResult(ros::Duration(CONTROLLER_TIMEOUT));
        gripper_pos_cli_.waitForResult(ros::Duration(CONTROLLER_TIMEOUT));
    }else{
        whole_body_cli_.sendGoal(whole_body_goal);
        whole_body_cli_.waitForResult(ros::Duration(CONTROLLER_TIMEOUT));
    }

}

//void FRASIERController::executeWholeBodyTraj2(trajectory_msgs::JointTrajectory &traj) {
//    std::cout << "CONTROL: executing whole body impedance trajectory..." << std::endl;
//    control_msgs::FollowJointTrajectoryGoal whole_body_goal;
//
//    ros::Duration dt(0.8);
//    traj.points[0].time_from_start = dt;
//
//    for (int i = 1; i < traj.points.size(); ++i) {
//        traj.points[i].time_from_start = traj.points[i-1].time_from_start + dt;
//    }
//
//    whole_body_goal.trajectory = traj;
//
//    whole_body_cli_.sendGoal(whole_body_goal);
//    whole_body_cli_.waitForResult(ros::Duration(CONTROLLER_TIMEOUT));
//
//}


void FRASIERController::moveToKnownState(MOVE_STATE state){

    std::cout << "CONTROL: moving body to known state... "  << std::endl;

    /////// BASE TRAJECTORY ///////
    control_msgs::FollowJointTrajectoryGoal base_goal;
    trajectory_msgs::JointTrajectory base_traj;

    base_traj.joint_names.push_back("odom_x");
    base_traj.joint_names.push_back("odom_y");
    base_traj.joint_names.push_back("odom_t");

    base_traj.points.resize(1);

    base_traj.points[0].positions.resize(3);
    if (state == MOVE_STATE::HOME){
        base_traj.points[0].positions[0] = 0.0;
        base_traj.points[0].positions[1] = 0.0;
        base_traj.points[0].positions[2] = 0.0;
    }
    else if (state == MOVE_STATE::PICK){
        base_traj.points[0].positions[0] = 0.0;
        base_traj.points[0].positions[1] = 0.0;
        base_traj.points[0].positions[2] = 0.0;
    }
    else if (state == MOVE_STATE::SHELF){
        base_traj.points[0].positions[0] = 0.0;
        base_traj.points[0].positions[1] = 0.0;
        base_traj.points[0].positions[2] = 0.0;
    }


    base_traj.points[0].time_from_start = ros::Duration(5.0);

    base_goal.trajectory = base_traj;

    /////// ARM TRAJECTORY ///////
    control_msgs::FollowJointTrajectoryGoal arm_goal;
    trajectory_msgs::JointTrajectory arm_traj;

    arm_traj.joint_names.push_back("arm_lift_joint");
    arm_traj.joint_names.push_back("arm_flex_joint");
    arm_traj.joint_names.push_back("arm_roll_joint");
    arm_traj.joint_names.push_back("wrist_flex_joint");
    arm_traj.joint_names.push_back("wrist_roll_joint");

    arm_traj.points.resize(1);

    arm_traj.points[0].positions.resize(5);
    if (state == MOVE_STATE::HOME) {

        arm_traj.points[0].positions[0] = 0.0;
        arm_traj.points[0].positions[1] = 0.0;
        arm_traj.points[0].positions[2] = -1.57;
        arm_traj.points[0].positions[3] = -1.57;
        arm_traj.points[0].positions[4] = 0.0;
    }
    else if(state == MOVE_STATE::PICK){
        arm_traj.points[0].positions[0] = 0.69;
        arm_traj.points[0].positions[1] = -2.60;
        arm_traj.points[0].positions[2] = 0.0;
        arm_traj.points[0].positions[3] = -0.54;
        arm_traj.points[0].positions[4] = 0.0;
    }
    else if (state == MOVE_STATE::SHELF){
        arm_traj.points[0].positions[0] = 0.69;
        arm_traj.points[0].positions[1] = -2.60;
        arm_traj.points[0].positions[2] = 0.0;
        arm_traj.points[0].positions[3] = -0.54;
        arm_traj.points[0].positions[4] = 0.0;
    }

    arm_traj.points[0].time_from_start = ros::Duration(5.0);

    arm_goal.trajectory = arm_traj;

    base_cli_.sendGoal(base_goal);
    arm_cli_.sendGoal(arm_goal);
    base_cli_.waitForResult(ros::Duration(CONTROLLER_TIMEOUT));
    arm_cli_.waitForResult(ros::Duration(CONTROLLER_TIMEOUT));


}

void FRASIERController::moveBase(geometry_msgs::Pose2D& pose) {
    std::cout << "CONTROL: moving base... "  << std::endl;

    /////// BASE TRAJECTORY ///////
    control_msgs::FollowJointTrajectoryGoal base_goal;
    trajectory_msgs::JointTrajectory base_traj;

    base_traj.joint_names.push_back("odom_x");
    base_traj.joint_names.push_back("odom_y");
    base_traj.joint_names.push_back("odom_t");

    base_traj.points.resize(1);

    base_traj.points[0].positions.resize(3);

    base_traj.points[0].positions[0] = pose.x;
    base_traj.points[0].positions[1] = pose.y;
    base_traj.points[0].positions[2] = pose.theta;


    base_traj.points[0].time_from_start = ros::Duration(5.0);

    base_goal.trajectory = base_traj;
    base_cli_.sendGoal(base_goal);
    base_cli_.waitForResult(ros::Duration(CONTROLLER_TIMEOUT));


}

void FRASIERController::moveArmToKnownState(ARM_STATE state) {
    std::cout << "CONTROL: moving arm to known state... "  << std::endl;

    control_msgs::FollowJointTrajectoryGoal arm_goal;
    trajectory_msgs::JointTrajectory arm_traj;

    arm_traj.joint_names.push_back("arm_lift_joint");
    arm_traj.joint_names.push_back("arm_flex_joint");
    arm_traj.joint_names.push_back("arm_roll_joint");
    arm_traj.joint_names.push_back("wrist_flex_joint");
    arm_traj.joint_names.push_back("wrist_roll_joint");

    arm_traj.points.resize(1);
    arm_traj.points[0].positions.resize(5);
    if(state == ARM_STATE::GRASP_CONF){
        arm_traj.points[0].positions[0] = 0.69;
        arm_traj.points[0].positions[1] = -2.60;
        arm_traj.points[0].positions[2] = 0.0;
        arm_traj.points[0].positions[3] = -0.54;
        arm_traj.points[0].positions[4] = 0.0;
    }

    else if (state == ARM_STATE::GIVE_CONF) {
        arm_traj.points[0].positions[0] = 0.14;
        arm_traj.points[0].positions[1] = -0.47;
        arm_traj.points[0].positions[2] = 0.0;
        arm_traj.points[0].positions[3] = -1.10;
        arm_traj.points[0].positions[4] = 0.0;
    }

    else if (state == ARM_STATE::GO_CONF) {
        arm_traj.points[0].positions[0] = 0.0;
        arm_traj.points[0].positions[1] = 0.0;
        arm_traj.points[0].positions[2] = 0.0;
        arm_traj.points[0].positions[3] = -M_PI/2;
        arm_traj.points[0].positions[4] = 0.0;
    }


    arm_traj.points[0].time_from_start = ros::Duration(3.0);

    arm_goal.trajectory = arm_traj;

    arm_cli_.sendGoal(arm_goal);
    arm_cli_.waitForResult(ros::Duration(CONTROLLER_TIMEOUT));

}

void FRASIERController::moveHeadToKnownState(HEAD_STATE state) {

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("head_pan_joint");
    goal.trajectory.joint_names.push_back("head_tilt_joint");

    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions.resize(2);
    if (state == HEAD_STATE::LOOK_TABLE_LEFT){
        std::cout << "CONTROL: moving head towards left table... "  << std::endl;
        goal.trajectory.points[0].positions[0] = M_PI/2;
        goal.trajectory.points[0].positions[1] = -0.60;
    }
    else if(state == HEAD_STATE::LOOK_TABLE_RIGHT){
        std::cout << "CONTROL: moving head towards right table... "  << std::endl;
        goal.trajectory.points[0].positions[0] = -M_PI/2;
        goal.trajectory.points[0].positions[1] = -0.60;
    }
    else if(state == HEAD_STATE::LOOK_TABLE_FRONT){
        std::cout << "CONTROL: moving head towards front table... "  << std::endl;
        goal.trajectory.points[0].positions[0] = 0.0;
        goal.trajectory.points[0].positions[1] = -0.50;
    }
    else if(state == HEAD_STATE::LOOK_SHELF_FRONT){
        std::cout << "CONTROL: moving head towards front table... "  << std::endl;
        goal.trajectory.points[0].positions[0] = 0.0;
        goal.trajectory.points[0].positions[1] = 0.0;
    }
    else if(state == HEAD_STATE::LOOK_FLOOR){
        std::cout << "CONTROL: moving head towards floor.. "  << std::endl;
        goal.trajectory.points[0].positions[0] = 0.0;
        goal.trajectory.points[0].positions[1] = -0.80;
    }

    goal.trajectory.points[0].time_from_start = ros::Duration(2.0);
    head_cli_.sendGoalAndWait(goal, ros::Duration(CONTROLLER_TIMEOUT));
}

void FRASIERController::graspOrRelease(GRIPPER_STATE state) {
    tmc_control_msgs::GripperApplyEffortGoal goal;

    if (state == GRIPPER_STATE::GRASP){
        goal.effort = -0.3;
    }
    else if(state == GRIPPER_STATE::RELEASE){
        goal.effort = 0.1;
    }


    gripper_cli_.sendGoalAndWait(goal, ros::Duration(2.0));
}

void FRASIERController::executeGraspTraj(trajectory_msgs::JointTrajectory &traj) {
    control_msgs::FollowJointTrajectoryGoal goal;

    goal.trajectory.joint_names.push_back("hand_motor_joint");

    goal.trajectory.points.resize(traj.points.size());

    for (int i = 0; i < traj.points.size(); i++) {
        goal.trajectory.points[i].positions.push_back(traj.points[i].positions[8]);
//    goal.trajectory.points[i].velocities.push_back(traj.points[i].velocities[8]);
        goal.trajectory.points[i].effort.push_back(-0.3);
        goal.trajectory.points[i].time_from_start = traj.points[i].time_from_start;
    }


    gripper_pos_cli_.sendGoal(goal);
    gripper_pos_cli_.waitForResult(ros::Duration(CONTROLLER_TIMEOUT));


}

void FRASIERController::gripperThread() {
    tf::TransformListener listener;
    std::string target_frame = "hand_palm_link"; //
    std::string fixed_frame = "odom";
    listener.waitForTransform(fixed_frame, target_frame, ros::Time(0), ros::Duration(2.0));
    Eigen::Vector3d err;
    ros::Rate rate(10);
    while(ros::ok()){
        tf::StampedTransform transform;
        listener.lookupTransform(fixed_frame, target_frame, ros::Time(0), transform);
        double x_err = eef_target_pose_.position.x - transform.getOrigin().x();
        double y_err = eef_target_pose_.position.y - transform.getOrigin().y();
        double z_err = eef_target_pose_.position.z - transform.getOrigin().z();
        err[0] = x_err;
        err[1] = y_err;
        err[2] = z_err;
//    std::cout << "eef x y z: " << transform.getOrigin().x() << " "
//                               << transform.getOrigin().y() << " "
//                               << transform.getOrigin().z() << std::endl;

        double err_total = err.transpose() * err;
//    std::cout << "err: " << err << std::endl;
//    std::cout << "err^T: " << err.transpose() << std::endl;
//    std::cout << "total error: " << err_total << std::endl;
        if (err_total < 1.0e-3){
            tmc_control_msgs::GripperApplyEffortGoal goal;
            goal.effort = -0.5;
            gripper_cli_.sendGoal(goal);
        }
        rate.sleep();
    }
}

void FRASIERController::runGripperThread(geometry_msgs::Pose& pose) {
    eef_target_pose_ = pose;
    gripper_thread_ = boost::thread(&FRASIERController::gripperThread, this);
}
