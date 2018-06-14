//  filter_traj_srv_ = nh_.serviceClient<tmc_manipulation_msgs::FilterJointTrajectory>("/filter_hsrb_trajectory");

//bool FRASIERController::filterTrajectory(trajectory_msgs::JointTrajectory& traj,
//                                         trajectory_msgs::JointTrajectory& traj_filtered){
//
//
//  tmc_manipulation_msgs::FilterJointTrajectoryRequest filter_req;
//  tmc_manipulation_msgs::FilterJointTrajectoryResponse filter_res;
//
//  sensor_msgs::JointState start_state;
//  start_state.name.resize(traj.joint_names.size());
//
//  filter_req.trajectory = traj;
//  filter_req.allowed_time = ros::Duration(30.0);
//  filter_req.start_state.joint_state = start_state_;
//
//  std::cout << "calling the filter service..." << std::endl;
//
//  bool result = filter_traj_srv_.call(filter_req, filter_res);
//
//  if (filter_res.error_code.val != tmc_manipulation_msgs::ArmNavigationErrorCodes::SUCCESS) {
//      std::cout << "cannot filter the trajectory, error code: " << filter_res.error_code.val << std::endl;
//      return false;
//  }else{
//    std::cout << "trajecory filtering is successfull!" << std::endl;
//
//    traj_filtered = filter_res.trajectory;
//
//    return true;
//  }
//
//
//}

