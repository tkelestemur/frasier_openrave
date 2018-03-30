#include <frasier_openrave/frasier_openrave.h>


FRASIEROpenRAVE::FRASIEROpenRAVE(ros::NodeHandle n) : nh_(n){

  // ROS
  joint_state_topic_ = "/hsrb/robot_state/joint_states";
  joint_state_sub_ = nh_.subscribe(joint_state_topic_, 1, &FRASIEROpenRAVE::jointSensorCb, this);
  robot_name_ = "hsrb";

  // OpenRAVE
  OpenRAVE::RaveInitialize(true);
  env_ = OpenRAVE::RaveCreateEnvironment();
  assert(env_);
  env_->SetDebugLevel(OpenRAVE::Level_Error);

  joint_state_flag_ = false;
  run_viewer_flag_ = true;
  run_joint_updater_flag_ = false;

  joint_names_ = {"arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint",
                  "head_pan_joint", "head_tilt_joint", "hand_motor_joint", "hand_l_spring_proximal_joint",
                  "hand_l_distal_joint", "hand_r_spring_proximal_joint", "hand_r_distal_joint"};


}

FRASIEROpenRAVE::~FRASIEROpenRAVE(){
  env_->Destroy();
  viewer_->quitmainloop();
  viewer_thread_.join();
  joint_state_thread_.join();
}


void FRASIEROpenRAVE::setViewer(){

  std::string viewer_name = OpenRAVE::RaveGetDefaultViewerType(); // qtcoin

  viewer_ = OpenRAVE::RaveCreateViewer(env_, viewer_name);
  BOOST_ASSERT(!!viewer_);

  // attach it to the environment:
  env_->Add(viewer_);
  // viewer_.SetBkgndColor() TODO: Change background colour

  // finally call the viewer's infinite loop (this is why a separate thread is needed)
  viewer_->main(true); //TODO: Need EnvironmentMutex ???

}


bool FRASIEROpenRAVE::LoadHSR(){
  std::string path = ros::package::getPath("frasier_openrave");
  std::string world_path;
  world_path = path + "/worlds/hsr_whole_body.xml";
  bool success = env_->Load(world_path);

  hsr_ = env_->GetRobot(robot_name_);

  if (success) {
    std::cout << "HSR Initialized!" << std::endl;
  }

  return success;
}


void FRASIEROpenRAVE::getActiveJointIndex(std::vector<int>& q_index){
  for (int i = 0; i < joint_names_.size(); i++) {
    q_index.push_back(hsr_->GetJoint(joint_names_[i])->GetDOFIndex());
  }
}

void FRASIEROpenRAVE::jointSensorCb(const sensor_msgs::JointState::ConstPtr &msg){
  boost::mutex::scoped_lock lock(joint_state_mutex_);
  joints_ = *msg;
  joint_state_flag_ = true;

}

void FRASIEROpenRAVE::updateJointStates(){
  ros::Rate r(50);
  std::vector<int> q_index;
  std::vector<double> q, q_v;
  std::vector<std::string> q_names;
  hsr_->GetActiveDOFValues(q);
  hsr_->GetActiveDOFVelocities(q_v);
  getActiveJointIndex(q_index);


  while (ros::ok()) {

    if (joint_state_flag_) {
      {
        boost::mutex::scoped_lock lock(joint_state_mutex_);

          // Set positions
          q[q_index[0]] = joints_.position[12];
          q[q_index[1]] = joints_.position[13];
          q[q_index[2]] = joints_.position[14];
          q[q_index[3]] = joints_.position[15];
          q[q_index[4]] = joints_.position[16];
          q[q_index[5]] = joints_.position[10];
          q[q_index[6]] = joints_.position[11];
          q[q_index[7]] = joints_.position[17];
          q[q_index[8]] = joints_.position[19];
          q[q_index[9]] = joints_.position[21];
          q[q_index[10]] = joints_.position[23];
          q[q_index[11]] = joints_.position[25];

          // Set velocities
          q_v[q_index[0]] = joints_.velocity[12];
          q_v[q_index[1]] = joints_.velocity[13];
          q_v[q_index[2]] = joints_.velocity[14];
          q_v[q_index[3]] = joints_.velocity[15];
          q_v[q_index[4]] = joints_.velocity[16];
          q_v[q_index[5]] = joints_.velocity[10];
          q_v[q_index[6]] = joints_.velocity[11];
          q_v[q_index[7]] = joints_.velocity[17];
          q_v[q_index[8]] = joints_.velocity[19];
          q_v[q_index[9]] = joints_.velocity[21];
          q_v[q_index[10]] = joints_.velocity[23];
          q_v[q_index[11]] = joints_.velocity[25];
      }


      hsr_->SetActiveDOFValues(q);
      hsr_->SetActiveDOFVelocities(q_v);

    }

    ros::spinOnce();
    r.sleep();
  }

}

void FRASIEROpenRAVE::startThreads(/* arguments */) {
  std::cout << "Starting threads..." << std::endl;

  if (run_joint_updater_flag_) {
    joint_state_thread_ = boost::thread(&FRASIEROpenRAVE::updateJointStates, this);
  }

  if (run_viewer_flag_) {
    viewer_thread_ = boost::thread(&FRASIEROpenRAVE::setViewer, this);
  }

}

void FRASIEROpenRAVE::startROSSpinner(){
    ros::Rate r(100);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    ros::Duration(1.0).sleep();
    ROS_INFO("Shutdown");
}
