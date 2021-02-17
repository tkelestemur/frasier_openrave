#include <frasier_openrave/frasier_openrave.h>


FRASIEROpenRAVE::FRASIEROpenRAVE(ros::NodeHandle n, bool run_viewer, bool real_robot) : nh_(n),
                                                                                        run_viewer_flag_(run_viewer),
                                                                                        run_joint_updater_flag_(
                                                                                                real_robot) {


    robot_name_ = "hsrb";

    package_path_ = ros::package::getPath("frasier_openrave");
    worlds_path_ = package_path_ + "/worlds/";
    config_path_ = package_path_ + "/config/";

    // ROS
    if (real_robot) {
        joint_state_topic_ = "/hsrb/robot_state/joint_states";
        base_state_topic_ = "/hsrb/pose2D";

        joint_state_sub_ = nh_.subscribe(joint_state_topic_, 1, &FRASIEROpenRAVE::jointSensorCb, this);
        base_state_sub_ = nh_.subscribe(base_state_topic_, 1, &FRASIEROpenRAVE::baseStateCb, this);
        while(ros::ok()){
            if (joint_state_sub_.getNumPublishers() == 0 || base_state_sub_.getNumPublishers() == 0) {
                std::cout << "RAVE: Waiting for the joint state & base state topics..." << std::endl;
                ros::Duration(0.1).sleep();
            }
            else{
                std::cout << "RAVE: Connected!" << std::endl;
                break;
            }

        }

    }

    // OpenRAVE
    OpenRAVE::RaveInitialize(true);
    env_ = OpenRAVE::RaveCreateEnvironment();


    assert(env_);
    env_->SetDebugLevel(OpenRAVE::Level_Error);

    joint_state_flag_ = false;
    base_state_flag_ = false;
    plan_plotter_ = false;

    base_link_ = "base_link";
    eef_link_ = "hand_palm_link";

    joint_names_ = {"base_x_joint", "base_y_joint", "base_t_joint", "arm_lift_joint", "arm_flex_joint",
                    "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint", "hand_motor_joint",
                    "hand_l_spring_proximal_joint", "hand_l_distal_joint", "hand_r_spring_proximal_joint",
                    "hand_r_distal_joint", "head_pan_joint", "head_tilt_joint"};

    whole_body_joint_names_ = {"base_x_joint", "base_y_joint", "base_t_joint", "arm_lift_joint", "arm_flex_joint",
                               "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};


    FRONT_EEF_ROT = OpenRAVE::Vector(0.0, 0.707, 0.0, 0.707);
    BACK_EEF_ROT = OpenRAVE::Vector(0.707, 0.0, -0.707, 0.0);
    LEFT_EEF_ROT = OpenRAVE::Vector(0.5, -0.5, -0.5, -0.5);
    RIGHT_EEF_ROT = OpenRAVE::Vector(0.5, 0.5, -0.5, 0.5);
    FRONT_TOP_EEF_ROT = OpenRAVE::Vector(0.0, 1.0, 0.0, 0.0);

    COLLISION_PENALTY = 0.020;
    COLLISION_COEFF = 20;
    MAX_FINGER_APERTURE = 0.120;

    bool load = loadHSR();
    assert(load);

//  env_->SetPhysicsEngine(OpenRAVE::RaveCreatePhysicsEngine(env_, "ode"));
//  env_->GetPhysicsEngine()->SetGravity(OpenRAVE::Vector(0, 0, 0));

    if (real_robot) {
        std::cout << "RAVE: starting joint update thread..." << std::endl;
    joint_state_thread_ = boost::thread(boost::bind(&FRASIEROpenRAVE::updateJointStatesThread, this));
    }

    if (run_viewer) {
        std::cout << "RAVE: starting viewer thread..." << std::endl;
        viewer_thread_ = boost::thread(boost::bind(&FRASIEROpenRAVE::viewerThread, this));
    }

    eef_joint_index_.push_back(hsr_->GetJoint("hand_motor_joint")->GetDOFIndex());
    for (int i = 0; i < whole_body_joint_names_.size(); i++) {
        whole_body_joint_index_.push_back(hsr_->GetJoint(whole_body_joint_names_[i])->GetDOFIndex());
    }


//    std::string octomap_plugin_name = "or_octomap";
//    if(OpenRAVE::RaveLoadPlugin(octomap_plugin_name)){
//        std::cout <<"RAVE: octomap plugin loaded" << std::endl;
//    }
//    OpenRAVE::SensorSystemBasePtr octomap = OpenRAVE::RaveCreateSensorSystem(env_, octomap_plugin_name);
//    std::string enable_output, enable_input, pc_topic, reset_topic;
//    enable_input = "Enable";
//    pc_topic = "/hsrb/head_rgbd_sensor/depth_registered/rectified_points";
//    reset_topic = "ResetTopic";
//    octomap->SendCommand(enable_output, enable_input);
//    octomap->SendCommand(pc_topic, reset_topic);

}

FRASIEROpenRAVE::~FRASIEROpenRAVE() {
    env_->Destroy();


    if (run_joint_updater_flag_) {
        joint_state_thread_.join();
    }

    if (run_viewer_flag_) {
        viewer_->quitmainloop();
        viewer_thread_.join();
    }


}

void FRASIEROpenRAVE::jointSensorCb(const sensor_msgs::JointState::ConstPtr &msg) {
    boost::mutex::scoped_lock lock(joint_state_mutex_);
    joints_ = *msg;
    joint_state_flag_ = true;

}

void FRASIEROpenRAVE::baseStateCb(const geometry_msgs::Pose2D::ConstPtr &msg) {
    boost::mutex::scoped_lock lock(base_state_mutex_);
    base_ = *msg;
    base_state_flag_ = true;
}

void FRASIEROpenRAVE::ResetEnv() {
    std::cout << "FRASIEROpenRAVE: Environment reset." << std::endl;
    env_->Reset();
}

sensor_msgs::JointState FRASIEROpenRAVE::getWholeBodyState() {

    sensor_msgs::JointState state;

    state.name.push_back("odom_x");
    state.name.push_back("odom_y");
    state.name.push_back("odom_t");
    state.name.push_back("arm_lift_joint");
    state.name.push_back("arm_flex_joint");
    state.name.push_back("arm_roll_joint");
    state.name.push_back("wrist_flex_joint");
    state.name.push_back("wrist_roll_joint");


    state.position.push_back(base_.x);
    state.position.push_back(base_.y);
    state.position.push_back(base_.theta);
    state.position.push_back(joints_.position[12]);
    state.position.push_back(joints_.position[13]);
    state.position.push_back(joints_.position[14]);
    state.position.push_back(joints_.position[15]);
    state.position.push_back(joints_.position[16]);

    return state;
}

bool FRASIEROpenRAVE::loadHSR() { // TODO: Check if env exist

    std::string world_path;
    world_path = worlds_path_ + "hsr_empty_world.xml";
    bool success = env_->Load(world_path);

    // Get robot
    hsr_ = env_->GetRobot(robot_name_);

    // Get manipulator
    manip_ = hsr_->GetManipulator("whole_body");


    if (success) {
        std::cout << "RAVE: HSR initialized!" << std::endl;
    }

    return success;
}

bool FRASIEROpenRAVE::loadCustomEnv(std::string &world_path) {

    bool success = env_->Load(world_path);

    if (success) {
        std::cout << "RAVE: custom world initialized!" << std::endl;
    }

    return success;
}


void FRASIEROpenRAVE::viewerThread() {
    // OpenRAVE::EnvironmentMutex::scoped_lock lockenv(env_->GetMutex());

    std::string viewer_name = OpenRAVE::RaveGetDefaultViewerType(); // qtcoin

    viewer_ = OpenRAVE::RaveCreateViewer(env_, viewer_name);
    BOOST_ASSERT(!!viewer_);

    env_->Add(viewer_);

    // viewer_.SetBkgndColor() TODO: Change background colour

    viewer_->main(true); //TODO: Need EnvironmentMutex ???


}


void FRASIEROpenRAVE::getActiveJointIndex(std::vector<int> &q_index) {
    for (int i = 0; i < joint_names_.size(); i++) {
        q_index.push_back(hsr_->GetJoint(joint_names_[i])->GetDOFIndex());
    }
}


OpenRAVE::Transform FRASIEROpenRAVE::getRobotTransform() {
    return hsr_->GetLink(base_link_)->GetTransform();
}

OpenRAVE::Transform FRASIEROpenRAVE::getEEFTransform() {
    return hsr_->GetLink(eef_link_)->GetTransform();
}

geometry_msgs::Pose2D FRASIEROpenRAVE::getRobotPose() {
    return base_;
}

void FRASIEROpenRAVE::updateJointStatesThread() {

    ros::Rate r(50);
    std::vector<int> q_index;
    std::vector<double> q(hsr_->GetDOF(), 0.0);
    std::vector<double> q_v(hsr_->GetDOF(), 0.0);
    std::vector<std::string> q_names;

    hsr_->GetActiveDOFValues(q);
    hsr_->GetActiveDOFVelocities(q_v);
    getActiveJointIndex(q_index);

    std::cout << "RAVE: started updating joints!" << '\n';

    while (ros::ok()) {

        if (joint_state_flag_ && base_state_flag_) {

            {
                boost::mutex::scoped_lock lock(joint_state_mutex_);
                {
                    boost::mutex::scoped_lock lock(base_state_mutex_);

                    // Set positions // TODO: Fix minus theta
                    q[q_index[0]] = base_.x;
                    q[q_index[1]] = base_.y;
                    q[q_index[2]] = base_.theta;


                    q[q_index[3]] = joints_.position[14];
                    q[q_index[4]] = joints_.position[15];
                    q[q_index[5]] = joints_.position[16];
                    q[q_index[6]] = joints_.position[17];
                    q[q_index[7]] = joints_.position[18];
                    q[q_index[8]] = joints_.position[20];
                    q[q_index[9]] = joints_.position[22];
                    q[q_index[10]] = joints_.position[24];
                    q[q_index[11]] = joints_.position[26];
                    q[q_index[12]] = joints_.position[28];
                    q[q_index[13]] = joints_.position[12];
                    q[q_index[14]] = joints_.position[13];


                    // Set velocities TODO: Set velocities
                    // q[q_index[0]] = base_.x;
                    // q[q_index[1]] = base_.y;
                    // q[q_index[2]] = base_.theta;

                    q_v[q_index[3]] = joints_.velocity[14];
                    q_v[q_index[4]] = joints_.velocity[15];
                    q_v[q_index[5]] = joints_.velocity[16];
                    q_v[q_index[6]] = joints_.velocity[17];
                    q_v[q_index[7]] = joints_.velocity[18];
                    q_v[q_index[8]] = joints_.velocity[20];
                    q_v[q_index[9]] = joints_.velocity[22];
                    q_v[q_index[10]] = joints_.velocity[24];
                    q_v[q_index[11]] = joints_.velocity[26];
                    q_v[q_index[12]] = joints_.velocity[28];
                    q_v[q_index[13]] = joints_.velocity[12];
                    q_v[q_index[14]] = joints_.velocity[13];

                }

            }

            hsr_->SetActiveDOFValues(q);
            hsr_->SetActiveDOFVelocities(q_v);

        }

        r.sleep();
    }

}

void FRASIEROpenRAVE::playTrajectory(trajectory_msgs::JointTrajectory &traj) {
    for (int i = 0; i < traj.points.size(); i++) {
        hsr_->SetDOFValues(traj.points[i].positions, 1, whole_body_joint_index_);
        hsr_->SetDOFVelocities(traj.points[i].velocities, 1, whole_body_joint_index_);
        ros::Duration(traj.points[1].time_from_start).sleep();
    }
}

void FRASIEROpenRAVE::moveToHomeState() {
    std::vector<double> q_home = {0, 0, 0, 0, 0, 0, 0, 0};
    hsr_->SetDOFValues(q_home, 1, whole_body_joint_index_);
}

void FRASIEROpenRAVE::setEEFValue(double &v) {
    std::vector<double> q;
    q.push_back(v);
    hsr_->SetDOFValues(q, 1, eef_joint_index_);
}
