#include <frasier_openrave/frasier_openrave.h>

////////////////////////////// UTILITIES //////////////////////////////
trajectory_msgs::JointTrajectory FRASIEROpenRAVE::eigenMatrixToTraj(Eigen::MatrixXd &traj) {

  int n_joints = manip_->GetArmDOF();

  trajectory_msgs::JointTrajectory traj_ros;

  traj_ros.joint_names.push_back("odom_x");
  traj_ros.joint_names.push_back("odom_y");
  traj_ros.joint_names.push_back("odom_t");
  traj_ros.joint_names.push_back("arm_lift_joint");
  traj_ros.joint_names.push_back("arm_flex_joint");
  traj_ros.joint_names.push_back("arm_roll_joint");
  traj_ros.joint_names.push_back("wrist_flex_joint");
  traj_ros.joint_names.push_back("wrist_roll_joint");


  traj_ros.points.resize(traj.rows());

  for (int i = 0; i < traj.rows(); i++) {
    traj_ros.points[i].positions.resize(n_joints);
    traj_ros.points[i].velocities.resize(n_joints);
    traj_ros.points[i].accelerations.resize(n_joints);

    for (int j = 0; j < n_joints; j++) {
      traj_ros.points[i].positions[j] = traj(i, j);
      traj_ros.points[i].velocities[j] = 0.0;
      traj_ros.points[i].accelerations[j] = 0.0;

    }
  }

  return traj_ros;
}

geometry_msgs::Pose FRASIEROpenRAVE::orTransformToROSPose(OpenRAVE::Transform &transform) {
  geometry_msgs::Pose pose;
  pose.position.x = transform.trans.x;
  pose.position.y = transform.trans.y;
  pose.position.z = transform.trans.z;

  pose.orientation.w = transform.rot[0];
  pose.orientation.x = transform.rot[1];
  pose.orientation.y = transform.rot[2];
  pose.orientation.z = transform.rot[3];

  return pose;

}

void FRASIEROpenRAVE::drawTransform(OpenRAVE::Transform &T, bool transparent) {
  OpenRAVE::EnvironmentMutex::scoped_lock lockenv(env_->GetMutex());
  OpenRAVE::TransformMatrix R;
  OpenRAVE::geometry::matrixFromQuat(R, T.rot);
  double arrow_length = 0.1;
  float arrow_width = 0.005;
  OpenRAVE::Vector origin(T.trans);

  OpenRAVE::Vector x_axis = origin + arrow_length * OpenRAVE::Vector(R.m[0], R.m[4], R.m[8]);
  OpenRAVE::Vector y_axis = origin + arrow_length * OpenRAVE::Vector(R.m[1], R.m[5], R.m[9]);
  OpenRAVE::Vector z_axis = origin + arrow_length * OpenRAVE::Vector(R.m[2], R.m[6], R.m[10]);

  OpenRAVE::Vector color_r, color_g, color_b;
  if(transparent){
    color_r = OpenRAVE::Vector(1, 0, 0, 0.3);
    color_g = OpenRAVE::Vector(0, 1, 0, 0.3);
    color_b = OpenRAVE::Vector(0, 0, 1, 0.3);
  }
  else{
    color_r = OpenRAVE::Vector(1, 0, 0, 1);
    color_g = OpenRAVE::Vector(0, 1, 0, 1);
    color_b = OpenRAVE::Vector(0, 0, 1, 1);
  }

  graph_handles_.push_back(env_->drawarrow(origin, x_axis, arrow_width, color_r));
  graph_handles_.push_back(env_->drawarrow(origin, y_axis, arrow_width, color_g));
  graph_handles_.push_back(env_->drawarrow(origin, z_axis, arrow_width, color_b));
}

void FRASIEROpenRAVE::drawArrow(OpenRAVE::Vector &origin, OpenRAVE::Vector &vector, bool transparent) {
  double arrow_length = 0.02;
  float arrow_width = 0.002;
  OpenRAVE::Vector target = origin + arrow_length * vector;
  OpenRAVE::Vector color;
  if(transparent){
    color = OpenRAVE::Vector(1, 0, 0, 0.3);
  }else{
    color = OpenRAVE::Vector(1, 0, 0, 1);
  }

  graph_handles_.push_back(env_->drawarrow(origin, target, arrow_width, color));

}

void FRASIEROpenRAVE::drawPoints(std::vector<OpenRAVE::Vector> &points) {
  std::vector<float> points_v(3 * points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    points_v[3 * i + 0] = static_cast<float>(points[i][0]);
    points_v[3 * i + 1] = static_cast<float>(points[i][1]);
    points_v[3 * i + 2] = static_cast<float>(points[i][2]);
  }

  int const num_points = static_cast<int>(points.size());
  float point_size = 10.0;
  graph_handles_.push_back(env_->plot3(&points_v.front(), num_points, 3 * sizeof(float), point_size, OpenRAVE::Vector(0, 1, 0, 1)));


}

void FRASIEROpenRAVE::drawPoint(OpenRAVE::Vector &point) {
  std::vector<float> point_v(3);
  point_v[0] = static_cast<float>(point[0]);
  point_v[1] = static_cast<float>(point[1]);
  point_v[2] = static_cast<float>(point[2]);

  int num_points = 1;
  float point_size = 10.0;

  graph_handles_.push_back(env_->plot3(&point_v.front(), num_points, 3 * sizeof(float), point_size, OpenRAVE::Vector(0, 1, 0, 1)));

}


