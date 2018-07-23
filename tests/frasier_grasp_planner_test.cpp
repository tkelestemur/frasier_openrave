#include <frasier_openrave/frasier_openrave.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "frasier_openrave_test");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  FRASIEROpenRAVE rave(nh, true, false);
  ros::Duration(1.0).sleep();

  OpenRAVE::Transform cylinder_pose(OpenRAVE::Vector(1, 0, 0, 0), OpenRAVE::Vector(1.2, 0.75, 0.844));
  OpenRAVE::Vector cylinder_size(0.04, 0.30, 0.0);
  std::string cylinder_name = "cylinder";
  rave.addCylinderCollObj(cylinder_size, cylinder_pose, cylinder_name);

  rave.sampleGraspPoses(cylinder_name);

  ros::waitForShutdown();
  return 0;
}
