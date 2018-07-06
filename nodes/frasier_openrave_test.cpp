#include <frasier_openrave/frasier_openrave.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "frasier_openrave_test");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  FRASIEROpenRAVE rave(nh, true, false);
  ros::Duration(1.0).sleep();

  ros::waitForShutdown();
  return 0;
}
