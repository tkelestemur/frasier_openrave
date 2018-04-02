#include <frasier_openrave/frasier_openrave.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "frasier_openrave_test");// ros init
    ros::NodeHandle nh; // Create a node handle and start the node

    FRASIEROpenRAVE frasier_or(nh);

      frasier_or.LoadHSR();

      frasier_or.startThreads();
      ros::Duration(2.0).sleep();
      // frasier_or.initPlanner();

      // std::vector<double> q = {0.5, 0, 0, 0, 0, 0, 0, 0};

      frasier_or.computeTrajectory();
      frasier_or.startROSSpinner();


    return 0;
}
