#include <frasier_openrave/frasier_openrave.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "frasier_perception_test");
    ros::NodeHandle nh;

    FRASIEROpenRAVE rave(nh, true);

    rave.loadHSR();

    rave.startThreads();
    ros::Duration(1.0).sleep();


    return 0;
}
