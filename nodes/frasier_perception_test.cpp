#include <frasier_openrave/frasier_openrave.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "frasier_perception_test");
    ros::NodeHandle nh; //

    FRASIEROpenRAVE rave(nh, true);

    rave.LoadHSR();

    rave.startThreads();
    ros::Duration(1.0).sleep();

    // Test perception
//    rave.addCollisionObj();
//    ros::Duration(2.0).sleep();
//    std::string obj_name = "test_obj";
//    rave.removeCollisionObj(obj_name);
//
//    rave.startROSSpinner();



    return 0;
}
