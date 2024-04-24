#include <ros/ros.h>

#include "venusblue.hpp"

int main(int argc, char* argv[])
{
    // Initialise ROS parameters.
    try {
        ros::init(argc, argv, "venusblue");
    }
    catch (const std::exception &err) {
        ROS_ERROR("Failed to initialises ros: %s", err.what());
        return 1;
    }

    // The NodeHandle constructor initialises the ROS node.
    ros::NodeHandle node;

    VenusBlue venusblue {&node};
    return venusblue.main();
}
