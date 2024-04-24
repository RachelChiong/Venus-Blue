#include <string>
#include <chrono>

#include "venusblue.hpp"

using namespace std::chrono_literals;

VenusBlue::VenusBlue(ros::NodeHandle *node)
    : m_node(node)
{}

int VenusBlue::main()
{
    while (ros::ok()) {
        ROS_INFO("VenusBlue time is %f", ros::Time::now().toSec());
        std::this_thread::sleep_for(1s);
    }

    return 0;
}
