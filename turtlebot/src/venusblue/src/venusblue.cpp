#include <string>
#include <chrono>

#include "venusblue.hpp"

using namespace std::chrono_literals;

VenusBlue::VenusBlue(ros::NodeHandle *node)
    : m_node(node)
    , m_mqtt_server("csse4011-iot.zones.eait.uq.edu.au")
    , m_mqtt_port(1883)
{
    if (!m_node.getParam("mqtt_server", m_server)) {
        ROS_INFO("Couldn't find \"mqtt_server\" parameter.");
    }

    if (!m_node.getParam("mqtt_port", m_port)) {
        ROS_INFO("Couldn't find \"mqtt_port\" parameter.");
    }

    ROS_INFO(
        "Using \"mqtt_server\" %s on port %i",
        m_mqtt_server.c_str(),
        m_mqtt_port
    );
}

int VenusBlue::main()
{
    while (ros::ok()) {
        ROS_INFO("VenusBlue time is %f", ros::Time::now().toSec());
        std::this_thread::sleep_for(1s);
    }

    return 0;
}

bool VenusBlue::connect_mqtt()
{
    
}
