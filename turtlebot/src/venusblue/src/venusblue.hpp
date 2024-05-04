#pragma once

#include <memory>

#include <mqtt/async_client.h>
#include "ros/ros.h"

class VenusBlue
{
public:

    VenusBlue(ros::NodeHandle *node);

    int main();

private:

    int initialise();

    void publish_state();

    void control_callback();

    bool connect_mqtt();

    /// The ROS node.
    ros::NodeHandle *m_node;

    /// The mqtt broker server address.
    std::string m_mqtt_server;

    /// The mqtt broker port.
    int m_mqtt_port;

    /// Topic to publish control commands to.
    ros::Publisher m_publisher_control;

    /// Topic to publish robot state to.
    ros::Publisher m_publisher_state;

    /// The MQTT client connection.
    std::unique_ptr<mqtt::async_client> m_mqtt;
};
