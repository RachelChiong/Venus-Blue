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

    /// @brief The ROS node.
    ros::NodeHandle *m_node;

    /// @brief Topic to publish control commands to.
    ros::Publisher m_publisher_control;

    /// @brief Topic to publish robot state to.
    ros::Publisher m_publisher_state;

    /// @brief The MQTT client connection.
    std::unique_ptr<mqtt::async_client> m_mqtt;
};
