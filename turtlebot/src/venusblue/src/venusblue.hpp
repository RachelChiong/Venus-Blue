#pragma once

#include <mutex>
#include <memory>
#include <deque>

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

    /**
     * @brief Callback to handle mqtt messages with.
     * @param message A message received on mqtt.
     */
    void mqtt_callback(mqtt::const_message_ptr message);

    bool connect_mqtt();

    /// The ROS node.
    ros::NodeHandle *m_node;

    /// The mqtt broker server address.
    std::string m_mqtt_server;

    /// The mqtt broker port.
    std::string m_mqtt_port;

    /// Timeout duration in milliseconds for mqtt connect and reconnection.
    int m_mqtt_timeout;

    /// Topic to get pedal messages from.
    std::string m_mqtt_pedal_topic;

    /// Topic to publish control commands to.
    ros::Publisher m_publisher_control;

    /// Topic to publish robot state to.
    ros::Publisher m_publisher_state;

    /// Mutex protecting concurrent access to the control queue.
    std::mutex m_mutex;

    /// Queue of (x, y, z) control parameters.
    std::deque<std::tuple<double, double, double>> m_control_queue;

    /// The MQTT client connection.
    std::unique_ptr<mqtt::async_client> m_mqtt;
};
