#pragma once

#include <chrono>
#include <mutex>
#include <memory>
#include <deque>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mqtt/async_client.h>

namespace venusblue {

class VenusBlue : public rclcpp::Node
{
public:

    /**
     * @brief Create a new application instance from a ros node.
     */
    VenusBlue();

    /**
     * @brief The destructor joins any threads.
     */
    ~VenusBlue();

    /**
     * @brief The main thread of the application.
     * @returns The main program return code.
     */
    int main();

private:

    /**
     * @brief Connects to the mqtt server.
    */
    bool mqtt_connect();

    /**
     * @brief Callback to handle mqtt messages with.
     * @param message A message received on mqtt.
     */
    void mqtt_callback(mqtt::const_message_ptr message);

    /**
     * @brief Thread function emitting controls.
    */
    void control_thread();

    /**
     * @brief Transform a control input (x, y, z) to the velocity on the left
     * and right of the turtlebot.
     * 
     * @param x The x parameter.
     * @param y The y parameter.
     * @param z The z parameter.
     * 
     * @returns A pair of (vx, vy) being the left and right velocities.
     */
    std::pair<double, double> control_to_velocity(int x, int y, int z);

    /// The main thread.
    std::thread m_main_thread;

    /// Thread sending control signals to the motors.
    std::thread m_control_thread;

    /// Topic to publish control commands to.
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher_control;

    /// Topic to publish robot state to.
    // rclcpp::Publisher< m_publisher_state;

    /// Mutex protecting concurrent access to the control queue.
    std::mutex m_mutex;

    /// Condition that a message has arrived.
    std::condition_variable m_condition;

    /// Queue of (x, y, z) control parameters.
    std::tuple<int, int, int> m_control;

    /// The MQTT client connection.
    std::unique_ptr<mqtt::async_client> m_mqtt;
};

} // namespace venusblue
