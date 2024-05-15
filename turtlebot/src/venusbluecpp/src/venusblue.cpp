#include "venusblue.hpp"

#include <memory>
#include <string>
#include <chrono>
#include <iostream>

#include <mqtt/exception.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace std::chrono_literals;

namespace venusblue {

static constexpr const char
    *API_MQTT_SERVER = "mqtt_server",
    *API_MQTT_TIMEOUT = "mqtt_timeout",
    *API_MQTT_CONTROL_TOPIC = "mqtt_control_topic",
    *API_MOTOR_CONTROL_TOPIC = "motor_control_topic";

VenusBlue::VenusBlue()
    : rclcpp::Node("venusbluecpp")
    , m_mqtt(nullptr)
{
    this->declare_parameter(API_MQTT_SERVER, "csse4011-iot.zones.eait.uq.edu.au");
    this->declare_parameter(API_MQTT_TIMEOUT, 1000);
    this->declare_parameter(API_MQTT_CONTROL_TOPIC, "venusBlueFootPedal");
    this->declare_parameter(API_MOTOR_CONTROL_TOPIC, "cmd_vel");
    // this->declare_parameter(API_MOTOR_STATE_TOPIC, "joint_states");

    m_publisher_control = this->create_publisher<geometry_msgs::msg::Twist>(
        this->get_parameter(API_MOTOR_CONTROL_TOPIC).as_string(), 10
    );

    m_main_thread = std::thread(&VenusBlue::main, this);
    m_control_thread = std::thread(&VenusBlue::control_thread, this);
}

VenusBlue::~VenusBlue()
{
    m_condition.notify_all();
    m_main_thread.join();
    m_control_thread.join();
}

int VenusBlue::main()
{
    mqtt_connect();

    while (rclcpp::ok()) {

        if (!m_mqtt->is_connected() && !mqtt_connect()) {
            std::this_thread::sleep_for(1s);
            continue;
        }

        // RCLCPP_INFO(get_logger(), "VenusBlue time is %f", rclcpp::Time::now().toSec());
        std::this_thread::sleep_for(1s);
    }

    RCLCPP_INFO(this->get_logger(), "Exiting");

    return 0;
}

bool VenusBlue::mqtt_connect()
{
    auto server = this->get_parameter(API_MQTT_SERVER).as_string();

    RCLCPP_INFO(this->get_logger(), "mqtt connecting to %s ...", server.c_str());

    if (m_mqtt) {
        try {
            m_mqtt->disconnect();
        }
        catch (const std::exception & err) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to cleanly disconnect. %s",
                err.what()
            );
        }
    }

    m_mqtt = std::make_unique<mqtt::async_client>(server, "venusblue", nullptr);

    m_mqtt->set_message_callback(
        [this](mqtt::const_message_ptr message){ mqtt_callback(message); }
    );

    m_mqtt->set_connected_handler(
        [this](const std::string&){
            auto topic = this->get_parameter(API_MQTT_CONTROL_TOPIC).as_string();
            m_mqtt->subscribe(topic.c_str(), 1)->wait();
        }
    );

    // Begin handling incoming messages.
    m_mqtt->start_consuming();

    auto timeout = this->get_parameter(API_MQTT_TIMEOUT).as_int();

    auto options = mqtt::connect_options_builder()
        .connect_timeout(std::chrono::milliseconds(timeout))
        .clean_session(false)
        .finalize();

    // Connect to the server.
    mqtt::token_ptr response;
    try {
        response = m_mqtt->connect(options);
        response->get_connect_response();
    }
    catch (const std::exception &err) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to connect to mqtt. %s", err.what()
        );
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "mqtt connected");
    return true;
}

void VenusBlue::mqtt_callback(mqtt::const_message_ptr message)
{
    const std::string &string = message->get_payload_str();

    int x, y, z;
    try {
        json object = json::parse(string);
        x = object.at("x").get<int>();
        y = object.at("y").get<int>();
        z = object.at("z").get<int>();
    }
    catch (const std::exception &err) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to parse json message %s.", string.c_str()
        );
        return;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Got control x = %d, y = %d, z = %d", x, y, z
    );

    /// Add the message to the control queue.
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_control = std::make_tuple(x, y, z);
    }

    /// Notify the main thread that a message was received.
    m_condition.notify_all();
}

void VenusBlue::control_thread()
{
    return;

    while (!rclcpp::ok()) {

        std::unique_lock<std::mutex> lock(m_mutex);

        // Wait for a message.
        auto timeout = m_condition.wait_for(lock, 1s);
        if (timeout == std::cv_status::timeout)
            continue;

        // Get the message and release the lock so the message can be updated.
        auto [x, y, z] = m_control;
        lock.release();

        // Transform the control to a left and right velocity.
        auto [vx, vy] = control_to_velocity(x, y, z);

        RCLCPP_INFO(this->get_logger(), "Velocity vx = %f, vy = %f", vx, vy);

        geometry_msgs::msg::Twist message;
        message.linear.x = 0;
        message.linear.y = 0;
        message.linear.z = 0;
        message.angular.x = vx;
        message.angular.y = vy;
        message.angular.z = 0;

        m_publisher_control->publish(message);
    }
}

std::pair<double, double> VenusBlue::control_to_velocity(
    int x,
    int y,
    int
) {
    return std::make_pair(
        (x & 0x7F) / 128.0 * 6,
        (y & 0x3f) / 64 * 6
    );
}

} // namespace venusblue
