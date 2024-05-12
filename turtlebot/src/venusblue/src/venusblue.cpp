#include "venusblue.hpp"

#include <memory>
#include <string>
#include <chrono>

#include <mqtt/exception.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace std::chrono_literals;

using namespace std::chrono_literals;

VenusBlue::VenusBlue(ros::NodeHandle *node)
    : m_node(node)
    , m_mqtt_server("csse4011-iot.zones.eait.uq.edu.au")
    , m_mqtt_port("1883")
    , m_mqtt_timeout(10000)
    , m_mqtt_pedal_topic("pedal")
    , m_mqtt(nullptr)
{
    if (!m_node->getParam("mqtt_server", m_mqtt_server)) {
        ROS_INFO("Couldn't find \"mqtt_server\" parameter.");
    }

    if (!m_node->getParam("mqtt_port", m_mqtt_port)) {
        ROS_INFO("Couldn't find \"mqtt_port\" parameter.");
    }

    if (!m_node->getParam("mqtt_timeout", m_mqtt_timeout)) {
        ROS_INFO("Couldn't find \"mqtt_timeout\" parameter.");
    }

    if (!m_node->getParam("mqtt_pedal_topic", m_mqtt_pedal_topic)) {
        ROS_INFO("Couldn't find \"mqtt_pedal_topic\" parameter.");
    }
}

int VenusBlue::main()
{
    while (ros::ok()) {

        // Wait to connect to the mqtt server.
        while (!connect_mqtt());

        ROS_INFO("VenusBlue time is %f", ros::Time::now().toSec());
        std::this_thread::sleep_for(1s);
    }

    return 0;
}

bool VenusBlue::connect_mqtt()
{
    ROS_INFO(
        "MQTT connecting to %s on port %s ...",
        m_mqtt_server.c_str(),
        m_mqtt_port.c_str()
    );

    // If already initialised then attempt to reconnect.
    if (m_mqtt) {
        mqtt::token_ptr token;
        try {
            token = m_mqtt->reconnect();
            token->wait_for(m_mqtt_timeout);
        }
        catch (const std::exception & err) {
            ROS_ERROR("Failed to reconnect to MQTT. %s", token->get_error_message().c_str());
        }
    }

    m_mqtt = std::make_unique<mqtt::async_client>(
        m_mqtt_server,
        m_mqtt_port,
        nullptr
    );

    m_mqtt->set_message_callback(
        [this](mqtt::const_message_ptr message){ mqtt_callback(message); }
    );

    m_mqtt->set_connected_handler(
        [this](const std::string& cause){
            m_mqtt->subscribe(m_mqtt_pedal_topic, 1)->wait();
        }
    );

    m_mqtt->set_disconnected_handler(
        [this](const mqtt::properties &properties, mqtt::ReasonCode code){}
    );

    // Begin handling incoming messages.
    m_mqtt->start_consuming();

    auto options = mqtt::connect_options_builder()
        .automatic_reconnect()
        .connect_timeout(std::chrono::milliseconds(m_mqtt_timeout))
        .keep_alive_interval(1s)
        .clean_session(false)
        .finalize();

    // Connect to the server.
    mqtt::token_ptr response;
    try {
        response = m_mqtt->connect(options);
        auto server = response->get_connect_response();
    }
    catch (const std::exception &err) {
        ROS_ERROR("Failed to connect to MQTT. %s", err.what());
        return false;
    }

    if (response->get_reason_code() != mqtt::ReasonCode::SUCCESS) {
        ROS_ERROR("Failed to connect to MQTT. %s", response->get_error_message().c_str());
        return false;
    }

    ROS_INFO("MQTT connected!");
    return true;
}

void VenusBlue::mqtt_callback(mqtt::const_message_ptr message)
{
    const std::string &string = message->get_payload_str();

    double x, y, z;
    try {
        json object = json::parse(string);
        x = object.at("x").get<double>();
        y = object.at("y").get<double>();
        z = object.at("z").get<double>();
    }
    catch (const std::exception &err) {
        ROS_ERROR("Failed to parse json message %s.", string.c_str());
        return;
    }

    std::unique_lock<std::mutex> lock(m_mutex);
    m_control_queue.emplace_back(x, y, z);
}

// void VenusBlue::control_callback()
// {

// }
