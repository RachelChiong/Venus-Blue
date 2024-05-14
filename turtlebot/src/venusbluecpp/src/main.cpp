#include <rclcpp/rclcpp.hpp>

#include "venusblue.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<venusblue::VenusBlue>());
    rclcpp::shutdown();
    return 0;
}
