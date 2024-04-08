#include "rclcpp/rclcpp.hpp"
#include "ik_ur.hpp"

int main(int argc, char **argv)
{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<inverse_kinem::FIKServer>(1000));
    rclcpp::shutdown();
    return 0;
}