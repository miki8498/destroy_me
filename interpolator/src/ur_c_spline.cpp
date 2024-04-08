#include "rclcpp/rclcpp.hpp"
#include "Spline.hpp"

int main(int argc, char **argv)
{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ur_c_spline::Spline>(500));
    rclcpp::shutdown();
    return 0;
}