#include "rclcpp/rclcpp.hpp"
#include "point_cloud_op.hpp"

int main(int argc, char **argv)
{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Segment>());
    
    
    rclcpp::shutdown();
    return 0;
}