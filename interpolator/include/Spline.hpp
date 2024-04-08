

#ifndef SPLINE_H
#define SPLINE_H

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <FIRFilter.hpp>
#include <string>

#include <message/srv/interpolate.hpp>
#include <message/srv/joint_interpolate.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>


using Vector6d = Eigen::Matrix<double,6,1>;


namespace ur_c_spline
{

    class Spline : public rclcpp::Node
    {
        public:
            Spline(int rate);			//Class Constructor #1
            virtual ~Spline();			//Class Destructor
            
            
        private:
            rclcpp::Service<message::srv::Interpolate>::SharedPtr interpolate_service;
            rclcpp::Service<message::srv::Interpolate>::SharedPtr interpolate_service_linear;
            rclcpp::Service<message::srv::JointInterpolate>::SharedPtr joint_interpolate_service;

            std::vector<fir_filter::FIRFilter*> filter;

            float rate_spline;
            int slerp_steps;
            float period;
            
            geometry_msgs::msg::Pose start_pose;
            geometry_msgs::msg::Pose goal_pose;
            std::vector<geometry_msgs::msg::Pose> trajectory;

            std_msgs::msg::Float64MultiArray start_joints;
            std::vector<std_msgs::msg::Float64MultiArray> joints_trajectory;
            


            // //methods
            void initialiseRos();
            // // void initialiseServer();
            Eigen::Quaterniond quaternion_negation(Eigen::Quaterniond v);

            void initialiseFilters(float period);
            void initialiseFiltersLinear(float period);
            void initialiseFiltersJoints(float period);

            bool interpolate_serviceCB(const std::shared_ptr<message::srv::Interpolate::Request> req, std::shared_ptr<message::srv::Interpolate::Response> res);
            bool joint_interpolate_serviceCB(const std::shared_ptr<message::srv::JointInterpolate::Request> req, std::shared_ptr<message::srv::JointInterpolate::Response> res);
            bool interpolate_service_linearCB(const std::shared_ptr<message::srv::Interpolate::Request> req, std::shared_ptr<message::srv::Interpolate::Response> res);
    };


}

#endif /* SPLINE_H */
