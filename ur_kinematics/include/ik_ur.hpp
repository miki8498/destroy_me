#ifndef IK_ROBOTICS_H
#define IK_ROBOTICS_H

#include <rclcpp/rclcpp.hpp>
#include <stdio.h>  
#include <vector>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <math.h> 
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "message/srv/ur_inverse_kinematics.hpp"
#include "message/srv/ur_forward_kinematics.hpp"
#include <sensor_msgs/msg/joint_state.h>

#include <boost/thread.hpp>


namespace inverse_kinem
{

    class FIKServer : public rclcpp::Node
    {

        public:

            FIKServer(int rate);
            virtual ~FIKServer(); // Class Destructor
            

        private:

            //Variables
            int rate;
        //     int num_sols;
            double d1, d4, d5,d6;
            double a2, a3;
            double alph1, alph4, alph5;

            //vector of joint limits                1      1       2    2      3     3          4     4         5     5       6      6
            std::vector<double> joint_limits = {-2*M_PI, 2*M_PI, -M_PI, M_PI, -M_PI, M_PI, -2*M_PI, 2*M_PI, -2*M_PI, 2*M_PI, -2*M_PI, 2*M_PI};
           
            Eigen::Matrix4d T_0_6 = Eigen::Matrix4d::Identity();
            Eigen::Matrix<double,6,1> q;
            Eigen::Matrix<double,8,6> ik_solutions = Eigen::Matrix<double,8,6>::Zero();

            rclcpp::Service<message::srv::UrForwardKinematics>::SharedPtr fk_service ;
            rclcpp::Service<message::srv::UrInverseKinematics>::SharedPtr ik_service ;
            std::string ur_type;
            message::srv::UrInverseKinematics::Response res_new;
            std::vector<double> max_error;
            bool verbose;
            std::vector<double> last_joints;
        
            Eigen::Matrix3d Rx_fk, Rz_fk;

            //Methods
            void initialiseRos();
            void coefficients();

            bool fk_cb(const std::shared_ptr<message::srv::UrForwardKinematics::Request> req, std::shared_ptr<message::srv::UrForwardKinematics::Response> res);
            bool ik_cb(const std::shared_ptr<message::srv::UrInverseKinematics::Request> req, std::shared_ptr<message::srv::UrInverseKinematics::Response> res);

            Eigen::Matrix4d HTrans(std::vector<double> q);
            Eigen::Matrix4d AH(double alpha, double a, double d, double theta);
            Eigen::Matrix<double,8,6> invKine(Eigen::Matrix4d desired_pos);

            bool checkq6();

    };
}

#endif //IK_ROBOTICS_H