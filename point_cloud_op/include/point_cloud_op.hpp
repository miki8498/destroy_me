#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
// #include <vision_msgs/srv/seg_cloud.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <message/srv/point_cloud.hpp>
#include <float.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "std_srvs/srv/trigger.hpp"
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include<pcl/io/ply_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/impl/common.hpp>

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::PointCloud<PointT> PointCloudIntT;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;

class Segment : public rclcpp::Node
{
    public:
        Segment();			//Class Constructor #1
        virtual ~Segment();			//Class Destructor
        
    private:
        
        std::string topic_cloud_, topic_srv_;
        sensor_msgs::msg::PointCloud2 sharedMsg;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_srv_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_2;

        void initialiseRos();
        bool serviceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);
        
        void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        pcl::PointCloud<pcl::PointXYZ>::Ptr remove_ground();
        std::set<std::string> getMeshNames(const std::string &filePath, bool my_xacro);

        
};


