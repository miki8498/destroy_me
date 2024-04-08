#include "rclcpp/rclcpp.hpp"
#include "point_cloud_op.hpp"


Segment::Segment(): Node("point_cloud_removal")
    {
        this->initialiseRos();
    }


Segment::~Segment(){}

void Segment::initialiseRos()
    {
        this->get_srv_ = create_service<std_srvs::srv::Trigger>("points_remove", std::bind(&Segment::serviceCallback, this, std::placeholders::_1, std::placeholders::_2));
        this->sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/points", 1, std::bind(&Segment::topic_callback, this,std::placeholders::_1));
        this->pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/camera/points2", 1);

    }


void Segment::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        this->sharedMsg = *msg;

        // sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
        // sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
        // sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

        // for(size_t i = 0; i < this->sharedMsg.height * this->sharedMsg.width; ++i, ++iter_x, ++iter_y, ++iter_z) {
        //     std::cout << "coords: " << *iter_x << ", " << *iter_y << ", " << *iter_z << '\n';
        // }
    }

bool Segment::serviceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res){
    // -----------------------
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_no_ground (new pcl::PointCloud<pcl::PointXYZ>);

    scene_no_ground = remove_ground();

    std::string filePath_hand = "/home/michela/ros2/dual_robot_ws/src/robotiq_hand_e/urdf/robotiq_wiring.urdf.xacro";
    
    std::string filePath_arm = "/home/michela/ros2/dual_robot_ws/src/Universal_Robots_ROS2_Description/urdf/ur_macro.xacro";

    std::set<std::string> meshes_hand; 
    meshes_hand = getMeshNames(filePath_hand, true);
    std::set<std::string> meshes_arm;
    meshes_arm = getMeshNames(filePath_arm, false);
   
    //print cloud_filtered typex

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gripper (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PLYReader Reader;
    // Reader.read("/home/michela/ros2/dual_robot_ws/src/robotiq_hand_e/meshes/wiring_gripper.ply", *cloud_gripper);

    // Eigen::Matrix4f hand_trans;
    // hand_trans<< 0.391, -0.028,  0.920, -0.219,
    //              0.068,  0.998,  0.002, -0.126,
    //             -0.918,  0.062,  0.392,  0.607,
    //              0.000,  0.000,  0.000,  1.000;

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gripper_world(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::transformPointCloud(*cloud_gripper, *cloud_gripper_world, hand_trans);

    // // convex hull
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::ConvexHull<pcl::PointXYZ> chull;
    // chull.setInputCloud(cloud_gripper_world);
    // chull.reconstruct(*cloud_hull);
    
    // std::vector<int> cropInlierIndices;
    // Eigen::Vector4f minPt, maxPt;
    // pcl::getMinMax3D(*cloud_hull, minPt, maxPt);
    // pcl::CropBox<pcl::PointXYZ> cropBoxFilter(true);
    // cropBoxFilter.setMin(minPt);
    // cropBoxFilter.setMax(maxPt);
    // cropBoxFilter.setInputCloud(cloud_filtered);
    // cropBoxFilter.setNegative(true);
    // cropBoxFilter.filter(cropInlierIndices);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr croppedCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::copyPointCloud(*cloud_filtered, cropInlierIndices, *croppedCloudPtr);
    // sensor_msgs::msg::PointCloud2 cloud_output_2;

    // pcl::toROSMsg(*croppedCloudPtr, cloud_output_2);
    // cloud_output_2.header.frame_id = "world";
    // this->pub_->publish(cloud_output_2);


    res->success = true;
}

std::set<std::string> Segment::getMeshNames(const std::string &filePath, bool my_xacro){
     // Open the Xacro file
    std::ifstream xacroFile(filePath);
    std::set<std::string> uniqueMeshNames; 
    std::string line;
    size_t filenameStart;
    int character_to_remove;
    size_t namePos;

    //Check if the file is opened successfully
    if (!xacroFile.is_open()) {
        std::cerr << "Failed to open Xacro file: " << filePath << std::endl;
        return uniqueMeshNames; // Return with error code
    }

    while (std::getline(xacroFile, line)) {
        // Check if the line contains the desired tag
        if(my_xacro == true){
            filenameStart = line.find("filename=\"file://$(find robotiq_hand_e)/meshes/");
            character_to_remove = 47;}
        else{
            namePos = line.find("<xacro:get_mesh");
            filenameStart = line.find("name=\"", namePos);
            character_to_remove = 6;}

        if (filenameStart != std::string::npos) {
            // Extract the substring after the filename attribute
            std::string filenameSubstring = line.substr(filenameStart + character_to_remove); // 47 is the length of "filename=\"file://$(find robotiq_hand_e)/meshes/"
            // Find the position of the closing quote
            size_t filenameEnd = filenameSubstring.find("\"");
            if (filenameEnd != std::string::npos) {
                // Extract the substring before the closing quote
                std::string meshName = filenameSubstring.substr(0, filenameEnd);
                if(my_xacro){
                    std::cout << my_xacro << std::endl;
                    // Find the position of the extension ".stl"
                    size_t extensionPos = meshName.rfind(".stl");
                    if (extensionPos != std::string::npos) {
                        // Remove the extension ".stl"
                        meshName = meshName.substr(0, extensionPos);
                    }
                }
                // Check if mesh name is unique
                if (uniqueMeshNames.find(meshName) == uniqueMeshNames.end()) {
                    // Add unique mesh name to set
                    uniqueMeshNames.insert(meshName);
                    // Process the extracted mesh name
                    std::cout << "Mesh name: " << meshName << std::endl;
                }
            }
        }
    }
    std:cout << uniqueMeshNames.size() << std::endl;
    return uniqueMeshNames;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Segment::remove_ground()
    {
        // Point clouds
    PointCloudXYZ::Ptr object(new PointCloudXYZ);
    PointCloudXYZ::Ptr object_result(new PointCloudXYZ);
    PointCloudXYZ::Ptr scene(new PointCloudXYZ);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_difference(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::console::print_highlight("Loading SCENE cloud...\n");
    // GET SCENE CLOUD --------------

    sensor_msgs::msg::PointCloud2 msg;
    while (this->sharedMsg.data.empty()){
        rclcpp::sleep_for(std::chrono::seconds(1));
    }
    msg = this->sharedMsg;

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(this->sharedMsg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *scene);
    pcl::console::print_highlight("done\n");
    std::cout << "scene cloud: " << scene->size() << std::endl;

    Eigen::Matrix4f trans;
    trans<<  1.000, -0.000, -0.000,  0.000,
            -0.000, -0.499,  0.866, -0.500,
            -0.000, -0.866, -0.499,  0.750,
             0.000,  0.000,  0.000,  1.000;

    pcl::transformPointCloud(*scene, *cloud_difference, trans);

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_difference);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.01, 1.0);
    pass.filter (*cloud_filtered);

    pcl::console::print_highlight("done\n");
    sensor_msgs::msg::PointCloud2 cloud_output;
    cloud_filtered->width = cloud_filtered->size();
    std::cout << "aaaaaaaa: " << cloud_filtered->size() << std::endl;
    pcl::toROSMsg(*cloud_filtered, cloud_output);
    cloud_output.header.frame_id = "world";
    this->pub_->publish(cloud_output);
    pcl::console::print_highlight("done\n");
    return cloud_filtered;
    }