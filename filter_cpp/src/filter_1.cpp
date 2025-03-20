#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/io.h>

class PCDListener : public rclcpp::Node {
public:
    PCDListener() : Node("pcd_listener") {
        // Define filter parameters
        max_y_ = 0.5;
        max_x_ = 0.5;
        min_z_ = 3.0;
        voxel_size_ = 0.01;
        distance_threshold_ = 5.0;
        publish_rate_ = 0.1;  // 10 Hz

        // QoS Profile for sensor data
        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        
        // Subscriber for point cloud data
        pcd_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera/depth/color/points", qos_profile,
            std::bind(&PCDListener::listener_callback, this, std::placeholders::_1)
        );

        // Publisher for filtered point cloud
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_points", qos_profile);

        // Timer for periodic publishing
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(publish_rate_ * 1000)),
            std::bind(&PCDListener::publish_points, this)
        );

        RCLCPP_INFO(this->get_logger(), "PCD Listener Node Initialized.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr latest_filtered_cloud_;
    std::mutex cloud_mutex_;

    // Filter parameters
    double max_y_, max_x_, min_z_, voxel_size_, distance_threshold_, publish_rate_;

    void listener_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert ROS 2 message to PCL format
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        // Process the point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = process_points(cloud);

        if (!filtered_cloud->empty()) {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            latest_filtered_cloud_ = filtered_cloud;
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr process_points(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        if (cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty point cloud.");
            return cloud;
        }

        // Distance filtering
        pcl::PointCloud<pcl::PointXYZ>::Ptr distance_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        for (auto &point : cloud->points) {
            double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            if (distance < distance_threshold_) {
                distance_filtered->points.push_back(point);
            }
        }
        distance_filtered->width = distance_filtered->points.size();
        distance_filtered->height = 1;
        distance_filtered->is_dense = false;

        // Spatial filtering (bounding box)
        pcl::PointCloud<pcl::PointXYZ>::Ptr spatial_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        for (auto &point : distance_filtered->points) {
            if (std::abs(point.y) <= max_y_ && std::abs(point.x) <= max_x_ && point.z < min_z_) {
                spatial_filtered->points.push_back(point);
            }
        }
        spatial_filtered->width = spatial_filtered->points.size();
        spatial_filtered->height = 1;
        spatial_filtered->is_dense = false;

        // Downsampling using Voxel Grid Filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(spatial_filtered);
        voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        voxel_filter.filter(*downsampled);

        return downsampled;
    }

    void publish_points() {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        if (latest_filtered_cloud_ && !latest_filtered_cloud_->empty()) {
            // Convert to ROS 2 message
            sensor_msgs::msg::PointCloud2 output_msg;
            pcl::toROSMsg(*latest_filtered_cloud_, output_msg);
            output_msg.header.frame_id = "camera_link";
            output_msg.header.stamp = this->now();

            publisher_->publish(output_msg);
            RCLCPP_INFO(this->get_logger(), "Published filtered point cloud with %zu points", latest_filtered_cloud_->points.size());
        }
    }
};

// Main function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCDListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
