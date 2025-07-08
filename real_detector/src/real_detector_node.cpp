#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <cmath>
#include <rclcpp/qos.hpp> 

class ConeDetectorNode : public rclcpp::Node {
public:
  ConeDetectorNode()
      : Node("cone_detector_node") {
    // Publisher
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/filtered_points", 10);

    // Subscriber to PointCloud2
    rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/prius/center_laser/scan", qos_profile,
        std::bind(&ConeDetectorNode::scanCallback, this, std::placeholders::_1));
  }

private:
  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    RCLCPP_INFO(this->get_logger(), "Received PointCloud2 message");

    // Convert to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Create new cloud for front sector
    pcl::PointCloud<pcl::PointXYZ>::Ptr front_points(new pcl::PointCloud<pcl::PointXYZ>());

    for (const auto& point : cloud->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }

      double angle = std::atan2(point.y, point.x);  // [-π, π]
      if (std::abs(angle) < 5.0 * M_PI / 180.0 / 2.0) {
        front_points->points.push_back(point);
        RCLCPP_INFO(this->get_logger(), "Front point: x=%.2f, y=%.2f, z=%.2f", point.x, point.y, point.z);
      }
    }

    // Convert back to ROS message and publish
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*front_points, output);
    output.header = cloud_msg->header;
    pub_->publish(output);
  }


  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
