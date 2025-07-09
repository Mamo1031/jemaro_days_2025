#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <cmath>
#include <rclcpp/qos.hpp> 
#include <geometry_msgs/msg/pose.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

class ConeDetectorNode : public rclcpp::Node {
public:
  ConeDetectorNode()
      : Node("cone_detector_node") {
    // Publisher
    //rclcpp::QoS qos_profile_pub = rclcpp::SensorDataQoS();
    rclcpp::QoS qos_profile_pub(10);
    qos_profile_pub.reliable();
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/useful_points", qos_profile_pub);

    // Subscriber to PointCloud2
    rclcpp::QoS qos_profile_sub(10);
    qos_profile_sub.reliable();

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/ZOE3/os_node/points", qos_profile_sub,
        std::bind(&ConeDetectorNode::scanCallback, this, std::placeholders::_1));
  
    // publish final cone poses
    pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/cone_pose_intensity", 10);
    

    // publish cluster.
    cluster_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/cluster_intensity", qos_profile_pub);

  }

private:
  
  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    RCLCPP_INFO(this->get_logger(), "Frame ID: %s", cloud_msg->header.frame_id.c_str());
    
    // Convert to PCL PointCloud with intensity
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Create filtered cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());

    const double angle_threshold_rad = 5.0 * M_PI / 180.0 / 2.0; 
    const float intensity_threshold = 1300.0; 

    //RCLCPP_INFO(this->get_logger(), "received pointcloud");

    for (const auto& point : cloud->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }

      double angle = std::atan2(point.y, point.x);
      if (std::abs(angle) < angle_threshold_rad && point.intensity > intensity_threshold) {
        if (point.x == 0.0f && point.y == 0.0f && point.z == 0.0f) {
          continue;
        }

        filtered->points.push_back(point);
        
        //RCLCPP_INFO(this->get_logger(), "Filtered point: x=%.2f, y=%.2f, z=%.2f, I=%.2f",
                    //point.x, point.y, point.z, point.intensity);
      }
    }


    // cluster
    auto clusters = performClustering(filtered, 2, 5, 100);
    int cluster_id = 0;
    for (const auto& cluster : clusters) {
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cluster, centroid);
      float dist = std::sqrt(centroid[0]*centroid[0] + centroid[1]*centroid[1]);
      
      RCLCPP_INFO(this->get_logger(),
          "Cluster %d: size=%lu, centroid=(%.2f, %.2f, %.2f), dist=%.2f m",
          cluster_id, cluster->points.size(),
          centroid[0], centroid[1], centroid[2], dist);
      
      // publish cone pose
      geometry_msgs::msg::Pose pose;
      pose.position.x = centroid[1];
      pose.position.y = centroid[0];
      pose.position.z = centroid[2];
      pose.orientation.w = 1.0;  // No rotation
      pose_pub_->publish(pose);
        
      cluster_id++;
    }

    // publish cluster
    if (!clusters.empty()) {
      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(*clusters[0], msg);
      msg.header = cloud_msg->header;
      cluster_pub_->publish(msg);
    }


    // Convert back to ROS message and publish
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*filtered, output);
    output.header = cloud_msg->header;
    pub_->publish(output);
  }


  //function for clustering
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> performClustering(
      pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,
      float tolerance,
      int min_size,
      int max_size) {

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    tree->setInputCloud(input_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(tolerance);      // max dist points in the same cluster
    ec.setMinClusterSize(min_size);         // min num of points in a cluster
    ec.setMaxClusterSize(max_size);         // max num of points in a cluster
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);
    RCLCPP_INFO(this->get_logger(), "Clustering input size: %lu", input_cloud->points.size());
    ec.extract(cluster_indices);
    RCLCPP_INFO(this->get_logger(), "Number of clusters found: %lu", cluster_indices.size());


    for (const auto& indices : cluster_indices) {
      RCLCPP_INFO(this->get_logger(), "found indices");
      pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>());
      for (int idx : indices.indices) {
        cluster->points.push_back(input_cloud->points[idx]);
      }
      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;
      clusters.push_back(cluster);
    }
    return clusters;
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeDetectorNode>());
  rclcpp::shutdown();
  return 0;
}

