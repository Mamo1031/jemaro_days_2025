#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <rclcpp/qos.hpp> 
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <cmath>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>




class ConeDetectorNode : public rclcpp::Node {
public:
  ConeDetectorNode()
      : Node("cone_detector_node") {
    // Publisher
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/filtered_points", 10);

    ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/ground_points", 10);

    // Subscriber to PointCloud2
    rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/prius/center_laser/scan", qos_profile,
        std::bind(&ConeDetectorNode::scanCallback, this, std::placeholders::_1));

    // publish cluster.
    for (int i = 0; i < 5; ++i) {  // Max 50 clusters (adjust as needed)
      auto pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/cluster_" + std::to_string(i), 10);
      cluster_pubs_.push_back(pub);
    }

    // publish final cone poses
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/cone_poses", 10);

}

private:
  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    //RCLCPP_INFO(this->get_logger(), "Received PointCloud2 message");

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
      if (std::abs(angle) < 20.0 * M_PI / 180.0 / 2.0) {
        front_points->points.push_back(point);
        //RCLCPP_INFO(this->get_logger(), "Front point: x=%.2f, y=%.2f, z=%.2f", point.x, point.y, point.z);
      }
    }

    // Step 2: Remove ground
    // auto cloud_no_ground = removeGround(front_points, -1.7f);
    // Keep only points between z = -1.5 m and z = 1.0 m
    auto cloud_no_ground = removeGround(front_points, -1.6f, 0.5f);
    auto ground_points   = removeGround(front_points, -1.6f, 0.10f);



    // cluster
    auto clusters = performClustering(cloud_no_ground, 2, 3, 100);
    int cluster_id = 0;
    for (const auto& cluster : clusters) {
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cluster, centroid);
      float dist = std::sqrt(centroid[0]*centroid[0] + centroid[1]*centroid[1]);
      
      RCLCPP_INFO(this->get_logger(),
          "Cluster %d: size=%lu, centroid=(%.2f, %.2f, %.2f), dist=%.2f m",
          cluster_id, cluster->points.size(),
          centroid[0], centroid[1], centroid[2], dist);

      cluster_id++;
    }


    // Step 3: choose cone cluster
    auto top_cones = getTop5ConeCandidates(clusters);
    int pub_id = 0;
    for (const auto& [idx, score] : top_cones) {
      if (pub_id >= static_cast<int>(cluster_pubs_.size())) break;

      RCLCPP_INFO(this->get_logger(), "TOP CONE CANDIDATE: Cluster %d with score %.3f", idx, score);
        
      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(*clusters[idx], msg);
      msg.header = cloud_msg->header;
      cluster_pubs_[pub_id]->publish(msg); 
      pub_id++;
    }


    // Step 4: From top 5, select 2 closest to the car
    std::vector<std::tuple<int, float, float>> scored_clusters;  // (cluster index, score, distance)

    for (const auto& [idx, score] : top_cones) {
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*clusters[idx], centroid);
      float dist = std::sqrt(centroid[0] * centroid[0] + centroid[1] * centroid[1]);
      scored_clusters.emplace_back(idx, score, dist);
    }

    // Sort by distance (ascending)
    std::sort(scored_clusters.begin(), scored_clusters.end(),
              [](const auto& a, const auto& b) {
                return std::get<2>(a) < std::get<2>(b);  // Compare by distance
              });

    // Retain only the closest 2 clusters
    if (scored_clusters.size() > 2) scored_clusters.resize(2);

    // publish top 2
    for (size_t i = 0; i < scored_clusters.size(); ++i) {
      int idx = std::get<0>(scored_clusters[i]);
      float score = std::get<1>(scored_clusters[i]);
      float dist = std::get<2>(scored_clusters[i]);

      RCLCPP_INFO(this->get_logger(), "CLOSEST CONE: Cluster %d | score=%.3f | dist=%.2f m", idx, score, dist);

      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(*clusters[idx], msg);
      msg.header = cloud_msg->header;
      cluster_pubs_[i]->publish(msg);  // Overwrite pubs 0 and 1
    }


    // Step 5: Publish centroids of top 2 closest clusters
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header = cloud_msg->header;

    for (const auto& [idx, score, dist] : scored_clusters) {
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*clusters[idx], centroid);

      geometry_msgs::msg::Pose pose;
      pose.position.x = centroid[1];
      pose.position.y = centroid[0];
      pose.position.z = centroid[2];
      pose.orientation.w = 1.0;  // No rotation

      pose_array.poses.push_back(pose);
    }

    pose_pub_->publish(pose_array);

    sensor_msgs::msg::PointCloud2 ground_msg;
    pcl::toROSMsg(*ground_points, ground_msg);
    ground_msg.header = cloud_msg->header;
    ground_pub_->publish(ground_msg);


    // Convert back to ROS message and publish
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_no_ground, output);
    output.header = cloud_msg->header;
    pub_->publish(output);
  }


  // function for clustering
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> performClustering(
      pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
      float tolerance,
      int min_size,
      int max_size) {

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(input_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
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
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>());
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


  // Remove ground and high points by keeping z in [min_z, max_z]
  pcl::PointCloud<pcl::PointXYZ>::Ptr removeGround(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float min_z, float max_z) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground(new pcl::PointCloud<pcl::PointXYZ>());

    for (const auto& point : cloud->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }

      if (point.z >= min_z && point.z <= max_z) {
        no_ground->points.push_back(point);
      }
    }

    no_ground->width = no_ground->points.size();
    no_ground->height = 1;
    no_ground->is_dense = true;

    return no_ground;
  }



  // function to choose the cone cluster
  std::vector<std::pair<int, float>> getTop5ConeCandidates(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters) {
    std::vector<std::pair<int, float>> scores;

    for (size_t i = 0; i < clusters.size(); ++i) {
      auto cluster = clusters[i];

      if (cluster->points.empty()) continue;

      // Get bounding box
      pcl::PointXYZ min_pt, max_pt;
      pcl::getMinMax3D(*cluster, min_pt, max_pt);

      float height = max_pt.z - min_pt.z;
      float width  = max_pt.x - min_pt.x;
      float depth  = max_pt.y - min_pt.y;

      float height_score = std::exp(-std::pow(height - 0.6f, 2) / 0.1f);  // Tall shape
      float depth_score  = std::exp(-std::pow(depth - 0.1f, 2) / 0.02f);  // Narrow (like column)
      float width_score  = std::exp(-std::pow(width - 0.1f, 2) / 0.02f);  // Narrow (like column)

      // Final score: shape only
      float total_score = 0.33f * height_score + 0.33f * width_score + 0.33f * depth_score;


      scores.emplace_back(i, total_score);
    }

    // Sort by score descending
    std::sort(scores.begin(), scores.end(), [](auto& a, auto& b) {
      return a.second > b.second;
    });

    // Take top 3
    if (scores.size() > 5) scores.resize(5);

    return scores;
  }




  // // Cone shape detection
  // bool isConeShaped(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster,
  //                   float min_h, float max_h, float min_base, float max_base) {
  //   pcl::PointXYZ min_pt, max_pt;
  //   pcl::getMinMax3D(*cluster, min_pt, max_pt);
  //   float height = max_pt.z - min_pt.z;
  //   float base = std::max(max_pt.x - min_pt.x, max_pt.y - min_pt.y);
  //   return (height >= min_h && height <= max_h && base >= min_base && base <= max_base);
  //}

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> cluster_pubs_;

};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeDetectorNode>());
  rclcpp::shutdown();
  return 0;
}

