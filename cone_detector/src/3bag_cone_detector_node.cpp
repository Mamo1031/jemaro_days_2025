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

    rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();
    rclcpp::QoS qos_profile_pub(10);
    qos_profile_pub.reliable();

    // sub lidar downsampling
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/ZOE3/os_node/points_downsampled", qos_profile,
      std::bind(&ConeDetectorNode::scanCallback, this, std::placeholders::_1));

    // Publish points without ground
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/filtered_points", 10);

    // Pub top five scored clusters 
    for (int i = 1; i <= 5; ++i) {
    auto pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/No" + std::to_string(i) + "_cluster", 10);
    cluster_pubs_.push_back(pub);
    }

    //publish confirmation of cone
    confirmed_cone_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/confirmed_cone_cluster", 10);

    // publish final cone poses
    pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/cluster_intensity", qos_profile_pub);

}

private:
  int high_score_count_ = 0;
  bool detected_once_ = false;
  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {

    // Convert to PCL PointCloud with intensity
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*cloud_msg, *cloud);


    //step1: Create new cloud for front sector (plot as xyz)
    pcl::PointCloud<pcl::PointXYZI>::Ptr front_points(new pcl::PointCloud<pcl::PointXYZI>());
    for (const auto& point : cloud->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }

      // Rectangular box in front of the car (in base_link frame)
      // x is in front of car, y is left way of car. 
      // You can tune these numbers: road(0,100,-2.2,3.2)
      float x_min = 0.0; //back 0
      float x_max = 40.0; //front 100
      float y_min = -2.2; //right -2.2
      float y_max = 0;  //left 3.2
      if (point.x >= x_min && point.x <= x_max &&
      point.y >= y_min && point.y <= y_max) {
        front_points->points.push_back(point);
        // RCLCPP_INFO(this->get_logger(), "Box-filtered point: x=%.2f, y=%.2f", point.x, point.y);
      }
    }

    // Step 2: Remove ground published as filtered_points
    // Keep only points between z = -0.8 m and z = 0.5 m, -1.1
    auto cloud_no_ground = removeGround(front_points, -1.1, 0.5f);
    // Convert back to ROS message and publish
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_no_ground, output);
    output.header = cloud_msg->header;
    pub_->publish(output);




    // step3: making clusters (published number of the cluster are defined above)
    // performClustering(cloud_no_ground, tolerance, min_size, maz_size)
    auto clusters = performClustering(cloud_no_ground, 0.3, 3, 10);
    int cluster_id = 0;
    for (const auto& cluster : clusters) {
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cluster, centroid);
      float dist = std::sqrt(centroid[0]*centroid[0] + centroid[1]*centroid[1]);
      cluster_id++;
    }

    for (size_t i = 0; i < clusters.size(); ++i) {
      if (i >= cluster_pubs_.size()) break;
      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(*clusters[i], msg);
      msg.header = cloud_msg->header;  // use same frame_id and timestamp
      cluster_pubs_[i]->publish(msg);
    }

    // Step 4: choose cone cluster 0.7
    auto top_cones = getTop5ConeCandidates(clusters);
    for (const auto& [idx, score] : top_cones) {
      if (score < 0.7f) continue; 
      RCLCPP_INFO(this->get_logger(), "Published cone maker");
      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(*clusters[idx], msg);
      msg.header = cloud_msg->header;
      confirmed_cone_pub_->publish(msg);
    }

    if (!detected_once_  && !top_cones.empty() && top_cones[0].second >= 0.7f) {

      int cone_idx = top_cones[0].first;
      if (cone_idx < clusters.size()) {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*clusters[cone_idx], centroid);

        geometry_msgs::msg::Pose pose;
        pose.position.x = centroid[0];
        pose.position.y = centroid[1];
        pose.position.z = centroid[2];
        pose.orientation.w = 1.0;

        RCLCPP_INFO(this->get_logger(), "Published cone Pose");
        pose_pub_->publish(pose);

        detected_once_ = true;
        RCLCPP_INFO(this->get_logger(), "detected_once = true");
      } else {
        RCLCPP_WARN(this->get_logger(), "Invalid cone index.");
      }

      // geometry_msgs::msg::Pose pose;
      // pose.header = cloud_msg->header;

      // geometry_msgs::msg::Pose pose;
      // Eigen::Vector4f centroid;
      // pcl::compute3DCentroid(*clusters[top_cones[0].first], centroid);
      // pose.position.x = centroid[0];
      // pose.position.y = centroid[1];
      // pose.position.z = centroid[2];
      // pose.orientation.w = 1.0;
      // RCLCPP_INFO(this->get_logger(), "Published cone Pose");

      // pose.poses.push_back(pose);
      // pose_pub_->publish(pose);

      // detected_once_ = true;
      // RCLCPP_INFO(this->get_logger(), "detected_once = true");
    }

  }

  // Remove ground and high points AND keep only right-side points (y < 0)
  pcl::PointCloud<pcl::PointXYZI>::Ptr removeGround(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float min_z, float max_z) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground(new pcl::PointCloud<pcl::PointXYZI>());

    for (const auto& point : cloud->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }

      if (point.z >= min_z && point.z <= max_z && point.y < 0.0f) {
        no_ground->points.push_back(point);
      }
    }

    no_ground->width = no_ground->points.size();
    no_ground->height = 1;
    no_ground->is_dense = true;

    return no_ground;
  }




  // function for clustering (step3)
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
      // RCLCPP_INFO(this->get_logger(), "found indices");
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


  // function to choose the cone cluster step4
  std::vector<std::pair<int, float>> getTop5ConeCandidates(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clusters) {
    std::vector<std::pair<int, float>> scores;

    for (size_t i = 0; i < clusters.size(); ++i) {
      auto cluster = clusters[i];

      if (cluster->points.empty()) continue;
      
      //cone is only right side
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cluster, centroid);
      if (centroid[1] > -1.0f) continue;

      // Get bounding box
      pcl::PointXYZI min_pt, max_pt;
      pcl::getMinMax3D(*cluster, min_pt, max_pt);
      float height = max_pt.z - min_pt.z;
      float width  = max_pt.x - min_pt.x;
      float depth  = max_pt.y - min_pt.y;

      // intensity
      float total_intensity = 0.0f;
      for (const auto& pt : cluster->points) {
        total_intensity += pt.intensity;
      }
      float avg_intensity = total_intensity / static_cast<float>(cluster->points.size());

      // RCLCPP_INFO(this->get_logger(), "Cluster %zu: Avg Intensity = %.2f", i, avg_intensity);

      float intensity_score = std::exp(-std::pow(avg_intensity - 150.0f, 2) / 2000.0f);
      float height_score = std::exp(-std::pow(height - 0.6f, 2) / 0.1f);  // Tall shape
      float depth_score  = std::exp(-std::pow(depth - 0.1f, 2) / 0.02f);  // Narrow (like column)
      float width_score  = std::exp(-std::pow(width - 0.1f, 2) / 0.02f);  // Narrow (like column)

      // float total_score = 
      //   0.4f * intensity_score +
      //   0.2f * height_score +
      //   0.2f * width_score +
      //   0.2f * depth_score;

      float total_score = intensity_score;

      if (total_score > 0.7f) {
        std::cout << "\033[1;31m[DETECTED] Cone detected! Score = " << total_score << "\033[0m" << std::endl;
        std::cout << "Cluster " << i
                  << " | Intensity=" << avg_intensity
                  << " | height=" << height << ", width=" << width << ", depth=" << depth << std::endl;

        std::cout << " → Scores: intensity=" << intensity_score
                  << ", height=" << height_score
                  << ", width=" << width_score
                  << ", depth=" << depth_score << std::endl;
      }

      scores.emplace_back(i, total_score);
    }

    // Sort by score descending
    std::sort(scores.begin(), scores.end(), [](auto& a, auto& b) {
      return a.second > b.second;
    });

    if (scores.size() > 5) scores.resize(5);
    return scores;
  }

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> cluster_pubs_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr confirmed_cone_pub_;


};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
