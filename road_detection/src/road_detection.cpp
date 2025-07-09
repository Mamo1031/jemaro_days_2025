#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl_conversions/pcl_conversions.h> // for pcl::fromROSMsg, pcl::toROSMsg
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <queue>
#include <cmath>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

// PCL PointCloud2 to PCL::PointCloud<PointT> conversion is handled by pcl_conversions

class RoadDetectionNode : public rclcpp::Node {
public:
  RoadDetectionNode()
      : Node("road_detection_node"),
        resolution_(0.1),
        grid_size_(400), // 40m x 40m
        origin_(-static_cast<double>(grid_size_) / 2.0 * resolution_) // Calculate origin in meters
  {
    // Subscriber to PointCloud2 from /ground_points
    // Use SensorDataQoS for sensor data streams
    rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/ground_points", qos_profile,
        std::bind(&RoadDetectionNode::pointcloudCallback, this, std::placeholders::_1));

    // Publisher for OccupancyGrid
    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/free_space", 10);

    RCLCPP_INFO(this->get_logger(), "RoadDetectionNode initialized.");
    RCLCPP_INFO(this->get_logger(), "Resolution: %.2f m", resolution_);
    RCLCPP_INFO(this->get_logger(), "Grid Size: %d x %d", grid_size_, grid_size_);
    RCLCPP_INFO(this->get_logger(), "Origin X, Y: %.2f m, %.2f m", origin_, origin_);
  }

private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received PointCloud2 message.");

    // Convert ROS PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud_pcl);

    // Initialize grid with -1 (Unknown)
    // std::vector<std::vector<int8_t>> grid(grid_size_, std::vector<int8_t>(grid_size_, -1));
    // Flattened 1D vector for OccupancyGrid message
    std::vector<int8_t> grid_data(grid_size_ * grid_size_, -1);

    // Populate grid: Mark points from ground_points as 0 (Free candidate)
    for (const auto& point : cloud_pcl->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        continue; // Skip NaN points in x or y
      }

      int gx = static_cast<int>(std::round(point.x / resolution_) + grid_size_ / 2);
      int gy = static_cast<int>(std::round(point.y / resolution_) + grid_size_ / 2);

      if (gx >= 0 && gx < grid_size_ && gy >= 0 && gy < grid_size_) {
        grid_data[gy * grid_size_ + gx] = 0; // Mark as free candidate (0 in OccupancyGrid for free)
      }
    }

    // Flood-fill for reachable free space (BFS)
    std::vector<bool> visited(grid_size_ * grid_size_, false);
    std::queue<std::pair<int, int>> q;

    int center_gx = grid_size_ / 2;
    int center_gy = grid_size_ / 2;

    // Start flood-fill from the cell corresponding to (0,0) of the robot
    // Only if the center cell is a free candidate
    if (grid_data[center_gy * grid_size_ + center_gx] == 0) {
        q.push({center_gx, center_gy});
        visited[center_gy * grid_size_ + center_gx] = true;
    } else {
        // If the robot's center is not on a ground point, try a small area around it
        // This makes the flood fill more robust if center is slightly off ground.
        // For simplicity, we start directly from (0,0) if it's 0.
        // Otherwise, the free space might not be connected.
        // A more robust solution might involve checking neighbors around (0,0)
        // or ensure the initial cloud provides a point at (0,0) or nearby.
        RCLCPP_WARN(this->get_logger(), "Robot center is not on a ground point. Flood fill might not connect.");
    }


    int dx[] = {-1, 1, 0, 0};
    int dy[] = {0, 0, -1, 1};

    while (!q.empty()) {
      std::pair<int, int> current = q.front();
      q.pop();
      int x = current.first;
      int y = current.second;

      for (int i = 0; i < 4; ++i) {
        int nx = x + dx[i];
        int ny = y + dy[i];
        int n_idx = ny * grid_size_ + nx;

        if (nx >= 0 && nx < grid_size_ && ny >= 0 && ny < grid_size_) {
          if (!visited[n_idx] && grid_data[n_idx] == 0) { // If not visited and is a free candidate
            visited[n_idx] = true;
            q.push({nx, ny});
          }
        }
      }
    }

    // Create OccupancyGrid message
    nav_msgs::msg::OccupancyGrid og_msg;
    og_msg.header = msg->header;
    
    // Map info
    og_msg.info.resolution = resolution_;
    og_msg.info.width = grid_size_;
    og_msg.info.height = grid_size_;
    
    // Origin of the map (bottom-left corner of the grid in map frame)
    og_msg.info.origin.position.x = origin_;
    og_msg.info.origin.position.y = origin_;
    og_msg.info.origin.position.z = 0.0;
    og_msg.info.origin.orientation.x = 0.0;
    og_msg.info.origin.orientation.y = 0.0;
    og_msg.info.origin.orientation.z = 0.0;
    og_msg.info.origin.orientation.w = 1.0; // No rotation for the map origin itself

    og_msg.data.resize(grid_size_ * grid_size_);

    for (int y = 0; y < grid_size_; ++y) {
      for (int x = 0; x < grid_size_; ++x) {
        int idx = y * grid_size_ + x;
        if (visited[idx]) {
          og_msg.data[idx] = 0; // Free space (reachable from robot)
        } else if (grid_data[idx] == 0) { // Was ground point, but not reachable
          og_msg.data[idx] = 100; // Occupied (unreachable ground, could be obstacle or unconnectable ground)
        } else {
          og_msg.data[idx] = -1; // Unknown
        }
      }
    }

    publisher_->publish(og_msg);
    RCLCPP_INFO(this->get_logger(), "Published OccupancyGrid.");
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;

  double resolution_;
  int grid_size_;
  double origin_; // Map origin in meters
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoadDetectionNode>());
  rclcpp::shutdown();
  return 0;
}