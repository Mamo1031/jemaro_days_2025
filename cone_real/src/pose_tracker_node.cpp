#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>




class PoseTrackerNode : public rclcpp::Node {
public:
  PoseTrackerNode() : Node("pose_tracker_node") {
    // Subscriber to cone pose from intensity filtering
    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/cone_pose_intensity", 10,
        std::bind(&PoseTrackerNode::poseCallback, this, std::placeholders::_1));
    
    // Subscriber to ekf odometry
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ZOE3/position/map_ekf_odometry", 10,
        std::bind(&PoseTrackerNode::odomCallback, this, std::placeholders::_1));

    // Publisher for dynamically tracked cone pose
    dyn_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/dyn_cone_pose", 10);

    // Publisher for rotated frame odom
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/odom_ekf", 10);


    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    //RCLCPP_INFO(this->get_logger(), "received odom");
    // Create a copy to modify and publish
    auto rotated_msg = *odom_msg;

    // // === Rotate POSITION ===
    // double x_in = odom_msg->pose.pose.position.x;
    // double y_in = odom_msg->pose.pose.position.y;
    // rotated_msg.pose.pose.position.x = -y_in;
    // rotated_msg.pose.pose.position.y =  x_in;

    // === Rotate ORIENTATION ===
    tf2::Quaternion original_q;
    tf2::fromMsg(odom_msg->pose.pose.orientation, original_q);

    tf2::Quaternion rot_z;
    rot_z.setRPY(0, 0, M_PI / 2);  // +90 degrees around Z

    tf2::Quaternion new_q =  original_q * rot_z;
    new_q.normalize();

    rotated_msg.pose.pose.orientation = tf2::toMsg(new_q);

    // === Rotate TWIST (linear velocity) ===
    double vx_in = odom_msg->twist.twist.linear.x;
    double vy_in = odom_msg->twist.twist.linear.y;
    rotated_msg.twist.twist.linear.x = -vy_in;
    rotated_msg.twist.twist.linear.y =  vx_in;
    rotated_msg.twist.twist.linear.z = odom_msg->twist.twist.linear.z;  // unchanged

    // === Angular velocity (z only matters in 2D, assume others zero or unchanged) ===
    rotated_msg.twist.twist.angular = odom_msg->twist.twist.angular;  // optional: rotate if needed, but often only z used

    // Publish rotated odometry
    //RCLCPP_INFO(this->get_logger(), "rotated odom");
    odom_pub_->publish(rotated_msg);



    // Pose tracking
    if (cone_detected_) {
        geometry_msgs::msg::PoseStamped global_pose_in_map;
        
        global_pose_in_map.header.frame_id = "map";
        global_pose_in_map.header.stamp = this->get_clock()->now();
        global_pose_in_map.pose = transformed_pose_;  

        try {
            geometry_msgs::msg::PoseStamped pose_in_car_frame =
                tf_buffer_->transform(global_pose_in_map, "base_link", tf2::durationFromSec(0.1));
            dyn_pose_pub_->publish(pose_in_car_frame);
            //RCLCPP_INFO(this->get_logger(), "Pose in car frame: x=%.2f, y=%.2f",
                        //pose_in_car_frame.pose.position.x, pose_in_car_frame.pose.position.y);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform pose to car frame: %s", ex.what());
        }

    }


    }


  void poseCallback(const geometry_msgs::msg::Pose::SharedPtr pose_msg) {
    geometry_msgs::msg::PoseStamped input_pose;
    input_pose.header.frame_id = "base_link"; 
    input_pose.header.stamp = this->get_clock()->now();
    input_pose.pose = *pose_msg;

    try {
        geometry_msgs::msg::PoseStamped transformed_pose_stamped =
            tf_buffer_->transform(input_pose, "map", tf2::durationFromSec(0.1));
        transformed_pose_ = transformed_pose_stamped.pose;
        cone_detected_ = true;
        RCLCPP_INFO(this->get_logger(), "Transformed pose: x=%.2f, y=%.2f, z=%.2f",
                    transformed_pose_stamped.pose.position.x,
                    transformed_pose_stamped.pose.position.y,
                    transformed_pose_stamped.pose.position.z);

    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform pose: %s", ex.what());
    }
  }


  geometry_msgs::msg::Pose transformed_pose_;
  bool cone_detected_ = false;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr dyn_pose_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
