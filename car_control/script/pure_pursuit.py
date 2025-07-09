#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
#from prius_msgs.msg import Control
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseArray


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
	
        self.cones = []  
        self.obstacles = []
        self.cone_clear_count = 0
	
        # Subscriptions and Publishers
        self.path_sub = self.create_subscription(Path, '/path', self.path_callback, 10)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cone_sub = self.create_subscription(PoseArray, '/cone_poses', self.cone_callback, 10)
        #self.cmd_prius = self.create_publisher(Control, '/prius/control', 10)
        
        self.real_path_pub = self.create_publisher(Path, '/real_path', 10)
        
        # Parameters and state
        self.timer = self.create_timer(0.1, self.control_loop)
        self.path = np.array([])
        self.pose = None
        self.yaw = 0.0
        self.speed = 0.0
        
        self.declare_parameter('max_steering', 1.22)
        self.max_steering = self.get_parameter('max_steering').get_parameter_value().double_value
        
        self.declare_parameter('look_ahead_dist', 2.5)
        self.look_ahead_dist = self.get_parameter('look_ahead_dist').get_parameter_value().double_value
        
        self.declare_parameter('avoidance_thresh', 12.5)
        self.avoidance_thresh = self.get_parameter('avoidance_thresh').get_parameter_value().double_value
        
        self.declare_parameter('security_dist', -2.75)
        self.security_dist = self.get_parameter('security_dist').get_parameter_value().double_value
        
        self.declare_parameter('kpp', 0.5)
        self.kpp = self.get_parameter('kpp').get_parameter_value().double_value
        
        self.declare_parameter('angle_offset', 1.5707963)
        self.angle_offset = self.get_parameter('angle_offset').get_parameter_value().double_value
        
        self.declare_parameter('plot_path', False)
        self.plot_path = self.get_parameter('plot_path').get_parameter_value().bool_value
        self.real_path = []
        
        self.trajectory_msg = Path()
        self.trajectory_msg.header.frame_id = "world"

    def path_callback(self, msg: Path):
        self.path = np.array([[pose.pose.position.x, pose.pose.position.y] for pose in msg.poses])
        
        
    def odom_callback(self, msg: Odometry):
        self.pose = msg.pose.pose
        q = self.pose.orientation
        self.yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.speed = np.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    def cone_callback(self, msg: PoseArray):
        if self.pose is None:
            return

        theta = self.yaw - self.angle_offset
        cos_t, sin_t = np.cos(theta), np.sin(theta)

        px_map, py_map = self.pose.position.x, self.pose.position.y

        for pose in msg.poses:
            xr_left  = pose.position.x
            yr_front = pose.position.y

            x_front = yr_front           
            y_left  = xr_left            

            xw =  x_front * cos_t - y_left * sin_t + px_map   
            yw =  x_front * sin_t + y_left * cos_t + py_map   

            if any(np.hypot(xw - xo, yw - yo) < 1.0 for xo, yo in self.obstacles):
                continue

            if len(self.obstacles) == 0:
                self.obstacles.append((xw, yw))
                self.get_logger().info(f"ADD cone  -> ({xw:.2f}, {yw:.2f})")

    def control_loop(self):
        if self.path.size == 0 or self.pose is None:
            return
            
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "world"
        pose_stamped.pose = self.pose
        self.trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        self.trajectory_msg.poses.append(pose_stamped)
        self.real_path_pub.publish(self.trajectory_msg)
        self.get_logger().info('Publishing real_path with {} poses'.format(len(self.trajectory_msg.poses))) ## magic line

            
        if self.plot_path :
            self.real_path.append([self.pose.position.x, self.pose.position.y])
        position = np.array([self.pose.position.x, self.pose.position.y])
        # Find closest point on path
        dists = np.linalg.norm(self.path - position, axis=1)
        closest_id = np.argmin(dists)

        # Find the lookahead point
        #TODO: definition of modified lookahead point
        lookahead_point = None
        for i in range(closest_id, len(self.path)):
            dist = np.linalg.norm(self.path[i] - position)
            if dist >= self.look_ahead_dist:
                desired_lookahead_point = self.path[i]
                break

        if self.obstacles:
            obs_array = np.array(self.obstacles)
            # dist_obs = np.linalg.norm(obs_array - position, axis=1)
            rel_vectors = obs_array - position
            heading_vector = np.array([np.cos(self.yaw - self.angle_offset), np.sin(self.yaw - self.angle_offset)])
            forward_mask = np.dot(rel_vectors, heading_vector) > 0  # 前方にある障害物のみ
            dist_obs = np.linalg.norm(rel_vectors, axis=1)
            dist_obs = dist_obs[forward_mask] if np.any(forward_mask) else np.array([np.inf])
        else:
            dist_obs = np.array([np.inf])
            
        in_danger = any(d_obs < self.avoidance_thresh for d_obs in dist_obs)

        if (not in_danger) and len(self.obstacles) == 1:
            self.cone_clear_count += 1
            self.obstacles.clear()
            self.get_logger().info(f"Cone #{self.cone_clear_count} cleared, waiting for next")

        if in_danger:
            lookahead_point = desired_lookahead_point + np.array([self.security_dist,0])
        else:
       	    lookahead_point = desired_lookahead_point



        if lookahead_point is None:
            self.get_logger().warn('No lookahead point found.')
            # Publish command
            cmd = Twist()
            cmd.linear.x = 0.0  # Break
            self.cmd_pub.publish(cmd)
            if self.plot_path :
                self.get_logger().info('End of path reached')
                plt.plot(self.path[:,0], self.path[:,1], label='target path', color='black')
                self.real_path = np.array(self.real_path)
                plt.plot(self.real_path[:,0], self.real_path[:,1], label='real path', color='red')
                plt.title('Path')
                plt.xlabel('x')
                plt.ylabel('y')
                #plt.axis('equal')
                plt.grid(True)
                plt.legend()
                plt.show(block=False)
                plt.pause(0.001)
                self.plot_path = False
                return
            


        path_pos = self.path[closest_id]
        dx = lookahead_point[0] - position[0]
        dy = lookahead_point[1] - position[1]
        
        alpha = np.arctan2(dy, dx) - (self.yaw - self.angle_offset)
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))

        # Pure Pursuit steering formula
        steer = np.arctan2(2 * self.look_ahead_dist * np.sin(alpha), self.kpp * self.speed + 1e-5)
        steer = np.clip(steer, -self.max_steering, self.max_steering)

        # Publish command
        cmd = Twist()
        cmd.linear.x = 1.0  # constant speed
        cmd.angular.z = steer
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

