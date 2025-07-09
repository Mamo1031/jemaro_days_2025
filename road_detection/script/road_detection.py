#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from collections import deque

class RoadDetectionNode(Node):
    def __init__(self):
        super().__init__('road_detection_node')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/ground_points',
            self.pointcloud_callback,
            10)

        self.publisher = self.create_publisher(
            OccupancyGrid,
            '/free_space',
            10)

        # パラメータ設定
        self.resolution = 0.1
        self.grid_size = 400  # 40m × 40m
        self.origin = -self.grid_size // 2

    def pointcloud_callback(self, msg):
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        grid = -np.ones((self.grid_size, self.grid_size), dtype=np.int8)
        for x, y, z in points:
            gx = int((x / self.resolution) + self.grid_size / 2)
            gy = int((y / self.resolution) + self.grid_size / 2)
            if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                grid[gy, gx] = 0  # Free候補として記録

        # Flood-fillで自車中心から走行可能領域だけ抽出
        visited = np.zeros_like(grid, dtype=bool)
        q = deque()
        cx = cy = self.grid_size // 2
        if grid[cy, cx] == 0:
            q.append((cx, cy))
            visited[cy, cx] = True

        while q:
            x, y = q.popleft()
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                    if not visited[ny, nx] and grid[ny, nx] == 0:
                        visited[ny, nx] = True
                        q.append((nx, ny))

        # OccupancyGridメッセージ生成
        og = OccupancyGrid()
        og.header = msg.header
        og.info.resolution = self.resolution
        og.info.width = self.grid_size
        og.info.height = self.grid_size
        og.info.origin.position.x = self.origin * self.resolution
        og.info.origin.position.y = self.origin * self.resolution
        og.info.origin.orientation.w = 1.0

        og.data = []
        for y in range(self.grid_size):
            for x in range(self.grid_size):
                if visited[y, x]:
                    og.data.append(0)      # Free
                elif grid[y, x] == 0:
                    og.data.append(100)    # 未連結の地面 (occupied)
                else:
                    og.data.append(-1)     # Unknown

        self.publisher.publish(og)


def main(args=None):
    rclpy.init(args=args)
    node = RoadDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
