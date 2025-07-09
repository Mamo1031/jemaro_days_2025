# jemaro_days_2025

The rosbag LiDAR dataset and the lane paths are available here: https://uncloud.univ-nantes.fr/index.php/s/tkEwQcM7qGMj9wp 

# Gazebo
The Gazebo prius environment is based on https://github.com/mattborghi/osrf_car_demo

Launch the Gazebo simulator with the straight road world and the prius:
```
ros2 launch car_demo straight_road.launch.py
```

Convert twist messages to the prius control messages:
```
ros2 launch twist_to_prius_cmd.launch.py
```

Launch a basic pure pursuit path following algorithm:
```
ros2 launch car_control car_control.launch.py
```

Publish the path of the center of the lane and of the right and left sides of the road (only one time, use -l for multiple messages):
```
ros2 bag play path_prius
```
```
ros2 bag play path_right_prius --remap /path:=path_right
```
```
ros2 bag play path_left_prius --remap /path:=path_left
```

# LiDAR dataset

Launch rviz interface:
```
rviz2 -d jemaro.rviz
```

Publish the path of the center of the lane and of the right and left sides of the road (only one time, use -l for multiple messages):
```
ros2 bag play path_zoe --remap /ZOE3/path_follower/setPath:=path
```
```
ros2 bag play path_left_zoe --remap /ZOE3/path_follower/setPath:=path_left
```
```
ros2 bag play path_right_zoe --remap /ZOE3/path_follower/setPath:=path_right
```

Read the LiDAR dataset:
```
ros2 bag play rosbag2_2025_06_26-10_27_18
```

Launch the pointcloud downsampling node (based on https://github.com/LihanChen2004/pointcloud_downsampling):
```
ros2 launch pointcloud_downsampling pointcloud_downsampling.launch.py
```


# Cone Detector Node (3bag_cone_detector)

This node detects traffic cones from 3D LiDAR point cloud data using clustering and rule-based scoring.

---

## 🔧 Inputs

- `/ZOE3/os_node/points_downsampled`  
  (Type: `sensor_msgs/msg/PointCloud2`)  
  → Downsampled LiDAR point cloud

---

## 📤 Outputs

- `/filtered_points`  
  Point cloud after removing ground and region cropping

- `/No1_cluster`, `/No2_cluster`, ..., `/No5_cluster`  
  Top 5 clusters with highest cone-likeness scores

- `/confirmed_cone_cluster`  
  Cluster that is confirmed to be a cone (score > threshold)

- `/cone_poses`  
  `PoseArray` with the cone's 3D position (only published once)

---

## 🧠 How Cone Detection Works

1. **Crop Region**  
   Only keep points in the right-front area of the car.

2. **Ground Removal**  
   Remove points outside a height range (`z ∈ [-1.1, 0.5]`).

3. **Clustering**  
   Use Euclidean clustering to extract object candidates.

4. **Scoring Clusters**  
   For each cluster:
   - Calculate average intensity
   - Compute a score using:
     ```
     score = exp(- (intensity - 150)^2 / 2000)
     ```

5. **Cone Decision**
   - Keep clusters with score > 0.7
   - Publish the highest-scoring cluster as the detected cone
   - Also publish its 3D position (`/cone_poses`) only on first detection

---

## ✅ Notes

- The node uses hardcoded region and intensity thresholds for simplicity.
- Designed for use with high-reflection cones and real-time applications.
- Visualization can be done in Rviz via the published topics.



