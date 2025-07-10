from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='car_control',
            executable='pure_pursuit.py',
            name='pure_pursuit',
            output='screen',
            parameters=[{'max_steering': 1.22,
                'look_ahead_dist': 2.5,
                'kpp': 0.5,
                'angle_offset': 1.5707963,
                'plot_path': False}],
            remappings=[
                ('/odom', '/ZOE3/position/map_ekf_odometry'),
                ('/real_path', '/ZOE3/path_follower/setPath'),
                ('/path', '/ZOE3/path_follower/PassPointsPath')
            ]
        ),
    ])

