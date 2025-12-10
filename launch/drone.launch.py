import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'autonomous_drone'
    pkg_share = get_package_share_directory(package_name)
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'mapping.rviz')
    
    home_dir = os.environ.get('HOME')
    px4_dir = os.path.join(home_dir, 'px4_ros2_ws', 'PX4-Autopilot')

    sim_time_config = {'use_sim_time': True}

    # 1. Start MicroXRCEAgent
    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen'
    )

    # 2. Start PX4 SITL
    px4_sitl = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gz_x500_depth'],
        cwd=px4_dir,
        output='screen'
    )

    # 3. Start ROS Gz Bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' 
        ],
        output='screen'
    )

    # 4. Start Control Nodes
    depth_avoidance_node = Node(
        package=package_name,
        executable='depth_avoidance',
        name='depth_avoidance',
        parameters=[sim_time_config],
        output='screen'
    )

    mapping_node = Node(
        package=package_name,
        executable='mapping_node',
        name='mapping_node',
        parameters=[sim_time_config],
        output='screen'
    )

    # --- NEW: ADDED PLANNER NODE ---
    path_planner_node = Node(
        package=package_name,
        executable='path_planner',  # Make sure you added this to setup.py!
        name='path_planner',
        parameters=[sim_time_config],
        output='screen'
    )
    # -------------------------------

    # 5. Start RViz & TF
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[sim_time_config],
        output='screen'
    )
    
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'x500_depth_0/lidar_link_custom/lidar_custom'],
        parameters=[sim_time_config],
        output='screen'
    )

    return LaunchDescription([
        # Phase 1: Infrastructure
        micro_xrce_agent,
        px4_sitl,

        # Phase 2: Bridge (Wait 5s)
        TimerAction(
            period=5.0,
            actions=[ros_gz_bridge]
        ),

        # Phase 3: Mission Control (Wait 10s)
        TimerAction(
            period=10.0,
            actions=[depth_avoidance_node, mapping_node, path_planner_node] # Added planner here
        ),

        # Phase 4: Visualization (Wait 12s)
        TimerAction(
            period=12.0,
            actions=[rviz_node, static_tf]
        )
    ])