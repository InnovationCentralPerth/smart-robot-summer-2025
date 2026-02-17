
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

def launch_setup(context, *args, **kwargs):
    name = LaunchConfiguration("name").perform(context)
    viz_enabled = LaunchConfiguration("viz").perform(context).lower() == 'true'
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    
    # ULTRALIGHT SETTINGS FOR RASPBERRY PI
    fps = "10"
    resolution = "640,480"
    profile_str = f"{resolution},{fps}"

    params_file = LaunchConfiguration("params_file")
    
    # SHARED PARAMETERS
    common_parameters = {
        "frame_id": name,
        "subscribe_rgb": True,
        "subscribe_depth": True,
        "approx_sync": True,
        "approx_sync_max_interval": 0.5,
        "queue_size": 100,
        "qos_image": 2, # Best Effort
        "qos_depth": 2,
        "qos_camera_info": 2,
    }

    # SLAM SPECIFIC
    slam_parameters = common_parameters.copy()
    slam_parameters.update({
        "subscribe_odom_info": True,
        "Rtabmap/DetectionRate": "1.0",       # Map update 1Hz (Very light)
        "Rtabmap/MemoryThr": "50",           # Even tighter memory limit
        "RGBD/OptimizeFromGraphEnd": "true",
        "Mem/ImagePreDecimation": "2",       # Process at 320x240
        "Mem/ImagePostDecimation": "2",
        "Vis/MaxFeatures": "400",            # Sufficient features
        "Vis/MinInliers": "12",
        "Vis/CorType": "1",                  # Optical Flow (Fastest tracking)
        "Vis/EstimationType": "1",           # 3D->2D PnP
        "Kp/DetectorStrategy": "6",          # GFTT/BRIEF (Fast)
        "Odom/Strategy": "1",                # Frame-to-Frame (Fastest)
        "Odom/GuessMotion": "true",          # Crucial for rotation prediction
        "Odom/ResetCountdown": "1",          # Auto-reset if lost (prevents long red screen)
        "Grid/CellSize": "0.1",
        "Grid/RangeMax": "4.0",
    })

    # GUI SPECIFIC (Re-enable views but keep it simple)
    viz_parameters = {
        "frame_id": name,
        "subscribe_depth": True,
        "subscribe_rgb": True,
        "subscribe_odom_info": False,
        "subscribe_map_data": True,
        "approx_sync": True,
        "approx_sync_max_interval": 0.5,
        "queue_size": 100,
    }

    remappings = [
        ("rgb/image", name + "/rgb/image_rect"),
        ("rgb/camera_info", name + "/rgb/camera_info"),
        ("depth/image", name + "/stereo/image_raw"),
        ("odom", "/odom"),
        ("mapData", "/mapData"),
    ]

    nodes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, "launch", "camera.launch.py")
            ),
            launch_arguments={
                "name": name, 
                "params_file": params_file,
                "rgb_camera.color_profile": profile_str,
                "depth_module.depth_profile": profile_str,
                "depth_module.infra_profile": profile_str,
                "pointcloud.enable": "false",
            }.items(),
        ),
        LoadComposableNodes(
            target_container=name + "_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="rtabmap_odom",
                    plugin="rtabmap_odom::RGBDOdometry",
                    name="rgbd_odometry",
                    parameters=[slam_parameters],
                    remappings=remappings,
                ),
                ComposableNode(
                    package="rtabmap_slam",
                    plugin="rtabmap_slam::CoreWrapper",
                    name="rtabmap",
                    parameters=[slam_parameters],
                    remappings=remappings,
                ),
            ],
        ),
    ]

    if viz_enabled:
        nodes.append(
            Node(
                package="rtabmap_viz",
                executable="rtabmap_viz",
                output="screen",
                parameters=[viz_parameters],
                remappings=remappings,
            )
        )

    return nodes

def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("viz", default_value="true"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(depthai_prefix, "config", "rgbd.yaml"),
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
