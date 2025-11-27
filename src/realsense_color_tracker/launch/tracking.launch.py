import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 配置分辨率 (如果要用 1280x720，请修改这里)
    WIDTH = '1280'
    HEIGHT = '720'
    FPS = '30'

    # 2. 获取 realsense2_camera 的安装路径
    rs_pkg_dir = get_package_share_directory('realsense2_camera')

    # 3. 定义 RealSense 的启动项 (相当于 ros2 launch realsense2_camera ...)
    # 我们在这里把所有需要的参数都配好
    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rs_pkg_dir, 'launch', 'rs_launch.py')),
        launch_arguments={
            'align_depth.enable': 'true',             # 开启对齐
            'filters': 'spatial,temporal,hole_filling', # 开启滤波器
            'rgb_camera.profile': f'{WIDTH}x{HEIGHT}x{FPS}',
            'depth_module.profile': f'{WIDTH}x{HEIGHT}x{FPS}',
        }.items()
    )

    # 4. 定义你的 Tracker 节点的启动项
    tracker_node = Node(
        package='realsense_color_tracker',
        executable='tracker',
        name='color_tracker',
        output='screen'
    )

    # 5. 返回描述，ROS 2 会按顺序启动它们
    return LaunchDescription([
        rs_launch,
        tracker_node
    ])