from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('vrx_gz'),
                'launch',
                'competition.launch.py'
            ]),
            launch_arguments={
                'urdf': '/home/jfetko/bard/vlm-cbf/src/odo_boat/wamv_with_odo.urdf',
                'world': 'sydney_regatta',
            }.items()
        ),
        ExecuteProcess(
            cmd=[['ros2 run ros_gz_bridge parameter_bridge "/model/wamv/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry"']],
            shell=True
        ),
    ])