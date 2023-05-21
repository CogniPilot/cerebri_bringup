from os import environ
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


ARGUMENTS = [
]

def generate_launch_description():

    cerebri_bin = environ.get('CEREBRI_BINARY')
    cerebri_cmd = f"{cerebri_bin}"
    cerebri = LaunchDescription([
        ExecuteProcess(
            name='cerebri',
            prefix="terminator -u -T cerebri -x gdbtui --args",
            cmd=cerebri_cmd.split(),
            output='screen',
            shell=True,
            on_exit=Shutdown()
            ),
    ])

    return LaunchDescription(ARGUMENTS + [
        cerebri,
    ])
