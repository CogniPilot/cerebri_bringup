from os import environ

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

def launch_cerebri(context, *args, **kwargs):

    gdb = LaunchConfiguration('gdb').perform(context)
    vehicle = LaunchConfiguration('vehicle').perform(context)

    cerebri_exe = [f"cerebri_{vehicle}_native_sim", "-attach_uart"]

    prefix = ''

    if gdb != 'false':
        prefix += 'xterm -title gdb_cerebri -fg grey -bg black -e gdb --args '

    cmd = cerebri_exe
    # print(f'executing cmd: {cmd}')
    # print(f'prefix: {prefix}')
    return [ExecuteProcess(
            cmd = cmd,
            name='cerebri',
            prefix=prefix,
            output='screen',
            sigterm_timeout='1',
            sigkill_timeout='1',
            on_exit=Shutdown()
            )]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gdb', default_value='false',
            description='Run in GDB Debugger'),
        DeclareLaunchArgument('vehicle',
            description='Vehicle to launch'),
        OpaqueFunction(function = launch_cerebri),
    ])
