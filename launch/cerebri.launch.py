from os import environ

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

def launch_cerebri(context, *args, **kwargs):

    uart_shell = LaunchConfiguration('uart_shell').perform(context)
    debugger = LaunchConfiguration('debugger').perform(context)
    vehicle = LaunchConfiguration('vehicle').perform(context)

    cmd = f"cerebri_{vehicle}_native_posix"
    xterm_cmd = 'xterm -fa Monospace -fs 12 -fg grey -bg black'

    if uart_shell != 'false':
        cmd_args = f"--attach_uart_cmd='{xterm_cmd} -T cerebri-shell -e screen %s &'"
    else:
        cmd_args = ""

    if debugger != 'false':
        debug_prefix = f'{xterm_cmd} -T cerebri -e gdb -ex run --args'
    else:
        debug_prefix = f'{xterm_cmd} -T cerebri -e'

    return [ExecuteProcess(
            cmd=[cmd, cmd_args],
            name='cerebri',
            prefix=debug_prefix,
            output='screen',
            shell=True,
            on_exit=Shutdown()
            )]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'uart_shell', default_value='false',
            choices=['true', 'false'],
            description='Run with uart shell.'),
        DeclareLaunchArgument(
            'debugger', default_value='false',
            description='Run in Debugger'),
        DeclareLaunchArgument('vehicle',
            description='Vehicle to launch'),
        OpaqueFunction(function = launch_cerebri),
    ])
