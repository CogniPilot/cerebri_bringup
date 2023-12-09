from os import environ

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

def launch_cerebri(context, *args, **kwargs):

    uart = LaunchConfiguration('uart').perform(context)
    gdb = LaunchConfiguration('gdb').perform(context)
    vehicle = LaunchConfiguration('vehicle').perform(context)
    xterm = LaunchConfiguration('xterm').perform(context)

    cerebri_exe = [f"cerebri_{vehicle}_native_posix"]
    xterm_cmd = 'xterm -fa Monospace -fs 12 -fg grey -bg black -T cerebri -e '

    if uart != 'false':
        uart_args = ["--attach_uart"]
    else:
        uart_args = []

    prefix = ''

    if xterm != 'false':
        prefix += xterm_cmd 

    if gdb != 'false':
        prefix += 'gdb --args '

    cmd = cerebri_exe + uart_args
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
            'xterm', default_value='true',
            choices=['true', 'false'],
            description='Run with xterm.'),
        DeclareLaunchArgument(
            'uart', default_value='false',
            choices=['true', 'false'],
            description='Run with uart shell.'),
        DeclareLaunchArgument(
            'gdb', default_value='false',
            description='Run in GDB Debugger'),
        DeclareLaunchArgument('vehicle',
            description='Vehicle to launch'),
        OpaqueFunction(function = launch_cerebri),
    ])
