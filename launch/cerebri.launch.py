from os import environ

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

def launch_cerebri(context, *args, **kwargs):

    uart_shell = LaunchConfiguration('uart_shell').perform(context)
    gdb = LaunchConfiguration('gdb').perform(context)
    xterm = LaunchConfiguration('xterm').perform(context)
    vehicle = LaunchConfiguration('vehicle').perform(context)

    cmd = f"cerebri_{vehicle}_native_posix"
    xterm_cmd = 'xterm -fa Monospace -fs 12 -fg grey -bg black'

    if uart_shell != 'false':
        uart_args = f"--attach_uart_cmd='{xterm_cmd} -T cerebri-shell -e screen %s &'"
    else:
        uart_args = ""

    prefix = ''
    if xterm != 'false':
        prefix += f'{xterm_cmd} -T cerebri -e'

    if gdb != 'false':
        prefix += 'gdb --args'

    return [ExecuteProcess(
            cmd=[cmd, uart_args],
            name='cerebri',
            prefix=prefix,
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
            'gdb', default_value='false',
            description='Run in GDB Debugger'),
        DeclareLaunchArgument(
            'xterm', default_value='true',
            description='Launch in xterm'),
        DeclareLaunchArgument('vehicle',
            description='Vehicle to launch'),
        OpaqueFunction(function = launch_cerebri),
    ])
