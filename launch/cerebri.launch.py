from os import environ

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

def launch_cerebri(context, *args, **kwargs):

    uart_shell = LaunchConfiguration('uart_shell').perform(context)
    debugger = LaunchConfiguration('debugger').perform(context)

    cerebri_bin = environ.get('CEREBRI_BINARY')
    cmd = f"{cerebri_bin}"

    if uart_shell:
        cmd_args = "--attach_uart"
    else:
        cmd_args = ""

    if debugger != 'false':
        debug_prefix = 'x-terminal-emulator -e gdb -ex run --args'
    else:
        debug_prefix = 'x-terminal-emulator -e'

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
            'uart_shell', default_value='true',
            choices=['true', 'false'],
            description='Run with uart shell.'),
        DeclareLaunchArgument(
            'debugger', default_value='false',
            description='Run in Debugger'),
        OpaqueFunction(function = launch_cerebri),
    ])
