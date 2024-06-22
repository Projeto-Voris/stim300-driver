from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'device_name',
            default_value='/dev/ttyUSB0',
            description='Device name'
        ),
        DeclareLaunchArgument(
            'standard_deviation_of_gyro',
            default_value='0.0004',
            description='Standard deviation of gyro'
        ),
        DeclareLaunchArgument(
            'standard_deviation_of_acc',
            default_value='0.004',
            description='Standard deviation of accelerometer'
        ),
        DeclareLaunchArgument(
            'sample_rate',
            default_value='125',
            description='Sample rate'
        ),
        Node(
            package='driver_stim300',
            executable='stim300_driver_node',
            name='stim300driver',
            parameters=[{
                'device_name': LaunchConfiguration('device_name'),
                'standard_deviation_of_gyro': LaunchConfiguration('standard_deviation_of_gyro'),
                'standard_deviation_of_acc': LaunchConfiguration('standard_deviation_of_acc'),
                'sample_rate': LaunchConfiguration('sample_rate')
            }]
        )
    ])

