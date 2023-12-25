from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    world_name_arg = DeclareLaunchArgument(
        'world_name', default_value='default_world',
        description='Name of the world for simulation')
    dataset_type_arg = DeclareLaunchArgument(
        'dataset_type', default_value='training',
        description='Type of dataset (training, testing, etc.)')
    number_tries_arg = DeclareLaunchArgument(
        'number_tries', default_value='10',
        description='Number of tries for the simulation')

    return LaunchDescription([
        world_name_arg,
        dataset_type_arg,
        number_tries_arg,
        Node(
            package='simulation',
            executable='simulation_data_collection',
            name='simulation_data_collection_node',
            output='screen',
            parameters=[
                {'world_name': LaunchConfiguration('world_name')},
                {'dataset_type': LaunchConfiguration('dataset_type')},
                {'number_tries': LaunchConfiguration('number_tries')}
            ]
        )
    ])
