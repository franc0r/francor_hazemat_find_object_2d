import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
   #/camera/manipulator/image_raw
    # Hole den Pfad zum Paket
    package_name = 'francor_hazemat_find_object_2d'  # Ändere den Namen deines Pakets
    package_dir = get_package_share_directory(package_name)
    return LaunchDescription([

        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'),

        # Launch arguments
        DeclareLaunchArgument('gui', default_value='false', description='Launch GUI.'),
        DeclareLaunchArgument('image_topic', default_value='/camera/manipulator/image_raw', description='Image topic to subscribe to.'),
        DeclareLaunchArgument('objects_path', default_value=os.path.join(package_dir, 'config', 'objects'), description='Directory containing objects to load on initialization.'),
        DeclareLaunchArgument('settings_path', default_value=os.path.join(package_dir, 'config', 'find_object_2d.ini'), description='Config file.'),

        # Nodes to launch
        Node(
            namespace='find_object',
            package='find_object_2d', executable='find_object_2d', output='screen',
            parameters=[{
              'gui':LaunchConfiguration('gui'),
              'objects_path':LaunchConfiguration('objects_path'),
              'settings_path':LaunchConfiguration('settings_path')
            }],
            remappings=[
                ('image', LaunchConfiguration('image_topic'))]), 
        
        Node(
            namespace='find_object',
            package='francor_hazemat_find_object_2d', executable='object_overlay_node', output='screen',
            remappings=[
                ('image', LaunchConfiguration('image_topic')),
                ('image_with_objects', 'image_with_objects'),
                
            ]),                 
    ])
