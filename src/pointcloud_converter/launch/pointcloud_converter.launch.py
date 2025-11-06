from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 声明launch参数
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/rslidar_points',
        description='Input PointCloud2 topic without ring and time fields'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/points_raw',
        description='Output PointCloud2 topic with ring and time fields'
    )
    
    num_rings_arg = DeclareLaunchArgument(
        'num_rings',
        default_value='16',
        description='Number of laser rings (e.g., 16, 32, 64)'
    )
    
    # 创建节点
    pointcloud_converter_node = Node(
        package='pointcloud_converter',
        executable='pointcloud_converter_node',
        name='pointcloud_converter_node',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'num_rings': LaunchConfiguration('num_rings'),
        }]
    )
    
    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        num_rings_arg,
        pointcloud_converter_node
    ])
