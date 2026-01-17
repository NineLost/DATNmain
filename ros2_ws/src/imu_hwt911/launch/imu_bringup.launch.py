from launch import LaunchDescription 

from launch_ros.actions import Node 

 

def generate_launch_description(): 

    return LaunchDescription([ 

        Node( 

            package='imu_hwt911', 

            executable='hwt911_node', 

            name='hwt911', 

            parameters=[{'port':'/dev/imu', 'baud':57600, 'frame_id':'imu_link'}], 

            output='screen' 

        ), 

        Node( 

            package='tf2_ros', 

            executable='static_transform_publisher', 

            arguments=['0','0','0','0','0','0','base_link','imu_link'], 

            name='static_tf_base_imu' 

        ), 

    ]) 
