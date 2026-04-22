
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    #Camera used Z UP, X FWD
    camera_offset_x_arg = DeclareLaunchArgument('camera_offset_x',default_value='0.0',description='camera coordinates (left) relative to USV NED')
    camera_offset_y_arg = DeclareLaunchArgument('camera_offset_y',default_value='0.0',description='camera coordinates (left) relative to USV NED')
    camera_offset_z_arg = DeclareLaunchArgument('camera_offset_z',default_value='0.0',description='camera coordinates (left) relative to USV NED')

    
    model_arg = DeclareLaunchArgument('model',default_value='best.onnx',description='Select which model to use. e.g: best.onnx')
    transform_broadcaster_node = Node(
                            package='transform_broadcaster',
                            executable='transform_broadcaster',
                            name='transform_broadcaster',
                            output='screen',
                            parameters= [
                                {"camera_offset_x":LaunchConfiguration("camera_offset_x")},
                                {"camera_offset_y":LaunchConfiguration("camera_offset_y")},
                                {"camera_offset_z":LaunchConfiguration("camera_offset_z")}
                            ]
                        )

    return LaunchDescription([
        camera_offset_x_arg,
        camera_offset_y_arg,
        camera_offset_z_arg,
        model_arg,
        transform_broadcaster_node
    ])
