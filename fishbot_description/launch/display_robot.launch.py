import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import launch_ros.descriptions
import launch_ros.parameter_descriptions
import os

def generate_launch_description():
    urdf_path = get_package_share_directory('fishbot_description')
    default_urdf_path = os.path.join(urdf_path, 'urdf', 'first_robot.urdf')
    default_rivz_path = os.path.join(urdf_path, 'config', 'display_robot_model.rviz')
    action_declare_arg_node_path = launch.actions.DeclareLaunchArgument(name='model',default_value=str(default_urdf_path),description='URDF path')
    
    command_result = launch.substitutions.Command(['xacro ',launch.substitutions.LaunchConfiguration('model')])
   
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(command_result,value_type = str)  
    
    
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_value}], 
    )
    
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rivz_path]
        ##arguments=['-d', get_package_share_directory('fishbot_description') + '/rviz/first_robot.rviz']
    )

    return launch.LaunchDescription([
        action_declare_arg_node_path,
        joint_state_publisher_node,
        robot_state_publisher_node,
        
        
        rviz_node
    ])

