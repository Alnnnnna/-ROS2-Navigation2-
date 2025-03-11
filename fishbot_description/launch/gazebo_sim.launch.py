import launch
import launch.event_handlers
import launch_ros
from ament_index_python.packages import get_package_share_directory
import launch_ros.descriptions
import launch_ros.parameter_descriptions
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch import event_handlers

def generate_launch_description():

    urdf_tutorial_path = get_package_share_directory('fishbot_description')
    default_model_path = urdf_tutorial_path +'/urdf/fishbot/fishbot.urdf.xacro'
    default_world_path = urdf_tutorial_path +'/world/test_world.world'

    action_declare_arg_node_path = launch.actions.DeclareLaunchArgument(
        name = 'model',default_value=str(default_model_path),
        description='Path to the URDF file')
    
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]),
            value_type=str)
    
    
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}], 
    )
    
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments=[('world', default_world_path), ('verbose', 'true')]
    )

    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic','/robot_description',
                   '-entity','first_xacrorobot',]
    )

    action_load_joint_controller = launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller fishbot_joint_state_broadcaster --set-state active'.split(' '),
        output='screen'
    )

    action_load_diff_driver_controller = launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller fishbot_diff_drive_controller --set-state active'.split(' '),
        output='screen'
    )





    

    return launch.LaunchDescription([
        action_declare_arg_node_path,
        robot_state_publisher_node,
        launch_gazebo,
        action_spawn_entity,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_spawn_entity,
                on_exit=[action_load_joint_controller],
        
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_load_joint_controller,
                on_exit=[action_load_diff_driver_controller],
            )
        )
        
        

    ])
 