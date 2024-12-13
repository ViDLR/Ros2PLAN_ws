from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    
    # Declare the launch argument.
    declare_robot_id = DeclareLaunchArgument(
        'robot_id',
        default_value='robot0',
        description='Robot ID for node initialization.'
    )
    
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the robot.'
    )
    
    robot_name = LaunchConfiguration("robot_id")
    namespace = LaunchConfiguration('namespace')
    
    # Use TextSubstitution to dynamically construct names and parameters
    robot_actions = [
        Node(
            package='plansys2_testexample',
            executable='landing_action_node',
            name=["landing_action_node_", robot_name],
            namespace=namespace,
            output='screen',
            parameters=[{'specialized_arguments': [[robot_name]]}]
        ),
        Node(
            package='plansys2_testexample',
            executable='takeoff_action_node',
            name=["takeoff_action_node_", robot_name],
            namespace=namespace,
            output='screen',
            parameters=[{'specialized_arguments': [[robot_name]]}]
        ),
        Node(
            package='plansys2_testexample',
            executable='navigation_action_node',
            name=["navigation_action_node_", robot_name],
            output='screen',
            parameters=[{'specialized_arguments': [[robot_name]]}]
        ),
        Node(
            package='plansys2_testexample',
            executable='switch_action_node',
            name=["switch_action_node_", robot_name],
            namespace=namespace,
            output='screen',
            parameters=[{'specialized_arguments': [[robot_name]]}]
        ),
        Node(
            package='plansys2_testexample',
            executable='translate_data_action_node',
            name=["translate_data_action_node_", robot_name],
            namespace=namespace,
            output='screen',
            parameters=[{'specialized_arguments': [[robot_name]]}]
        ),
        Node(
            package='plansys2_testexample',
            executable='observe_action_node',
            name=["observe_action_node_", robot_name],
            namespace=namespace,
            output='screen',
            parameters=[{'specialized_arguments': [[robot_name]]}]
        ),
        Node(
            package='plansys2_testexample',
            executable='sample_action_node',
            name=["sample_action_node_", robot_name],
            namespace=namespace,
            output='screen',
            parameters=[{'specialized_arguments': [[robot_name]]}]
        ),
        
        Node(
            package='action_simulator',
            executable='action_simulator_node',
            name=["simulator_", robot_name],
            namespace=namespace,
            output='screen',
            parameters=[{'robot_id': robot_id}]
        ),
    ]
    
    return LaunchDescription([
        declare_robot_id,
        *robot_actions
    ])
