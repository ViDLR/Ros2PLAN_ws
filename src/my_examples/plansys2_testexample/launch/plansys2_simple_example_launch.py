import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('plansys2_testexample')

    # Declare namespace for the global PlanSys2 components, default to global if empty
    declare_plansys2_namespace_cmd = DeclareLaunchArgument(
        'plansys2_namespace',
        default_value='',
        description='Global namespace for PlanSys2 components')

    # PlanSys2 system bringup in a common namespace, could be global or specific
    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/test_domain.pddl',
        #   'problem_file': example_dir + '/pddl/test_problem.pddl',
          'namespace': LaunchConfiguration('plansys2_namespace')
          }.items())
    
    # Robot-specific configurations, but without separate namespaces for actions
    robots = ['robot1', 'robot2']
    robot_actions = []
    
    manager_cmd = Node(
            package='action_simulator',
            executable='manager_node',
            name='manager_node',
            output='screen',
            parameters=[{'robots_ids': robots}])

    for robot in robots:
        move_cmd = Node(
            package='plansys2_testexample',
            executable='move_action_node',
            name=f'move_action_node_{robot}',
            output='screen',
            parameters=[{'specialized_arguments': [robot]}])  # Ensure robot_id is passed as a parameter

        charge_cmd = Node(
            package='plansys2_testexample',
            executable='charge_action_node',
            name=f'charge_action_node_{robot}',
            output='screen',
            parameters=[{'specialized_arguments': [robot]}])

        ask_charge_cmd = Node(
            package='plansys2_testexample',
            executable='ask_charge_action_node',
            name=f'ask_charge_action_node_{robot}',
            output='screen',
            parameters=[{'specialized_arguments': [robot]}])
        
        simulator_cmd = Node(
            package='action_simulator',
            executable='action_simulator_node',
            name=f'simulator_{robot}',
            output='screen',
            parameters=[{'robot_id': robot}])

        robot_actions.extend([
            move_cmd,
            charge_cmd,
            ask_charge_cmd,
            simulator_cmd
        ])

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_plansys2_namespace_cmd)
    ld.add_action(plansys2_cmd)
    ld.add_action(manager_cmd)

    for action in robot_actions:
        ld.add_action(action)

    return ld


