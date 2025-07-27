import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
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
          'model_file': example_dir + '/pddl/MMdomainextended.pddl',
          'namespace': LaunchConfiguration('plansys2_namespace')
          }.items())
    
    # Robot-specific configurations, but without separate namespaces for actions
    robots = ["robot{0}".format(i) for i in range(0, 4)]
    robot_actions = []

    # ADDING NAMESPACE TO ROBOTS ?
    for robot in robots:
        landing_cmd = Node(
            package='plansys2_testexample',
            executable='landing_action_node_exec',
            name=f'landing_action_node_{robot}',
            output='screen',
            parameters=[{'specialized_arguments': [robot]}]) 
        
        takeoff_cmd = Node(
            package='plansys2_testexample',
            executable='takeoff_action_node_exec',
            name=f'takeoff_action_node_{robot}',
            output='screen',
            parameters=[{'specialized_arguments': [robot]}]) 
        
        navigation_air_cmd = Node(
            package='plansys2_testexample',
            executable='navigation_air_action_node_exec',
            name=f'navigation_air_action_node_{robot}',
            output='screen',
            parameters=[{'specialized_arguments': [robot]}]) 
        
        navigation_water_cmd = Node(
            package='plansys2_testexample',
            executable='navigation_water_action_node_exec',
            name=f'navigation_water_action_node_{robot}',
            output='screen',
            parameters=[{'specialized_arguments': [robot]}]) 
        
        change_site_cmd = Node(
            package='plansys2_testexample',
            executable='change_site_action_node_exec',
            name=f'change_site_action_node_{robot}',
            output='screen',
            parameters=[{'specialized_arguments': [robot]}]) 
        
        switch_airwater_cmd = Node(
            package='plansys2_testexample',
            executable='switch_airwater_action_node_exec',
            name=f'switch_airwater_action_node_{robot}',
            output='screen',
            parameters=[{'specialized_arguments': [robot]}])
        
        switch_waterair_cmd = Node(
            package='plansys2_testexample',
            executable='switch_waterair_action_node_exec',
            name=f'switch_waterair_action_node_{robot}',
            output='screen',
            parameters=[{'specialized_arguments': [robot]}]) 
        
        translate_data_cmd = Node(
            package='plansys2_testexample',
            executable='translate_data_action_node_exec',
            name=f'translate_data_action_node_{robot}',
            output='screen',
            parameters=[{'specialized_arguments': [robot]}]) 
        
        observe_cmd = Node(
            package='plansys2_testexample',
            executable='observe_action_node_exec',
            name=f'observe_action_node_{robot}',
            output='screen',
            parameters=[{'specialized_arguments': [robot]}]) 
        
        observe_2r_cmd = Node(
            package='plansys2_testexample',
            executable='observe_2r_action_node_exec',
            name=f'observe_2r_action_node_{robot}',
            output='screen',
            parameters=[{'specialized_arguments': [robot]}]) 
        
        sample_cmd = Node(
            package='plansys2_testexample',
            executable='sample_action_node_exec',
            name=f'sample_action_node_{robot}',
            output='screen',
            parameters=[{'specialized_arguments': [robot]}]) 
        
        simulator_cmd = Node(
            package='action_simulator',
            executable='action_simulator_node_exec',
            name=f'simulator_{robot}',
            output='screen',
            parameters=[{'robot_id': robot}])

        robot_actions.extend([
            landing_cmd,
            navigation_water_cmd,
            navigation_air_cmd,
            change_site_cmd,
            observe_cmd,
            observe_2r_cmd,
            switch_airwater_cmd,
            switch_waterair_cmd,
            takeoff_cmd,
            translate_data_cmd,
            sample_cmd,
            simulator_cmd
        ])

    # # Create delayed actions for manager and UI nodes using ExecuteProcess
    # delayed_manager_cmd = TimerAction(
    #     period=3.0,  # Increase delay to ensure complete initialization
    #     actions=[ExecuteProcess(
    #         cmd=['ros2', 'run', 'action_simulator', 'execution_manager_node'],
    #         output='screen'
    #     )]
    # )

    # delayed_user_visualization_cmd = TimerAction(
    #     period=1.0,  # Increase delay to ensure complete initialization
    #     actions=[ExecuteProcess(
    #         cmd=['ros2', 'run', 'user_visualization_interface', 'uservisualization_node'],
    #         output='screen'
    #     )]
    # )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_plansys2_namespace_cmd)
    ld.add_action(plansys2_cmd)

    for action in robot_actions:
        ld.add_action(action)

    # ld.add_action(delayed_manager_cmd)
    # ld.add_action(delayed_user_visualization_cmd)

    return ld
