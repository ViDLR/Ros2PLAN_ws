
# from ament_index_python.packages import get_package_share_directory
# from launch.actions import TimerAction
# import os
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node, PushRosNamespace


# def generate_launch_description():
#     example_dir = get_package_share_directory('plansys2_testexample')
#     bringup_dir = get_package_share_directory('plansys2_bringup')
#     params_file = LaunchConfiguration('params_file')

#     # Create the launch configuration variables
#     model_file = LaunchConfiguration('model_file')
#     params_file = LaunchConfiguration('params_file')
#     action_bt_file = LaunchConfiguration('action_bt_file')
#     start_action_bt_file = LaunchConfiguration('start_action_bt_file')
#     end_action_bt_file = LaunchConfiguration('end_action_bt_file')
#     bt_builder_plugin = LaunchConfiguration('bt_builder_plugin')

#     declare_model_file_cmd = DeclareLaunchArgument(
#         'model_file',
#         default_value= example_dir + '/pddl/MMdomainextended.pddl',
#         description='PDDL Model file')

#     declare_namespace_cmd = DeclareLaunchArgument(
#         'namespace',
#         default_value='',
#         description='Namespace')
#     declare_action_bt_file_cmd = DeclareLaunchArgument(
#         'action_bt_file',
#         default_value=os.path.join(
#           get_package_share_directory('plansys2_executor'),
#           'behavior_trees', 'plansys2_action_bt.xml'),
#         description='BT representing a PDDL action')

#     declare_start_action_bt_file_cmd = DeclareLaunchArgument(
#         'start_action_bt_file',
#         default_value=os.path.join(
#           get_package_share_directory('plansys2_executor'),
#           'behavior_trees', 'plansys2_start_action_bt.xml'),
#         description='BT representing a PDDL start action')

#     declare_end_action_bt_file_cmd = DeclareLaunchArgument(
#         'end_action_bt_file',
#         default_value=os.path.join(
#           get_package_share_directory('plansys2_executor'),
#           'behavior_trees', 'plansys2_end_action_bt.xml'),
#         description='BT representing a PDDL end action')

#     declare_bt_builder_plugin_cmd = DeclareLaunchArgument(
#         'bt_builder_plugin',
#         default_value='SimpleBTBuilder',
#         description='Behavior tree builder plugin.',
#     )

#     declare_params_file_cmd = DeclareLaunchArgument(
#         'params_file',
#         default_value=os.path.join(bringup_dir, 'params', 'plansys2_params.yaml'),
#         description='Full path to the ROS2 parameters file to use for all launched nodes')
    
#     # Shared components (Domain Expert, Problem Expert, Planner)
#     domain_expert_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(
#             get_package_share_directory('plansys2_domain_expert'),
#             'launch',
#             'domain_expert_launch.py')),
#         launch_arguments={
#             'model_file': example_dir + '/pddl/MMdomainextended.pddl',
#             'params_file': params_file
#         }.items())

#     problem_expert_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(
#             get_package_share_directory('plansys2_problem_expert'),
#             'launch',
#             'problem_expert_launch.py')),
#         launch_arguments={
#             'model_file': example_dir + '/pddl/MMdomainextended.pddl',
#             'params_file': params_file
#         }.items())

#     planner_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(
#             get_package_share_directory('plansys2_planner'),
#             'launch',
#             'planner_launch.py')),
#         launch_arguments={
#             'params_file': params_file
#         }.items())
    
#     lifecycle_manager_cmd = Node(
#         package='plansys2_lifecycle_manager',
#         executable='lifecycle_manager_node',
#         name='lifecycle_manager_core',
#         output='screen',
#         parameters=[{
#             'managed_nodes': ['domain_expert', 'problem_expert', 'planner']
#         }]
#     )

#     from launch.actions import TimerAction

#     manager_node_cmd = TimerAction(
#         period=3.0,  # Delay of 3 seconds
#         actions=[
#             Node(
#                 package='action_simulator',
#                 executable='execution_manager_node',
#                 output='screen',
#                 parameters=[
#                     {
#                         'model_file': model_file,
#                         'default_action_bt_xml_filename': action_bt_file,
#                         'default_start_action_bt_xml_filename': start_action_bt_file,
#                         'default_end_action_bt_xml_filename': end_action_bt_file,
#                         'bt_builder_plugin': bt_builder_plugin,
#                     },
#                     params_file
#                 ]
#             )
#         ]
#     )

#     # Create the launch description and populate
#     ld = LaunchDescription()

#     ld.add_action(declare_params_file_cmd)

#     # Shared components
#     ld.add_action(domain_expert_cmd)
#     ld.add_action(problem_expert_cmd)
#     ld.add_action(planner_cmd)
#     ld.add_action(lifecycle_manager_cmd)
#     ld.add_action(declare_model_file_cmd)
#     ld.add_action(declare_action_bt_file_cmd)
#     ld.add_action(declare_start_action_bt_file_cmd)
#     ld.add_action(declare_end_action_bt_file_cmd)
#     ld.add_action(declare_bt_builder_plugin_cmd)
#     ld.add_action(declare_namespace_cmd)
#     ld.add_action(declare_params_file_cmd)

#     # Manager node
#     ld.add_action(manager_node_cmd)
#     return ld



from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    example_dir = get_package_share_directory('plansys2_testexample')
    bringup_dir = get_package_share_directory('plansys2_bringup')

    declare_model_file_cmd = DeclareLaunchArgument(
        'model_file',
        default_value= os.path.join(example_dir, 'pddl', 'MMdomainextended.pddl'),
        description='PDDL Model file')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')
    declare_action_bt_file_cmd = DeclareLaunchArgument(
        'action_bt_file',
        default_value=os.path.join(
          get_package_share_directory('plansys2_executor'),
          'behavior_trees', 'plansys2_action_bt.xml'),
        description='BT representing a PDDL action')

    declare_start_action_bt_file_cmd = DeclareLaunchArgument(
        'start_action_bt_file',
        default_value=os.path.join(
          get_package_share_directory('plansys2_executor'),
          'behavior_trees', 'plansys2_start_action_bt.xml'),
        description='BT representing a PDDL start action')

    declare_end_action_bt_file_cmd = DeclareLaunchArgument(
        'end_action_bt_file',
        default_value=os.path.join(
          get_package_share_directory('plansys2_executor'),
          'behavior_trees', 'plansys2_end_action_bt.xml'),
        description='BT representing a PDDL end action')

    declare_bt_builder_plugin_cmd = DeclareLaunchArgument(
        'bt_builder_plugin',
        default_value='SimpleBTBuilder',
        description='Behavior tree builder plugin.',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'plansys2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # Create the launch configuration variables
    model_file = LaunchConfiguration('model_file')
    params_file = LaunchConfiguration('params_file')
    action_bt_file = LaunchConfiguration('action_bt_file')
    start_action_bt_file = LaunchConfiguration('start_action_bt_file')
    end_action_bt_file = LaunchConfiguration('end_action_bt_file')
    bt_builder_plugin = LaunchConfiguration('bt_builder_plugin')

    plansys2_core_bringup_cmd = Node(
        package='plansys2_testexample',
        executable='plansys2_core_bringup',
        output='screen',
        parameters=[
            {
                'model_file': model_file,
                'default_action_bt_xml_filename': action_bt_file,
                'default_start_action_bt_xml_filename': start_action_bt_file,
                'default_end_action_bt_xml_filename': end_action_bt_file,
                'bt_builder_plugin': bt_builder_plugin,
            },
            params_file
        ]
    )
    
    # Execution Manager Node with Delayed Startup
    execution_manager_node_cmd = TimerAction(
        period=5.0,  # Delay of 5 seconds
        actions=[
            Node(
                package='plansys2_testexample',
                executable='emn_bringup',
                output='screen',
                parameters=[
                    {
                        'model_file': LaunchConfiguration('model_file'),
                        'default_action_bt_xml_filename': LaunchConfiguration('action_bt_file'),
                        'default_start_action_bt_xml_filename': LaunchConfiguration('start_action_bt_file'),
                        'default_end_action_bt_xml_filename': LaunchConfiguration('end_action_bt_file'),
                        'bt_builder_plugin': LaunchConfiguration('bt_builder_plugin'),
                    },
                    LaunchConfiguration('params_file')
                ]
            )
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_params_file_cmd)

    # Shared components
    ld.add_action(declare_model_file_cmd)
    ld.add_action(declare_action_bt_file_cmd)
    ld.add_action(declare_start_action_bt_file_cmd)
    ld.add_action(declare_end_action_bt_file_cmd)
    ld.add_action(declare_bt_builder_plugin_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)

    # Manager node
    ld.add_action(plansys2_core_bringup_cmd)
    ld.add_action(execution_manager_node_cmd)
    return ld