# Copyright 2019 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('plansys2_simple_example')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/simple_example.pddl',
          'namespace': namespace
          }.items())

    # Specify the actions
    move_cmd = Node(
        package='plansys2_simple_example',
        executable='move_action_node',
        name='move_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    charge_cmd = Node(
        package='plansys2_simple_example',
        executable='charge_action_node',
        name='charge_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    ask_charge_cmd = Node(
        package='plansys2_simple_example',
        executable='ask_charge_action_node',
        name='ask_charge_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])   # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    ld.add_action(move_cmd)
    ld.add_action(charge_cmd)
    ld.add_action(ask_charge_cmd)

    return ld
    
    
    
    
    
    
    
    
    
    
    
    import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('plansys2_simple_example_py')

    # Declare namespace for the global PlanSys2 components
    declare_plansys2_namespace_cmd = DeclareLaunchArgument(
        'plansys2_namespace',
        default_value='',
        description='Global namespace for PlanSys2 components')

    # PlanSys2 system bringup in a common namespace
    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/simple_example.pddl',
          'namespace': LaunchConfiguration('plansys2_namespace')
          }.items())

    # Robot-specific configurations
    robots = ['robot1', 'robot2']
    robot_actions = []

    for robot in robots:
        declare_namespace_cmd = DeclareLaunchArgument(
            f'namespace_{robot}',
            default_value=robot,
            description=f'Namespace for {robot}')

        move_cmd = Node(
            package='plansys2_simple_example_py',
            executable='move_action_node',
            name='move_action_node',
            namespace=LaunchConfiguration(f'namespace_{robot}'),
            output='screen',
            parameters=[])

        charge_cmd = Node(
            package='plansys2_simple_example_py',
            executable='charge_action_node.py',
            name='charge_action_node',
            namespace=LaunchConfiguration(f'namespace_{robot}'),
            output='screen',
            parameters=[])

        ask_charge_cmd = Node(
            package='plansys2_simple_example_py',
            executable='ask_charge_action_node',
            name='ask_charge_action_node',
            namespace=LaunchConfiguration(f'namespace_{robot}'),
            output='screen',
            parameters=[])

        robot_actions.extend([
            declare_namespace_cmd,
            move_cmd,
            charge_cmd,
            ask_charge_cmd
        ])

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_plansys2_namespace_cmd)
    ld.add_action(plansys2_cmd)

    for action in robot_actions:
        ld.add_action(action)

    return ld
