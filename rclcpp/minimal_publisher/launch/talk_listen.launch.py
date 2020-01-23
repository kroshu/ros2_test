import os
import sys
from numpy.distutils.npy_pkg_config import pkg_name
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa

from launch import LaunchDescription
from launch import LaunchIntrospector
from launch import LaunchService
import launch
import launch.events
import launch_ros.events.lifecycle
import launch_ros.actions
from launch_ros import get_default_launch_description
import ament_index_python #for finding packages: get_package_share_directory

import lifecycle_msgs.msg

def generate_launch_description():

    talker = launch_ros.actions.Node(
        node_name='talker',
        package='examples_rclcpp_minimal_publisher',
        node_executable='publisher_lambda',
        output='screen')
    
    listener = launch_ros.actions.LifecycleNode(
        node_name='listener',
        package='examples_rclcpp_minimal_subscriber',
        node_executable='subscriber_lambda',
        output='screen')
    
    configure_listener = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(listener),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
        )
    )
    
    activate_listener = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(listener),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
        )
    )
    
    reg_event_handler_listener_configured = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=listener, 
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(
                    msg="Node listener configured, activating"),
                activate_listener
            ]
        )
    )
    
    reg_event_handler_listener_active = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=listener, 
            goal_state='active',
            entities=[
                launch.actions.LogInfo(
                    msg="Node listener active, starting talker"),
                talker
            ]
        )
    )
    
    ld = LaunchDescription();
    ld.add_action(reg_event_handler_listener_configured);
    ld.add_action(reg_event_handler_listener_active);
    ld.add_action(listener);
    ld.add_action(configure_listener);
    
    
    print(LaunchIntrospector().format_launch_description(ld))
    
    return ld

        
