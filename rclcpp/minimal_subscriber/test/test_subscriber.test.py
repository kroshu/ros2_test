import os
import unittest
import rclpy
import launch_testing.util
import launch_testing_ros
import launch
import launch.events
import launch_ros.events.lifecycle
import launch_ros.actions
import lifecycle_msgs.msg
import std_msgs.msg
import uuid


def generate_test_description(ready_fn):
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'
    
    listener = launch_ros.actions.Node(
        node_name='listener',
        package='examples_rclcpp_minimal_subscriber',
        node_executable='subscriber_lambda',
        output='screen',
        env=proc_env
    )
    
    start_tests = launch.actions.OpaqueFunction(function=lambda context: ready_fn())

    
    """configure_listener = launch.actions.EmitEvent(
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
                    msg="Node listener active, starting tests"),
                start_tests
            ]
        )
    )"""
    
    return (launch.LaunchDescription([
            listener,
            launch.actions.OpaqueFunction(function=lambda context: ready_fn())
        ]),
        {
            'listener': listener
        }
    )
    
class TestListener(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls, proc_output, listener):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node('tester_node', context=cls.context)
        publisher = cls.node.create_publisher(
            std_msgs.msg.String,
            '/topic',
            10
        )
        msg = std_msgs.msg.String()
        msg.data = 'test message {}'.format(uuid.uuid4())
        for _ in range(5):
            try:
                publisher.publish(msg)
                proc_output.assertWaitFor(
                    expected_output=msg.data,
                    process=listener,
                    timeout=1.0
                )
            except AssertionError:
                continue
            except launch_testing.util.NoMatchingProcessException:
                continue
            else:
                return
        else:
            assert False, 'Failed to plumb chatter topic to listener process'

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        
    def test_Subscriber(self, listener):
        pub = self.node.create_publisher(
            std_msgs.msg.String,
            '/topic',
            10
        )
        self.addCleanup(self.node.destroy_publisher, pub)
        
        msg = std_msgs.msg.String()
        msg.data = str(uuid.uuid4())
        
        for _ in range(10):

            pub.publish(msg)
            success = self.proc_output.waitFor(
                expected_output=msg.data,
                process=listener,
                timeout=1.0,
            )
            if success:
                break
        assert success, 'Waiting for output timed out'
        
        