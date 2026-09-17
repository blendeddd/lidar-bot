"""
Mapping: slam_toolbox in async online mode, tuned for a small indoor robot.

slam_toolbox is a LIFECYCLE node.  Launching it as a plain Node starts the
process in the 'unconfigured' state, where it creates no publishers, no
subscriptions and no TF broadcaster - it looks alive, logs nothing, and exits
cleanly on Ctrl-C while /map never appears.  So the configure and activate
transitions have to be emitted explicitly, which is what the two events below
do.  Set autostart:=false only if you want to drive the transitions by hand
(ros2 lifecycle set /slam_toolbox configure, then activate).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            RegisterEventHandler, LogInfo)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg = get_package_share_directory('lidarbot')
    default_params = os.path.join(pkg, 'config', 'slam_toolbox.yaml')

    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('slam_params_file')

    slam = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[params_file, {'use_sim_time': False}],
    )

    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(autostart),
    )

    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[lidarbot] slam_toolbox configured, activating'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
        condition=IfCondition(autostart),
    )

    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('slam_params_file', default_value=default_params),
        slam,
        # the handler must be registered before the transition it waits on
        activate,
        configure,
    ])
