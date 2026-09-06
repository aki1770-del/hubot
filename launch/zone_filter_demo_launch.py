# hubot — a costmap filter that actually comes up.
#
# ⚑ WHY THIS FILE EXISTS, WRITTEN BEFORE THE ACT.
#
# A nav2 costmap filter needs FIVE things before it can run at all: the plugin, a
# filter mask, a map_server publishing that mask, a costmap_filter_info_server,
# and params plus a launch file wiring them together. Until 2026-09-06 this
# package shipped the first and told you to build the other four. That is not a
# component; it is a component and a homework assignment, and the homework was
# the part nobody could do without reading nav2's source.
#
# Everything this launches is either a stock nav2 executable or a file installed
# by this package. It reads nothing from a source tree.
#
#   ros2 launch hubot zone_filter_demo_launch.py
#
# WHAT YOU SHOULD SEE, and it is the falsifier for this whole file:
#
#   ros2 topic echo /zone_decision
#       -> enforced: yes
#   the zone_target_demo log
#       -> "set_parameters ARRIVED HERE: demo_speed -> 0.3"
#
# If you see `enforced: NO`, or nothing at all, this launch file has not done its
# job — say so, do not adjust it until it looks right.
#
# TWO CONTROLS SHIP BESIDE IT, and running one is worth more than running this
# twice:
#
#   ros2 launch hubot zone_filter_demo_launch.py overlay:=overlay_target_readonly.yaml
#   ros2 launch hubot zone_filter_demo_launch.py overlay:=overlay_target_inside_namespace.yaml
#
# Both are supposed to end in `enforced: NO`, for two DIFFERENT reasons, and the
# sentences differ. A value with no control beside it is not evidence.
#
# AND ONE ARGUMENT THAT MOVES THE ROBOT WITHOUT A SIMULATOR:
#
#   ros2 launch hubot zone_filter_demo_launch.py robot_x:=2.0 robot_y:=2.0
#
# places it OUTSIDE the painted zone. Same stack, different cell, different
# report. ⚑ The robot does not move during a run — nothing here is a simulator,
# and this package has still never run on a robot.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context, *args, **kwargs):
    pkg = get_package_share_directory('hubot')

    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    mask_yaml = LaunchConfiguration('mask')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')

    # ⚑ An overlay is a params file applied AFTER the base, so a condition is a
    # two-key diff and the subject and its controls cannot drift apart. ros2
    # applies later files over earlier ones.
    #
    # Resolved HERE, in Python, rather than with a PathJoinSubstitution, because
    # joining an EMPTY overlay name yields the params/examples/ DIRECTORY -- and
    # ros2 then fails to open a directory as a parameter file, on the default
    # invocation, for a feature nobody asked for. The first draft of this file
    # did exactly that and its own comment claimed the opposite.
    overlay = context.perform_substitution(LaunchConfiguration('overlay'))

    # ⚑ `nominal_defaults` IS SET HERE, NOT IN THE YAML, AND THAT IS THE ONLY
    # FORM THAT HAS EVER BEEN MEASURED TO LOAD.
    #
    # hubot reads `<filter>.nominal_defaults` as a string ARRAY and
    # `<filter>.nominal_defaults.<name>.node` as children of the SAME key. In a
    # ROS 2 parameter FILE one key cannot be both a sequence and a mapping, so
    # there is no yaml spelling of this block -- README.md says so and says the
    # dotted form is untested. Passed as node parameters from a launch file the
    # dotted names are exactly what the code reads, and there is no yaml
    # document for them to conflict inside.
    #
    # It matters: without it, driving OUT of a zone restores nothing. The filter
    # says so at startup ("no matching nominal_defaults entry exists; state-0
    # reset will NOT restore it") and then reports `enforced: yes` for a state
    # that put nothing back. A demo that left this out would ship the reassuring
    # value this package exists to abolish.
    nominal_defaults = {
        'zone_filter.nominal_defaults': ['restore_speed'],
        'zone_filter.nominal_defaults.restore_speed.node': '/zone_target_demo',
        'zone_filter.nominal_defaults.restore_speed.parameter': 'demo_speed',
        'zone_filter.nominal_defaults.restore_speed.value': 1.0,
    }

    costmap_params = [params_file, nominal_defaults]
    if overlay:
        costmap_params.append(os.path.join(pkg, 'params', 'examples', overlay))

    return _node_list(pkg, params_file, costmap_params, map_yaml, mask_yaml, robot_x, robot_y)


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory('hubot')

    declares = [
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg, 'params', 'zone_filter_demo.yaml'),
            description='Base parameters. Defaults to the one installed by this package.',
        ),
        DeclareLaunchArgument(
            'overlay',
            default_value='',
            description=(
                'A file in params/examples/ applied over the base: '
                'overlay_target_readonly.yaml or overlay_target_inside_namespace.yaml. '
                'Both are controls and both are supposed to end in enforced: NO.'
            ),
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg, 'maps', 'demo_map.yaml'),
            description='The world the costmap static_layer reads.',
        ),
        DeclareLaunchArgument(
            'mask',
            default_value=os.path.join(pkg, 'maps', 'zone_mask.yaml'),
            description='The zone mask. Plain text; open it.',
        ),
        DeclareLaunchArgument(
            'robot_x', default_value='6.5',
            description='Where to stand the robot. 6.5,6.5 is inside the painted zone.',
        ),
        DeclareLaunchArgument(
            'robot_y', default_value='6.5',
            description='Try 2.0,2.0 for outside it.',
        ),
    ]

    return LaunchDescription(declares + [OpaqueFunction(function=_nodes)])


def _node_list(pkg, params_file, costmap_params, map_yaml, mask_yaml, robot_x, robot_y):
    return [
        # ── the node hubot enforces the limit ON. Not a nav2 node; see
        #    src/zone_target_demo_node.cpp for why a demo cannot omit it.
        Node(
            package='hubot', executable='zone_target_demo_node',
            name='zone_target_demo', output='screen',
            parameters=[params_file],
        ),

        # ── TF. Two STATIC transforms and no broadcaster process.
        # ⚑ /tf_static is timeless in tf2, so getRobotPose() keeps succeeding for
        # as long as the run lasts. A transform broadcast on /tf instead would go
        # stale against the costmap's transform_tolerance, getRobotPose() would
        # start failing, and the map update thread would stop driving the filter
        # WITHOUT anything dying -- which looks exactly like a healthy stack.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='tf_map_odom', output='log',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='tf_odom_base', output='log',
            arguments=['--x', robot_x, '--y', robot_y,
                       '--frame-id', 'odom', '--child-frame-id', 'base_link'],
        ),

        # ── the world, so the costmap's static_layer declares a window every
        #    cycle. Without a bounds-declaring plugin the filter is driven ZERO
        #    times and everything still looks green; see params/zone_filter_demo.yaml note 2.
        Node(
            package='nav2_map_server', executable='map_server',
            name='map_server', output='screen',
            parameters=[params_file, {'yaml_filename': map_yaml}],
        ),

        # ── the mask, and the info message that tells the filter where it is.
        Node(
            package='nav2_map_server', executable='map_server',
            name='zone_mask_server', output='screen',
            parameters=[params_file, {'yaml_filename': mask_yaml}],
        ),
        Node(
            package='nav2_map_server', executable='costmap_filter_info_server',
            name='costmap_filter_info_server', output='screen',
            parameters=[params_file],
        ),

        # ── the costmap that hosts the filter. nav2_costmap_2d's own standalone
        #    binary -- no controller_server, no planner, no behaviour tree.
        # ⚑ The namespace is not decoration. The standalone node is
        # LifecycleNode("costmap", "") and would otherwise sit at ROOT, which is
        # the one place a relative target name resolves correctly by accident.
        # Putting it in /local_costmap gives the demo production's topology, so a
        # configuration that works here works on a robot.
        Node(
            package='nav2_costmap_2d', executable='nav2_costmap_2d',
            name='costmap', namespace='local_costmap', output='screen',
            parameters=costmap_params,
        ),

        # ── configure + activate the four lifecycle nodes above.
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager', output='screen',
            parameters=[params_file],
        ),
    ]
