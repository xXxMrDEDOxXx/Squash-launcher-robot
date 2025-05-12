# File: launch/system_launch.py (e.g., in a bringup package or one of the existing packages)
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define configuration parameters for nodes (could also be loaded from YAML files)
    logger_params = {
        'log_file': 'pressure_log.csv',       # log file name (in current dir or specify full path)
        'logging_enabled': True,             # start with logging on
        'flush_interval': 1.0,               # flush logs every 1 second
        'flush_count': 10,                   # flush after 10 entries
        'actual_pressure_topic': '/actual_pressure',
        'recommended_pressure_topic': '/recommended_pressure'
        # 'target_pressure': 100.0  # Not used by logger directly, would be used by recommender
    }

    # Nodes to launch
    nodes_to_launch = []

    # DataLoggerNode (from logger package)
    nodes_to_launch.append(Node(
        package='logger',
        executable='data_logger_node',  # this refers to the console script set in setup.py for DataLoggerNode
        name='data_logger',
        output='screen',
        parameters=[logger_params]
    ))

    # If the SimulatorInterfaceNode and PressureRecommenderNode were available as executables, 
    # they would be launched similarly, for example:
    #
    # nodes_to_launch.append(Node(
    #     package='simulator',
    #     executable='simulator_interface_node',
    #     name='simulator_interface',
    #     output='screen',
    #     parameters=[{'target_pressure': 100.0}]  # example param usage
    # ))
    # nodes_to_launch.append(Node(
    #     package='recommender',
    #     executable='pressure_recommender_node',
    #     name='pressure_recommender',
    #     output='screen',
    #     parameters=[{'target_pressure': 100.0}]
    # ))
    #
    # We skip actually launching these since their implementation is excluded.

    # Assemble the launch description with all nodes
    return LaunchDescription(nodes_to_launch)
