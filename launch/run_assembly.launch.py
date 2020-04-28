import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ros2_launch_util import *
from nav2_common.launch import RewrittenYaml


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    rviz_dir = LaunchConfiguration(
        'rviz_dir',
        default=os.path.join(
            get_package_share_directory('ros2_assembly'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_system  = xacro_to_urdf("ros2_assembly", "urdf", "system.urdf.xacro")
    urdf_sweepee = xacro_to_urdf("ros2_assembly", "urdf/sweepee", "sweepee.urdf.xacro")
    urdf_ur5     = xacro_to_urdf("ros2_assembly", "urdf", "ur5.urdf.xacro")

    robot_description_config = ""
    with open(urdf_system, 'r') as file:
            robot_description_config = file.read()
    robot_description = {'robot_description' : robot_description_config}

    sweepee_description_config = ""
    with open(urdf_sweepee, 'r') as file:
            sweepee_description_config = file.read()
    sweepee_description = {'robot_description' : sweepee_description_config}

    ur5_description_config = ""
    with open(urdf_ur5, 'r') as file:
            ur5_description_config = file.read()
    ur5_description = {'robot_description' : ur5_description_config}

    map_param = {'yaml_filename' : '/home/kolmogorov/ros2/ros2_extra/src/ros2_assembly/map/second.yaml'}

    rviz_config_file = os.path.join(
            get_package_share_directory('ros2_assembly'),
            'rviz',
            'model.rviz')

    param_substitutions = {
        'use_sim_time': 'true',
        'yaml_filename': '/home/kolmogorov/ros2/ros2_extra/src/ros2_assembly/map/second.yaml'}

    configured_params = RewrittenYaml(
        source_file=os.path.join(get_package_share_directory("ros2_assembly"), "config", "map.yaml"),
        root_key='/',
        param_rewrites=param_substitutions,
        convert_types=True)

    rviz2_node =    Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            output='log',
            arguments=['-d', rviz_config_file],
            parameters=[robot_description])

    map_serv_node = Node(
            package='nav2_map_server',
            node_executable='map_server',
            node_name='map_server',
            output='screen',
            parameters=[configured_params])

    rs_sweepee =    Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            node_name='robot_state_publisher_sweepee',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                        ('/robot_description', '/robot_description_sweepee'),
                        ('/joint_states', '/sweepee/joint_states'),
                        ],
            arguments=[urdf_sweepee])

    rs_ur5 =    Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            node_name='robot_state_publisher_ur5',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                        ('/robot_description', '/robot_description_ur5'),
                        ('/joint_states', '/ur5/joint_states'),
                        ],
            arguments=[urdf_ur5])

    fk_sweepee =    Node(
            package='fake_joint_driver',
            node_executable='fake_joint_driver_node',
            node_namespace ='sweepee',
            #node_name='fake_joint_driver_node_try',
            parameters=[os.path.join(get_package_share_directory("ros2_assembly"), "config", "sweepee_controllers.yaml"),
                        os.path.join(get_package_share_directory("ros2_assembly"), "config", "sweepee_start_positions.yaml"),
                        sweepee_description]
            )
      
    fk_ur5 =   Node(
            package='fake_joint_driver',
            node_executable='fake_joint_driver_node',
            node_namespace ='ur5',
            #node_name='fake_joint_driver_node_try',
            parameters=[os.path.join(get_package_share_directory("ros2_assembly"), "config", "ur5_controllers.yaml"),
                        os.path.join(get_package_share_directory("ros2_assembly"), "config", "ur5_start_positions.yaml"),
                        ur5_description]
            )

        
        # Node(
        #     package='nav2_lifecycle_manager',
        #     node_executable='lifecycle_manager',
        #     node_name='lifecycle_manager_localization',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time},
        #                 {'autostart': True},
        #                 {'node_names': ['/map_server']}]),



    tf1 =     Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            node_name='static_transform_publisher',
            output='log',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'map'])

    tf2 =     Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            node_name='static_transform_publisher',
            output='log',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'base_footprint'])



    ld = LaunchDescription()

    ld.add_action(map_serv_node)
    ld.add_action(rviz2_node)
    ld.add_action(rs_sweepee)
    ld.add_action(rs_ur5)
    ld.add_action(fk_sweepee)
    ld.add_action(fk_ur5)
    ld.add_action(tf1)
    ld.add_action(tf2)

    return ld