import os
import yaml

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
    except EnvironmentError: 
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.load(file)
    except EnvironmentError: 
        return None

def generate_launch_description():


    use_sim_time  = LaunchConfiguration('use_sim_time', default='false')

    urdf_system   = xacro_to_urdf("ros2_assembly", "urdf", "system.urdf.xacro")
    urdf_sweepee  = xacro_to_urdf("ros2_assembly", "urdf/sweepee", "sweepee.urdf.xacro")
    urdf_ur5      = xacro_to_urdf("ros2_assembly", "urdf", "ur5.urdf.xacro")
    urdf_ur10     = xacro_to_urdf("ros2_assembly", "urdf", "ur10.urdf.xacro")
#     urdf_env     = xacro_to_urdf("ros2_assembly", "urdf", "env.urdf.xacro")

    sweepee_description_config = ""
    with open(urdf_sweepee, 'r') as file:
            sweepee_description_config = file.read()
    sweepee_description = {'robot_description' : sweepee_description_config}

    ur5_description_config = ""
    with open(urdf_ur5, 'r') as file:
            ur5_description_config = file.read()
    ur5_description = {'robot_description' : ur5_description_config}

    ur10_description_config = ""
    with open(urdf_ur10, 'r') as file:
            ur10_description_config = file.read()
    ur10_description = {'robot_description' : ur10_description_config}


    robot_description_config = ""
    with open(urdf_system, 'r') as file:
            robot_description_config = file.read()
    system_description = {'robot_description'         : robot_description_config,
                          'robot_description_ur10'    : ur10_description_config,
                          'robot_description_ur5'     : ur5_description_config,
                          'robot_description_sweepee' : sweepee_description_config,}

#     env_description_config = ""
#     with open(urdf_env, 'r') as file:
#             env_description_config = file.read()
#     env_description = {'env_description' : env_description_config}

    rviz_config_file = "/home/kolmogorov/ros2/ros2_extra/src/ros2_assembly/rviz/model.rviz"

    param_substitutions = {
        'use_sim_time': 'true',
        'yaml_filename': '/home/kolmogorov/ros2/ros2_extra/src/ros2_assembly/map/second.yaml'}

    configured_params = RewrittenYaml(
        source_file=os.path.join(get_package_share_directory("ros2_assembly"), "config", "map.yaml"),
        root_key='/',
        param_rewrites=param_substitutions,
        convert_types=True)

    rviz2_node    = Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[system_description])

    param_node    = Node(
            package='ros2_assembly',
            node_executable='param_node',
            node_name='param_node',
            output='screen',
            parameters=[system_description])

    map_serv_node = Node(
            package='nav2_map_server',
            node_executable='map_server',
            node_name='map_server',
            output='screen',
            parameters=[configured_params])
  
    rs_sweepee    = Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            node_name='robot_state_publisher_sweepee',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            node_namespace='sweepee',
            arguments=[urdf_sweepee])

    rs_ur10       = Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            node_name='robot_state_publisher_ur10',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            node_namespace='ur10',
            arguments=[urdf_ur10])


    rs_ur5        = Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            node_name='robot_state_publisher_ur5',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            node_namespace='ur5',
            arguments=[urdf_ur5])

    fk_ur10       = Node(
            package='fake_joint_driver',
            node_executable='fake_joint_driver_node',
            node_namespace ='ur10',
            #node_name='fake_joint_driver_node_try',
            parameters=[os.path.join(get_package_share_directory("ros2_assembly"), "config", "ur10_controllers.yaml"),
                        os.path.join(get_package_share_directory("ros2_assembly"), "config", "ur10_start_positions.yaml"),
                        ur10_description]
            )

    fk_sweepee    = Node(
            package='fake_joint_driver',
            node_executable='fake_joint_driver_node',
            node_namespace ='sweepee',
            #node_name='fake_joint_driver_node_try',
            parameters=[os.path.join(get_package_share_directory("ros2_assembly"), "config", "sweepee_controllers.yaml"),
                        os.path.join(get_package_share_directory("ros2_assembly"), "config", "sweepee_start_positions.yaml"),
                        sweepee_description]
            )
      
    fk_ur5        = Node(
            package='fake_joint_driver',
            node_executable='fake_joint_driver_node',
            node_namespace ='ur5',
            #node_name='fake_joint_driver_node_try',
            parameters=[os.path.join(get_package_share_directory("ros2_assembly"), "config", "ur5_controllers.yaml"),
                        os.path.join(get_package_share_directory("ros2_assembly"), "config", "ur5_start_positions.yaml"),
                        ur5_description]
            )

    tf1           = Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            node_name='static_transform_publisher_1',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'map'])

    tf2           = Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            node_name='static_transform_publisher_2',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'base_footprint'])
   
    tf_sweepee_ur10   = Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            node_name='static_transform_publisher_sweepee_ur10',
            output='screen',
            arguments=['0.0', '0.0', '0.3', '0.0', '0.0', '0.0', 'base_footprint', 'ur10_base_link'])


    moveit_params_file = get_package_share_directory('ros2_assembly') + "/config/moveit_params.yaml"

    robot_description_semantic_config = load_file('ros2_assembly', 'config/srdf/ur10.srdf')
    ur10_description_semantic      = {'robot_description_semantic' : robot_description_semantic_config}

    ompl_planning_pipeline_config = { 'ompl' : {
        'planning_plugin' : 'ompl_interface/OMPLPlanner',
        'request_adapters' : """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""" ,
        'start_state_max_bounds_error' : 0.1 } }
    ompl_planning_yaml = load_yaml('ros2_assembly', 'config/ompl_planning.yaml')
#     ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)
    

    kinematics_yaml = load_yaml('ros2_assembly', 'config/kinematics.yaml')
    robot_description_kinematics = { 'robot_description_kinematics' : kinematics_yaml }

    controllers_yaml = load_yaml('ros2_assembly', 'config/controllers.yaml')
    moveit_controllers = { 'moveit_simple_controller_manager' : controllers_yaml }

    # MoveItCpp demo executable
    run_moveit_cpp_node = Node(node_name='run_moveit_cpp',
                               package='ros2_assembly',
                               prefix='xterm -e gdb --args',
                               node_namespace='ur10',
                               node_executable='run_moveit_cpp',
                               output='screen',
                               parameters=[moveit_params_file,
                                           ur10_description,
                                           ur10_description_semantic,
                                           kinematics_yaml,
                                           ompl_planning_pipeline_config,
                                           moveit_controllers])


    ld = LaunchDescription()

#     ld.add_action(map_serv_node)

    ld.add_action(rviz2_node)
    ld.add_action(param_node)

    ld.add_action(rs_sweepee)
    ld.add_action(rs_ur5)
    ld.add_action(rs_ur10)

    # ld.add_action(fk_sweepee)
    ld.add_action(fk_ur10)
    ld.add_action(fk_ur5)

    ld.add_action(tf1)
    ld.add_action(tf2)
    ld.add_action(tf_sweepee_ur10)

    ld.add_action(run_moveit_cpp_node)

    return ld