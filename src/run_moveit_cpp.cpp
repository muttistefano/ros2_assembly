#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <shape_msgs/msg/mesh.hpp>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/body_operations.h>
#include "tf2_ros/transform_listener.h"



static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_demo");

class echoListener
{
public:
  tf2_ros::Buffer buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;

  //constructor with name
  echoListener(rclcpp::Clock::SharedPtr clock) :
    buffer_(clock)
  {
    tfl_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
  };

  ~echoListener()
  {

  };

private:

};

class MoveItCppDemo
{
public:
  MoveItCppDemo(const rclcpp::Node::SharedPtr& node)
    : node_(node)
    , robot_state_publisher_(node_->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 1))
    , trajectory_publisher_(node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
          "/fake_joint_trajectory_controller/joint_trajectory", 1))
    , tf_read(node->get_clock())
    , rate_(30)
  {
  }

  rclcpp::Rate rate_;

  void run()
  {

    RCLCPP_INFO(LOGGER, "Initialize MoveItCpp");
    moveit_cpp_ = std::make_shared<moveit::planning_interface::MoveItCpp>(node_);
    moveit_cpp_->getPlanningSceneMonitor()->setPlanningScenePublishingFrequency(30);


    RCLCPP_INFO(LOGGER, "Initialize PlanningComponent");
    moveit::planning_interface::PlanningComponent arm("manipulator_ur10", moveit_cpp_);

    // A little delay before running the plan
    rclcpp::sleep_for(std::chrono::seconds(3));

    // Create collision object, planning shouldn't be too easy
    RCLCPP_INFO(LOGGER, "Initialize Collisions");
    moveit_msgs::msg::CollisionObject collision_object;
    

    collision_object.header.frame_id = "ur10_base_link";
    collision_object.id = "mesh";
    shapes::Mesh* m = shapes::createMeshFromResource ("file:///home/kolmogorov/ros2/ros2_extra/src/eureca_description/meshes/components/collision/fuselage_nopanelfixing.stl");

    shape_msgs::msg::Mesh mesh;
    
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape ( m, mesh_msg );
    mesh = boost::get<shape_msgs::msg::Mesh> ( mesh_msg );

    collision_object.meshes.resize ( 1 );
    collision_object.mesh_poses.resize ( 1 );
    // collision_object.meshes[0] = mesh;

    geometry_msgs::msg::Pose mesh_or;

    mesh_or = wait_for_trans_pose("ur10_base_link","world");

    // collision_object.mesh_poses[0] = pose;

    collision_object.meshes.push_back ( mesh );
    collision_object.mesh_poses.push_back ( mesh_or);
    collision_object.operation = collision_object.ADD;



    // Add object to planning scene
    {  // Lock PlanningScene
      planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
      scene->processCollisionObjectMsg(collision_object);
    }  // Unlock PlanningScene

    // Set joint state goal
    RCLCPP_INFO(LOGGER, "Set goal");
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.pose.position.x = std::rand()/RAND_MAX;
    goal_pose.pose.position.y = std::rand()/RAND_MAX;
    goal_pose.pose.position.z = std::rand()/RAND_MAX;
    
    goal_pose.pose.orientation.x = 1;
    goal_pose.pose.orientation.y = 1;
    goal_pose.pose.orientation.z = 1;
    goal_pose.pose.orientation.w = 1;
    
    
    std::string link_name = "ur10_ee_link";
    
//     arm.setGoal("home");
    arm.setGoal(goal_pose,link_name);

    // Run actual plan
    RCLCPP_INFO(LOGGER, "Plan to goal");
    const auto plan_solution = arm.plan();
    if (plan_solution)
    {
      visualizeTrajectory(*plan_solution.trajectory);

      // TODO(henningkayser): Enable trajectory execution once controllers are available
      // RCLCPP_INFO(LOGGER, "arm.execute()");
      arm.execute();
      // Right now the joint trajectory controller doesn't support actions and the current way to send trajectory is by
      // using a publisher
      // See https://github.com/ros-controls/ros2_controllers/issues/12 for enabling action interface progress
      RCLCPP_INFO(LOGGER, "Sending the trajectory for execution");
      moveit_msgs::msg::RobotTrajectory robot_trajectory;
      plan_solution.trajectory->getRobotTrajectoryMsg(robot_trajectory);
      trajectory_publisher_->publish(robot_trajectory.joint_trajectory);
    }
  }

private:
  void visualizeTrajectory(const robot_trajectory::RobotTrajectory& trajectory)
  {
    moveit_msgs::msg::DisplayRobotState waypoint;
    const auto start_time = node_->now();
    for (size_t i = 0; i < trajectory.getWayPointCount(); ++i)
    {
      moveit::core::robotStateToRobotStateMsg(trajectory.getWayPoint(i), waypoint.state);
      const auto waypoint_time =
          start_time + rclcpp::Duration::from_seconds(trajectory.getWayPointDurationFromStart(i));
      const auto now = node_->now();
      if (waypoint_time > now)
        rclcpp::sleep_for(std::chrono::nanoseconds((waypoint_time - now).nanoseconds()));

      robot_state_publisher_->publish(waypoint);
    }
  }

  geometry_msgs::msg::TransformStamped wait_for_trans_tf(std::string fr1,std::string fr2)
  {

    geometry_msgs::msg::TransformStamped echo_transform;

    while(rclcpp::ok() && !tf_read.buffer_.canTransform(fr1, fr2, std::chrono::system_clock::now()))
    {
      rate_.sleep();

    }

    try
    {
      echo_transform = tf_read.buffer_.lookupTransform(fr1, fr2, std::chrono::system_clock::now());
      auto translation = echo_transform.transform.translation;
      auto rotation = echo_transform.transform.rotation;
      RCLCPP_INFO(LOGGER, "ADASDASDASDASDASDASDASDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD");
      std::cout << "- Translation: [" << translation.x << ", " << translation.y << ", " << translation.z << "]" << std::endl;
      std::cout << "- Rotation: in Quaternion [" << rotation.x << ", " << rotation.y << ", " 
                << rotation.z << ", " << rotation.w << "]" << std::endl;
    }
    catch(tf2::TransformException& ex)
    {
      std::cout << "Exception thrown:" << ex.what()<< std::endl;
      std::cout << "The current list of frames is:" <<std::endl;
      
    }
    return echo_transform;
  }

  geometry_msgs::msg::Pose             wait_for_trans_pose(std::string fr1,std::string fr2)
  {

    geometry_msgs::msg::TransformStamped echo_transform;
    geometry_msgs::msg::Pose Pose_ret;

    while(rclcpp::ok() && !tf_read.buffer_.canTransform(fr1, fr2, std::chrono::system_clock::now()))
    {
      rate_.sleep();

    }

    try
    {
      echo_transform = tf_read.buffer_.lookupTransform(fr1, fr2, std::chrono::system_clock::now());
      auto translation = echo_transform.transform.translation;
      auto rotation = echo_transform.transform.rotation;
      Pose_ret.position.x = translation.x;
      Pose_ret.position.y = translation.y;
      Pose_ret.position.z = translation.z;
      Pose_ret.orientation.x = rotation.x;
      Pose_ret.orientation.y = rotation.y;
      Pose_ret.orientation.z = rotation.z;
      Pose_ret.orientation.w = rotation.w;
      RCLCPP_INFO(LOGGER, "ADASDASDASDASDASDASDASDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD");
      std::cout << "- Translation: [" << translation.x << ", " << translation.y << ", " << translation.z << "]" << std::endl;
      std::cout << "- Rotation: in Quaternion [" << rotation.x << ", " << rotation.y << ", " 
                << rotation.z << ", " << rotation.w << "]" << std::endl;
    }
    catch(tf2::TransformException& ex)
    {
      std::cout << "Exception thrown:" << ex.what()<< std::endl;
      std::cout << "The current list of frames is:" <<std::endl;
      
    }


    return Pose_ret;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
  moveit::planning_interface::MoveItCppPtr moveit_cpp_;
  echoListener tf_read;  

};

int main(int argc, char** argv)
{
  RCLCPP_INFO(LOGGER, "Initialize node");
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  // This enables loading undeclared parameters
  // best practice would be to declare parameters in the corresponding classes
  // and provide descriptions about expected use
  node_options.automatically_declare_parameters_from_overrides(true);
  
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("run_moveit_cpp", "", node_options);

  MoveItCppDemo demo(node);
  std::thread run_demo([&demo]() {
    // Let RViz initialize before running demo
    // TODO(henningkayser): use lifecycle events to launch node
    rclcpp::sleep_for(std::chrono::seconds(5));
    demo.run();
  });

  rclcpp::spin(node);
  run_demo.join();

  return 0;
}
