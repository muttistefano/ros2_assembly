#include <memory>
#include <string>

#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rclcpp/rclcpp.hpp"

class ParameterBlackboard : public rclcpp::Node
{
public:
  ParameterBlackboard(
    const std::string & name = "param_node",
    const std::string & namespace_ = "",
    const rclcpp::NodeOptions & options = (
      rclcpp::NodeOptions()
      .allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true)
  ))
  : rclcpp::Node(name, namespace_, options)
  {
    RCLCPP_INFO(this->get_logger(),
      "Parameter blackboard node named '%s' ready, and serving '%zu' parameters already!",
      this->get_fully_qualified_name(), this->list_parameters(
        {}, rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE).names.size());
  }
};

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParameterBlackboard>());
  return 0;
}