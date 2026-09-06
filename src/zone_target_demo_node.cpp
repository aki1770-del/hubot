// hubot — THE NODE THE FILTER TALKS TO, for the demo bring-up only.
//
// ⚑ WHY THIS EXISTS, WRITTEN BEFORE THE ACT.
//
// hubot does not set a variable. It sends `set_parameters` across ROS to a node
// in ANOTHER PROCESS and waits for that node to answer
// (src/zone_parameter_filter.cpp, rclcpp::AsyncParametersClient). So a bring-up
// with no target cannot reach `enforced: yes` at all -- there is nobody to
// enforce anything on, and the launch file would demonstrate the one thing the
// package is not about.
//
// ON YOUR ROBOT THIS NODE IS NOT HERE. The target is `controller_server` and the
// parameter is `FollowPath.max_vel_x`, or your own node and your own parameter.
// params/zone_filter_demo.yaml says so at the line that names this node. This
// executable exists so the first hour ends in a working stack rather than in
// writing one.
//
// It declares exactly two parameters, and the pair is the point:
//
//   demo_speed      writable  -- the set lands, this process logs it, and the
//                               filter reports `enforced: yes`. THE POSITIVE.
//   readonly_speed  read_only -- rclcpp refuses it BEFORE any callback here
//                               runs, so the refusal is rclcpp's own and not a
//                               callback imitating one. THE NEGATIVE, and it is
//                               reached by params/examples/overlay_target_readonly.yaml.
//
// A value with no control beside it is not evidence. That is this package's
// whole argument, so its own demo carries the control.

#include <memory>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

class ZoneTargetDemo : public rclcpp::Node
{
public:
  ZoneTargetDemo()
  : rclcpp::Node("zone_target_demo")
  {
    declare_parameter<double>("demo_speed", 1.0);

    rcl_interfaces::msg::ParameterDescriptor ro;
    ro.read_only = true;
    ro.description =
      "Declared read_only so a set against it is refused by rclcpp itself, "
      "before any callback in this process runs. It is the negative control.";
    declare_parameter<double>("readonly_speed", 1.0, ro);

    // Logged on arrival so the reader can tell a refusal APART from a set that
    // never arrived. Those two look identical from the filter's side, and
    // telling them apart is the whole of hubot's report.
    cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & p : params) {
          RCLCPP_INFO(
            get_logger(), "set_parameters ARRIVED HERE: %s -> %s",
            p.get_name().c_str(), p.value_to_string().c_str());
        }
        return result;
      });

    RCLCPP_INFO(
      get_logger(),
      "zone_target_demo up at %s. demo_speed is writable; readonly_speed is not. "
      "Nothing is logged above unless a set actually reached this process.",
      get_fully_qualified_name());
  }

private:
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr cb_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZoneTargetDemo>());
  rclcpp::shutdown();
  return 0;
}
