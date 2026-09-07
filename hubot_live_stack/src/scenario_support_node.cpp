// hubot_live_stack — THE COSTMAP-SIDE PRECONDITIONS, IN THEIR OWN OS PROCESS.
//
// A real Costmap2DROS will not drive a costmap filter until two things arrive
// that no test fixture has to think about, because a fixture calls
// `initializeFilter()` by hand:
//
//   1. a `nav2_msgs/CostmapFilterInfo` on the filter's `filter_info_topic`,
//      read with `nav2::qos::LatchedSubscriptionQoS()` (zone_parameter_filter.cpp,
//      `initializeFilter`) -- so the publisher MUST be TRANSIENT_LOCAL or a
//      filter that subscribes later gets nothing and stays inactive forever;
//   2. the mask `nav_msgs/OccupancyGrid` named by that info message, same QoS.
//
// And Costmap2DROS::mapUpdateLoop only calls `updateMap()` when `getRobotPose()`
// succeeds (costmap_2d_ros.cpp:609), so without TF the map_update_thread_ spins
// at 5 Hz and drives the filter ZERO times -- a harness that forgot TF would
// look like a working stack and exercise nothing.
//
// SUBSTRATE ONLY -- no assertions. Flags exist so a case can name the
// precondition it is removing (`--withhold-mask`, `--stop-tf-after-ms`).

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

using namespace std::chrono_literals;

namespace
{

class ScenarioSupport : public rclcpp::Node
{
public:
  ScenarioSupport()
  : rclcpp::Node("hubot_live_stack_support")
  {
    info_topic_ = declare_parameter<std::string>("filter_info_topic", "/costmap_filter_info");
    mask_topic_ = declare_parameter<std::string>("mask_topic", "/zone_filter_mask");
    global_frame_ = declare_parameter<std::string>("global_frame", "odom");
    robot_frame_ = declare_parameter<std::string>("robot_base_frame", "base_link");
    // The value written into every mask cell. hubot maps mask value -> zone id,
    // so this is the knob that selects WHICH zone the robot is standing in.
    mask_value_ = declare_parameter<int>("mask_value", 1);
    mask_cells_ = declare_parameter<int>("mask_cells", 10);
    mask_resolution_ = declare_parameter<double>("mask_resolution", 1.0);
    withhold_mask_ = declare_parameter<bool>("withhold_mask", false);
    stop_tf_after_ms_ = declare_parameter<int>("stop_tf_after_ms", -1);
    // `base` and `multiplier` are the OccupancyGrid -> filter-space conversion
    // (CostmapFilterInfo.msg). Identity keeps mask cell value == zone id, which
    // is what makes `mask_value` legible in a test name.
    base_ = declare_parameter<double>("base", 0.0);
    multiplier_ = declare_parameter<double>("multiplier", 1.0);
    // 4 == nav2_costmap_2d::ZONE_PARAMETER_FILTER, the type hubot registers as.
    filter_type_ = declare_parameter<int>("filter_type", 4);

    rclcpp::QoS latched(1);
    latched.transient_local().reliable();

    info_pub_ = create_publisher<nav2_msgs::msg::CostmapFilterInfo>(info_topic_, latched);
    mask_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(mask_topic_, latched);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    publishInfo();
    if (!withhold_mask_) {
      publishMask();
    } else {
      RCLCPP_WARN(
        get_logger(),
        "hubot_live_stack: withhold_mask=true -- CostmapFilterInfo is published but the mask "
        "it names is NOT. The filter will subscribe and never activate.");
    }

    start_ = now();
    // TF must be continuous: Costmap2DROS::getRobotPose() uses a transform
    // tolerance, so a single static broadcast goes stale and the update loop
    // silently stops driving the filter.
    tf_timer_ = create_wall_timer(50ms, [this]() {publishTf();});
    RCLCPP_INFO(
      get_logger(),
      "hubot_live_stack support up: info=%s mask=%s frames=%s->%s mask_value=%d",
      info_topic_.c_str(), mask_topic_.c_str(), global_frame_.c_str(), robot_frame_.c_str(),
      mask_value_);
  }

  // Re-publish the mask with a new cell value. Used to move the robot between
  // zones without moving the robot -- the same transition production sees when
  // a new mask is served.
  void republishMask(int value)
  {
    mask_value_ = value;
    publishMask();
  }

private:
  void publishInfo()
  {
    nav2_msgs::msg::CostmapFilterInfo msg;
    msg.header.frame_id = global_frame_;
    msg.header.stamp = now();
    msg.type = static_cast<uint8_t>(filter_type_);
    msg.filter_mask_topic = mask_topic_;
    msg.base = static_cast<float>(base_);
    msg.multiplier = static_cast<float>(multiplier_);
    info_pub_->publish(msg);
  }

  void publishMask()
  {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = global_frame_;
    grid.header.stamp = now();
    grid.info.resolution = static_cast<float>(mask_resolution_);
    grid.info.width = static_cast<unsigned int>(mask_cells_);
    grid.info.height = static_cast<unsigned int>(mask_cells_);
    grid.info.origin.position.x = 0.0;
    grid.info.origin.position.y = 0.0;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(
      static_cast<size_t>(mask_cells_) * static_cast<size_t>(mask_cells_),
      static_cast<int8_t>(mask_value_));
    mask_pub_->publish(grid);
    RCLCPP_INFO(get_logger(), "published mask %dx%d all-cells=%d", mask_cells_, mask_cells_,
      mask_value_);
  }

  void publishTf()
  {
    if (stop_tf_after_ms_ >= 0 &&
      (now() - start_) > rclcpp::Duration(std::chrono::milliseconds(stop_tf_after_ms_)))
    {
      if (!tf_stopped_reported_) {
        tf_stopped_reported_ = true;
        RCLCPP_WARN(
          get_logger(),
          "hubot_live_stack: stop_tf_after_ms reached -- TF goes stale. "
          "Costmap2DROS::getRobotPose() will start failing and the map_update_thread_ will "
          "stop driving the filter WITHOUT the node dying. That is the stopped-vs-quiet "
          "condition, produced by the real update loop rather than by not calling it.");
      }
      return;
    }
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = global_frame_;
    t.child_frame_id = robot_frame_;
    // Mid-mask, so the robot is inside the zone the mask declares.
    t.transform.translation.x = mask_cells_ * mask_resolution_ / 2.0;
    t.transform.translation.y = mask_cells_ * mask_resolution_ / 2.0;
    t.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(t);
  }

  std::string info_topic_, mask_topic_, global_frame_, robot_frame_;
  int mask_value_{1}, mask_cells_{10}, stop_tf_after_ms_{-1}, filter_type_{4};
  double mask_resolution_{1.0}, base_{0.0}, multiplier_{1.0};
  bool withhold_mask_{false}, tf_stopped_reported_{false};
  rclcpp::Time start_;
  rclcpp::Publisher<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScenarioSupport>());
  rclcpp::shutdown();
  return 0;
}
