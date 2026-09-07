// hubot_live_stack — THE PARAMETER TARGET, IN ITS OWN OS PROCESS.
//
// WHY THIS IS A SEPARATE EXECUTABLE AND NOT A NODE IN A TEST FIXTURE
// -----------------------------------------------------------------
// Every hubot suite to date hosts the filter's parameter target in the SAME
// process as the filter, on an executor the TEST THREAD pumps by hand
// (pluginlib_live_costmap_test.cpp: `target_executor_.spin_some()` inside
// `spinAll()`). That topology cannot be trusted to answer the question the
// filter exists to answer, in either direction:
//
//   * it can MANUFACTURE an async effect -- the target only ever answers when
//     the test thread chooses to pump it, so "the set was still pending" may be
//     an artifact of the pump schedule and not of the target at all;
//   * it can MASK one -- an in-process client/server pair can complete inside
//     a single spin_some() that a real cross-process round trip would not.
//
// `rclcpp::AsyncParametersClient` (zone_parameter_filter.cpp:387) issues a real
// `<target>/set_parameters` service call. Across an OS process boundary that is
// a real round trip over the middleware, scheduled by nobody in this program.
// That is the only topology in which "the set is pending" is an observation.
//
// SUBSTRATE ONLY. This process asserts nothing. It offers the target-side
// conditions FSE needs to write assertions against, each switched on by a flag
// so a case names the condition it is exercising.

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace
{

struct Options
{
  std::string node_name{"zpf_target_node"};
  // Milliseconds of latency injected into the set-parameters callback. The
  // filter's own deadline is `set_parameters_timeout` (default 5.0 s), so a
  // value above that produces a late answer rather than no answer.
  int reply_delay_ms{0};
  // After this many ms of normal service, stop processing callbacks entirely
  // while the process stays alive. The service remains in the graph, so the
  // filter's client still finds a target -- it simply never answers. This is
  // the "target hung" condition, and it is NOT the same as the process exiting.
  int freeze_after_ms{-1};
  // After this many ms, exit(0). This is the "target went away" condition, and
  // it differs from freeze: the service leaves the graph.
  int exit_after_ms{-1};
  // Refuse every set, including ones to writable parameters. Independent of the
  // read-only parameter below, which is refused by rclcpp itself.
  bool refuse_all{false};
};

Options parse(int argc, char ** argv)
{
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string a{argv[i]};
    auto val = [&](int & out) {
        if (i + 1 < argc) {out = std::atoi(argv[++i]);}
      };
    if (a == "--node-name" && i + 1 < argc) {o.node_name = argv[++i];} else if (
      a == "--reply-delay-ms") {val(o.reply_delay_ms);} else if (
      a == "--freeze-after-ms") {val(o.freeze_after_ms);} else if (
      a == "--exit-after-ms") {val(o.exit_after_ms);} else if (
      a == "--refuse-all") {o.refuse_all = true;}
  }
  return o;
}

class ParamTarget : public rclcpp::Node
{
public:
  explicit ParamTarget(const Options & o)
  : rclcpp::Node(o.node_name), opts_(o)
  {
    // The REFUSING parameter. rclcpp itself rejects a set on a read-only
    // parameter, so the refusal is the middleware's, not a callback of ours
    // pretending to be one.
    rcl_interfaces::msg::ParameterDescriptor ro;
    ro.description = "Read-only on purpose: a set on this is refused by rclcpp.";
    ro.read_only = true;
    this->declare_parameter("readonly_speed", 1.0, ro);

    // The ACCEPTING parameter, so a case can distinguish "the target refused"
    // from "the target never answered" -- two different failures that the
    // in-process fixture reports identically.
    this->declare_parameter("writable_speed", 1.0);

    cb_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult r;
        r.successful = !opts_.refuse_all;
        if (opts_.refuse_all) {
          r.reason = "hubot_live_stack: --refuse-all";
        }
        if (opts_.reply_delay_ms > 0) {
          // Deliberately blocking. With a MultiThreadedExecutor the node stays
          // responsive on other callbacks, which is what a busy real target
          // looks like.
          std::this_thread::sleep_for(std::chrono::milliseconds(opts_.reply_delay_ms));
        }
        for (const auto & p : params) {
          RCLCPP_INFO(
            this->get_logger(), "set_parameters: %s -> %s (successful=%s)",
            p.get_name().c_str(), p.value_to_string().c_str(), r.successful ? "true" : "false");
        }
        return r;
      });

    RCLCPP_INFO(
      this->get_logger(),
      "hubot_live_stack param target up: name=%s reply_delay_ms=%d freeze_after_ms=%d "
      "exit_after_ms=%d refuse_all=%s",
      o.node_name.c_str(), o.reply_delay_ms, o.freeze_after_ms, o.exit_after_ms,
      o.refuse_all ? "true" : "false");
  }

private:
  Options opts_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr cb_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const Options o = parse(argc, argv);
  auto node = std::make_shared<ParamTarget>(o);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);

  const auto start = std::chrono::steady_clock::now();
  auto elapsed_ms = [&]() {
      return static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - start).count());
    };

  while (rclcpp::ok()) {
    exec.spin_once(std::chrono::milliseconds(50));
    if (o.exit_after_ms >= 0 && elapsed_ms() >= o.exit_after_ms) {
      RCLCPP_WARN(node->get_logger(), "hubot_live_stack: --exit-after-ms reached; leaving graph");
      rclcpp::shutdown();
      return 0;
    }
    if (o.freeze_after_ms >= 0 && elapsed_ms() >= o.freeze_after_ms) {
      RCLCPP_WARN(
        node->get_logger(),
        "hubot_live_stack: --freeze-after-ms reached; the service stays in the graph and "
        "stops answering. This process will NOT exit.");
      // Hold the node alive, unspun. Sets arrive and are never serviced.
      while (rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
      break;
    }
  }
  rclcpp::shutdown();
  return 0;
}
