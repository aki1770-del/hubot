// hubot_live_stack — THE OBSERVER, OUTSIDE EVERY PROCESS IT OBSERVES.
//
// hubot's human-decision surface is `zone_decision`
// (diagnostic_msgs/DiagnosticArray, zone_parameter_filter.hpp:299-301) plus the
// upstream-compatible `std_msgs/UInt8` state event. An integrator reads those
// over the wire from another process; so does this.
//
// It writes one JSON object per received message to a file, each stamped with
// the RECEIVE time as well as the message's own header stamp, because the
// question hubot's liveness countermeasure turns on -- "has this filter stopped
// speaking?" -- is a question about ARRIVAL, and an in-process fixture that
// reads a member variable cannot ask it at all.
//
// SUBSTRATE ONLY: it records; it does not judge. Every `enforced`/`watching`
// verdict is FSE's to write against this transcript.

#include <chrono>
#include <fstream>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace
{

std::string jsonEscape(const std::string & in)
{
  std::string out;
  out.reserve(in.size() + 8);
  for (char c : in) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default: out += c;
    }
  }
  return out;
}

class DecisionObserver : public rclcpp::Node
{
public:
  DecisionObserver()
  : rclcpp::Node("hubot_live_stack_observer")
  {
    // ⚑ Measured on the live stack with `ros2 topic list`, not assumed: the
    // filter's `zone_decision` publisher resolves to `/zone_decision` at ROOT,
    // NOT to `/local_costmap/zone_decision`, even though the owning node is
    // `/local_costmap/local_costmap`. An observer subscribed to the plausible
    // namespaced name receives nothing and writes an EMPTY transcript, which
    // reads exactly like a filter that published nothing.
    const auto decision_topic =
      declare_parameter<std::string>("decision_topic", "/zone_decision");
    const auto state_topic =
      declare_parameter<std::string>("state_event_topic", "/zone_filter_state");
    out_path_ = declare_parameter<std::string>("out_path", "/ws/live_stack_transcript.jsonl");

    out_.open(out_path_, std::ios::out | std::ios::trunc);
    if (!out_) {
      RCLCPP_ERROR(get_logger(), "cannot open %s for writing", out_path_.c_str());
    }

    // Sensor-data-ish depth so a burst is not dropped, RELIABLE so a missing
    // record means "not published", never "not delivered". A lossy observer
    // would turn hubot's silence question into a transport question.
    rclcpp::QoS qos(rclcpp::KeepLast(200));
    qos.reliable();

    decision_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      decision_topic, qos,
      [this](diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {onDecision(*msg);});
    state_sub_ = create_subscription<std_msgs::msg::UInt8>(
      state_topic, qos,
      [this](std_msgs::msg::UInt8::SharedPtr msg) {onState(*msg);});

    RCLCPP_INFO(
      get_logger(), "observing decision=%s state=%s -> %s",
      decision_topic.c_str(), state_topic.c_str(), out_path_.c_str());
  }

private:
  void onDecision(const diagnostic_msgs::msg::DiagnosticArray & msg)
  {
    for (const auto & s : msg.status) {
      std::string line = "{\"kind\":\"decision\",\"recv_ns\":" + std::to_string(nowNs()) +
        ",\"stamp_ns\":" + std::to_string(
        rclcpp::Time(msg.header.stamp).nanoseconds()) +
        ",\"level\":" + std::to_string(static_cast<int>(s.level)) +
        ",\"name\":\"" + jsonEscape(s.name) + "\"" +
        ",\"message\":\"" + jsonEscape(s.message) + "\"" +
        ",\"hardware_id\":\"" + jsonEscape(s.hardware_id) + "\"" +
        ",\"values\":{";
      bool first = true;
      for (const auto & kv : s.values) {
        if (!first) {line += ",";}
        first = false;
        line += "\"" + jsonEscape(kv.key) + "\":\"" + jsonEscape(kv.value) + "\"";
      }
      line += "}}";
      emit(line);
    }
  }

  void onState(const std_msgs::msg::UInt8 & msg)
  {
    emit(
      "{\"kind\":\"state_event\",\"recv_ns\":" + std::to_string(nowNs()) +
      ",\"state\":" + std::to_string(static_cast<int>(msg.data)) + "}");
  }

  void emit(const std::string & line)
  {
    if (out_) {
      out_ << line << "\n";
      out_.flush();  // a crash must not cost the transcript
    }
    printf("%s\n", line.c_str());
    fflush(stdout);
  }

  int64_t nowNs()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  }

  std::string out_path_;
  std::ofstream out_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr decision_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr state_sub_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DecisionObserver>());
  rclcpp::shutdown();
  return 0;
}
