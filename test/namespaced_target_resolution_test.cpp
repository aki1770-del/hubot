// Copyright (c) 2026 Komada (aki1770-del)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// ⚑ THE FIXTURE THAT WAS DECLARED STRUCTURALLY IMPOSSIBLE, AND WAS NOT.
//
// The finding this file answers was measured only in a real multi-process
// navigation stack, and was recorded as invisible to every in-process fixture
// "which hosts the target at root namespace where a relative name resolves
// correctly BY ACCIDENT". That is true of every other suite here, and it was
// read as meaning the case needed a robot.
//
// It did not. The variable is not where the TARGET lives -- it is where the
// HOST lives. Put the filter's own node in `/local_costmap`, leave the target
// at the root exactly where the other suites already put it, and the accident
// is gone: a relative `node:` now has a wrong answer available to it, and
// before the namespace join it took that answer every time.
//
// Everything else keeps working under the namespaced host for a reason worth
// stating, because it is what makes this a single-variable experiment: the
// filter joins its OWN four topic names to the PARENT namespace, and the parent
// of `/local_costmap` is the root. So info, mask and decision all land at the
// root, byte-identical to the un-namespaced fixtures. The ONLY thing that
// changes between those suites and this one is how the integrator's `node:`
// field resolves.
//
// WHY (written before the act): this component's entire product is telling a
// person what to go and look at. It reached the right verdict -- `enforced: NO`
// -- while naming a node that was healthy the whole time, and an operator at
// 03:00 spends her minutes and her trust walking to it while the robot is still
// moving. Cases A and B prove the name is now built correctly. Cases C and D
// prove the SENTENCE tells her which of two opposite things happened, because
// fixing the one configuration that produced this did not fix the sentence, and
// the sentence is the part she reads.
//
// HONEST BOUND: `colcon` cannot run on the host this file was written on, so
// these cases were NOT compiled and NOT executed by their author. They are
// written against the fixture patterns of degrade_at_production_caller_test.cpp
// and costmap_silence_liveness_test.cpp, which do run. Any green claim must
// come from a run, not from this comment.

#include <gtest/gtest.h>

#include <chrono>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "hubot/nav2_compat.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "hubot/zone_parameter_filter.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr char kFilterName[] = "zone_parameter_filter";
constexpr char kInfoTopic[] = "costmap_filter_info";
constexpr char kMaskTopic[] = "mask";

// ⚑ The namespace nav2 actually launches a local costmap under. The whole
// point of this file is that the filter's node sits HERE and not at the root.
constexpr char kHostNamespace[] = "local_costmap";

std::string fp(const std::string & suffix)
{
  return std::string(kFilterName) + "." + suffix;
}

nav_msgs::msg::OccupancyGrid make_mask(uint32_t w, uint32_t h, int8_t fill_value)
{
  nav_msgs::msg::OccupancyGrid mask;
  mask.header.frame_id = "map";
  mask.info.resolution = 1.0;
  mask.info.width = w;
  mask.info.height = h;
  mask.info.origin.orientation.w = 1.0;
  mask.data.assign(w * h, fill_value);
  return mask;
}

// A target at the ROOT namespace -- where nav2 puts controller_server, and
// where the other suites already put this one.
class RootTargetNode : public rclcpp::Node
{
public:
  explicit RootTargetNode(const std::string & name)
  : rclcpp::Node(name)
  {
    declare_parameter("speed", 1.0);
  }
  double getSpeed() {return get_parameter("speed").as_double();}
};

class InfoPublisher : public rclcpp::Node
{
public:
  InfoPublisher()
  : Node("costmap_filter_info_pub")
  {
    publisher_ = create_publisher<nav2_msgs::msg::CostmapFilterInfo>(
      kInfoTopic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    auto msg = std::make_unique<nav2_msgs::msg::CostmapFilterInfo>();
    msg->type = hubot::kZoneParameterFilterType;
    msg->filter_mask_topic = kMaskTopic;
    msg->base = 0.0f;
    msg->multiplier = 1.0f;
    publisher_->publish(std::move(msg));
  }
  ~InfoPublisher() override {publisher_.reset();}

private:
  rclcpp::Publisher<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr publisher_;
};

class MaskPublisher : public rclcpp::Node
{
public:
  explicit MaskPublisher(const nav_msgs::msg::OccupancyGrid & mask)
  : Node("mask_pub")
  {
    publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      kMaskTopic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    publisher_->publish(mask);
  }
  ~MaskPublisher() override {publisher_.reset();}

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

// Reads `zone_decision` -- the surface an integrator actually has. `event` is
// the field this file is about, and it is kept as a HISTORY because a later
// message overwrites it.
class DecisionSubscriber : public rclcpp::Node
{
public:
  DecisionSubscriber()
  : Node("zpf_decision_sub")
  {
    subscriber_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "zone_decision", rclcpp::QoS(50),
      [this](const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
        for (const auto & st : msg->status) {
          for (const auto & kv : st.values) {
            seen_[kv.key].push_back(kv.value);
          }
        }
      });
  }

  std::vector<std::string> history(const std::string & k) const
  {
    auto it = seen_.find(k);
    return it == seen_.end() ? std::vector<std::string>{} : it->second;
  }

  // Did any value this key ever carried contain `needle`?
  bool anyContains(const std::string & k, const std::string & needle) const
  {
    for (const auto & v : history(k)) {
      if (v.find(needle) != std::string::npos) {return true;}
    }
    return false;
  }

private:
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr subscriber_;
  std::map<std::string, std::vector<std::string>> seen_;
};

// param_clients_ is protected; its KEYS are the whole question here, and a
// subclass reads them without widening the public API.
class InspectableZpf : public hubot::ZoneParameterFilter
{
public:
  std::set<std::string> clientKeys() const
  {
    std::set<std::string> keys;
    for (const auto & kv : param_clients_) {keys.insert(kv.first);}
    return keys;
  }
};
}  // namespace

class NamespacedTargetTest : public ::testing::Test
{
protected:
  void TearDown() override
  {
    filter_.reset();
    info_pub_.reset();
    mask_pub_.reset();
    layers_.reset();
    if (decision_sub_) {
      decision_executor_.remove_node(decision_sub_);
      decision_sub_.reset();
    }
    if (node_) {
      node_executor_.remove_node(node_->get_node_base_interface());
    }
    node_.reset();
    if (target_) {
      target_executor_.remove_node(target_);
      target_.reset();
    }
  }

  // `declared_target` is written into the YAML exactly as an integrator would.
  // `spin_target` false models a target that EXISTS and is discoverable but
  // never answers -- the other arm of the discrimination cases C/D make.
  bool build(
    const std::string & declared_target,
    double set_timeout,
    bool create_target,
    bool spin_target)
  {
    if (create_target) {
      target_ = std::make_shared<RootTargetNode>("zpf_target_node");
      target_executor_.add_node(target_);
    }
    spin_target_ = spin_target;

    std::vector<rclcpp::Parameter> cfg = {
      rclcpp::Parameter(fp("filter_info_topic"), std::string(kInfoTopic)),
      rclcpp::Parameter(fp("transform_tolerance"), 0.5),
      rclcpp::Parameter(fp("set_parameters_timeout"), set_timeout),
      rclcpp::Parameter(fp("states"), std::vector<std::string>{"slow"}),
      rclcpp::Parameter(fp("slow.id"), 1),
      rclcpp::Parameter(fp("slow.setpoints"), std::vector<std::string>{"cap"}),
      rclcpp::Parameter(fp("slow.cap.node"), declared_target),
      rclcpp::Parameter(fp("slow.cap.parameter"), std::string("speed")),
      rclcpp::Parameter(fp("slow.cap.value"), rclcpp::ParameterValue(0.25)),
      rclcpp::Parameter(fp("nominal_defaults"), std::vector<std::string>{"cap"}),
      rclcpp::Parameter(fp("nominal_defaults.cap.node"), declared_target),
      rclcpp::Parameter(fp("nominal_defaults.cap.parameter"), std::string("speed")),
      rclcpp::Parameter(fp("nominal_defaults.cap.value"), rclcpp::ParameterValue(1.0)),
    };
    rclcpp::NodeOptions opts;
    opts.parameter_overrides(cfg);

    // ⚑ THE ONE LINE THIS WHOLE FILE EXISTS FOR.
    node_ = std::make_shared<hubot::HostNode>("zpf_test_host", kHostNamespace, opts);
    node_executor_.add_node(node_->get_node_base_interface());

    decision_sub_ = std::make_shared<DecisionSubscriber>();
    decision_executor_.add_node(decision_sub_);

    layers_ = std::make_shared<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    tf_buffer_ = hubot::createTransformBuffer(node_);
    tf_buffer_->setUsingDedicatedThread(true);

    filter_ = std::make_shared<InspectableZpf>();
    filter_->initialize(layers_.get(), kFilterName, tf_buffer_.get(), node_, nullptr);
    filter_->initializeFilter(kInfoTopic);

    info_pub_ = std::make_shared<InfoPublisher>();
    mask_pub_ = std::make_shared<MaskPublisher>(make_mask(4, 4, 1));
    pub_executor_.add_node(info_pub_);
    pub_executor_.add_node(mask_pub_);

    auto start = node_->now();
    while (!filter_->isActive()) {
      if (node_->now() - start > rclcpp::Duration(3s)) {return false;}
      spinFor(10ms);
    }
    return true;
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    auto start = std::chrono::steady_clock::now();
    do {
      pub_executor_.spin_some();
      node_executor_.spin_some();
      decision_executor_.spin_some();
      if (spin_target_) {target_executor_.spin_some();}
      std::this_thread::sleep_for(5ms);
    } while (std::chrono::steady_clock::now() - start < duration);
  }

  // Drives the filter through process(), which is where the pending-set drain
  // runs, and keeps the executors turning between ticks.
  void tickFor(std::chrono::milliseconds duration)
  {
    auto start = std::chrono::steady_clock::now();
    do {
      nav2_costmap_2d::Costmap2D costmap(4, 4, 1.0, 0.0, 0.0, 0);
      // See degrade_at_production_caller_test.cpp: the pose type `process()`
      // takes differs by nav2 line, so the test names neither of them.
      const hubot::FilterPose pose = hubot::makeFilterPose(1.5, 1.5);
      filter_->process(costmap, 0, 0, 4, 4, pose);
      spinFor(50ms);
    } while (std::chrono::steady_clock::now() - start < duration);
  }

  rclcpp::executors::SingleThreadedExecutor node_executor_;
  rclcpp::executors::SingleThreadedExecutor pub_executor_;
  rclcpp::executors::SingleThreadedExecutor target_executor_;
  rclcpp::executors::SingleThreadedExecutor decision_executor_;

  std::shared_ptr<hubot::HostNode> node_;
  std::shared_ptr<InspectableZpf> filter_;
  std::shared_ptr<nav2_costmap_2d::LayeredCostmap> layers_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<InfoPublisher> info_pub_;
  std::shared_ptr<MaskPublisher> mask_pub_;
  std::shared_ptr<DecisionSubscriber> decision_sub_;
  std::shared_ptr<RootTargetNode> target_;
  bool spin_target_{true};
};

// ===========================================================================
// A -- THE NAME. Deterministic, no timing, no target required: the client key
// IS the resolved name, and it is the thing that was wrong.
//
// ⚑ THIS CASE FAILS ON THE TREE BEFORE THE NAMESPACE JOIN. There the declared
// string was stored raw, so the key was `zpf_target_node`, rclcpp resolved that
// against the OWNING node (`/local_costmap`), and the set went to
// `/local_costmap/zpf_target_node` -- a phantom the client creates by asking
// for it. The missing-client guard cannot fire on it, because the key it looks
// up is the same key the phantom was built under.
// ===========================================================================
TEST_F(NamespacedTargetTest, A_RelativeTargetResolvesAgainstTheParentNamespace)
{
  ASSERT_TRUE(build("zpf_target_node", 5.0, /*create_target=*/true, /*spin_target=*/true));

  // ⚑ ASSERT THE PREMISE. If the namespace did not take, every case below
  // passes for the wrong reason and this file has measured nothing.
  ASSERT_STREQ(node_->get_namespace(), "/local_costmap")
    << "the host node is not namespaced, so a relative target name would "
       "resolve correctly by accident and this suite would be vacuous";

  const auto keys = filter_->clientKeys();
  ASSERT_EQ(keys.size(), 1u) << "one declared target must build exactly one client";
  EXPECT_EQ(*keys.begin(), "/zpf_target_node")
    << "a relative `node:` must resolve against the PARENT namespace, the same "
       "rule this filter already applies to its own four topic names";
  EXPECT_EQ(keys.count("/local_costmap/zpf_target_node"), 0u)
    << "resolved against the costmap's own namespace: this is the phantom, and "
       "it is a node that does not exist";
  EXPECT_EQ(keys.count("zpf_target_node"), 0u)
    << "the raw declared string was stored, which is what produced the phantom";
}

// ===========================================================================
// B -- THE ARRIVAL. A is about a string; this is about whether the robot's
// limits actually changed. Both are needed: a correct-looking key that does not
// deliver is the same class of success-shaped value this package exists to
// abolish.
// ===========================================================================
TEST_F(NamespacedTargetTest, B_TheSetActuallyArrivesAtTheRootTarget)
{
  ASSERT_TRUE(build("zpf_target_node", 5.0, /*create_target=*/true, /*spin_target=*/true));
  ASSERT_STREQ(node_->get_namespace(), "/local_costmap");
  ASSERT_DOUBLE_EQ(target_->getSpeed(), 1.0) << "precondition";

  tickFor(2000ms);

  EXPECT_DOUBLE_EQ(target_->getSpeed(), 0.25)
    << "the zone's value never reached the target: under a namespaced host the "
       "set was addressed to a node that does not exist";
  EXPECT_FALSE(filter_->enforcementDegraded())
    << "the target answered, so nothing should be reported degraded";
}

// ===========================================================================
// C and D -- THE SENTENCE, AND THEY ARE A PAIR.
//
// Both arms end `enforced: NO` with a set that was never answered. They differ
// in EXACTLY ONE THING: whether anything exists at the name. Before this
// change both produced the identical words -- "no answer within Ns" -- which
// describes only one of them, and sends the operator to a node in the case
// where there is no node.
//
// ⚑ NEITHER CASE MEANS ANYTHING WITHOUT THE OTHER. If the discrimination were
// removed and the message hard-coded to either branch, one of this pair goes
// red. That is the only property being asserted here.
// ===========================================================================
TEST_F(NamespacedTargetTest, C_AbsentTargetSaysNothingWasFoundAndSendsHerNowhere)
{
  // Nobody is running under this name at all.
  ASSERT_TRUE(build("zpf_target_node", 0.5, /*create_target=*/false, /*spin_target=*/false));
  ASSERT_STREQ(node_->get_namespace(), "/local_costmap");

  tickFor(2500ms);

  EXPECT_TRUE(decision_sub_->anyContains("event", "NO parameter service has been discovered"))
    << "nothing exists at the name, and the report must say so rather than "
       "describe a node that failed to answer";
  EXPECT_FALSE(decision_sub_->anyContains("event", "is the place to look"))
    << "there is no node to look at; naming no node is better than naming one";
  // The mapping she needs to fix her YAML, in the message that reports the fault.
  EXPECT_TRUE(decision_sub_->anyContains("event", "node: zpf_target_node"))
    << "the report names '/zpf_target_node' but her YAML says 'zpf_target_node'; "
       "without the mapping she cannot connect the two";
}

TEST_F(NamespacedTargetTest, D_PresentButSilentTargetIsReportedAsPresent)
{
  // The node EXISTS and is discoverable; its executor is never spun, so it can
  // never answer. Same silence as C, opposite instruction to the operator.
  ASSERT_TRUE(build("zpf_target_node", 1.5, /*create_target=*/true, /*spin_target=*/false));
  ASSERT_STREQ(node_->get_namespace(), "/local_costmap");

  // Let discovery settle before the deadline can expire, so that what is being
  // measured is presence and not discovery latency.
  spinFor(800ms);
  tickFor(3000ms);

  EXPECT_TRUE(decision_sub_->anyContains("event", "A parameter service IS discovered"))
    << "the target is running and merely silent; reporting it as never found "
       "would be the same defect mirrored -- declaring a live node dead";
  EXPECT_FALSE(decision_sub_->anyContains("event", "NO parameter service has been discovered"))
    << "something IS there under that name";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
