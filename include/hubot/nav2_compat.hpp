// Copyright (c) 2026 Komada
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

#ifndef HUBOT__NAV2_COMPAT_HPP_
#define HUBOT__NAV2_COMPAT_HPP_

// ⚑ EVERY nav2-VERSION DIFFERENCE THIS PACKAGE HAS, IN ONE FILE.
//
// Written 2026-09-07 (CPP). Before it, this package required nav2 >= 1.5.0, and
// a reader on jazzy or kilted met a `find_package` error in about one second.
// That wall was measured and then read as permanent. It was not. Six couplings
// produced it; four were CONVENIENCES with no behaviour in them, one is read
// off the base class at compile time, and one is a genuine API move.
//
// Measured at the source on 2026-09-07, in `ros:jazzy-ros-base` +
// ros-jazzy-nav2-costmap-2d 1.3.12, `ros:kilted-ros-base` +
// ros-kilted-nav2-costmap-2d 1.4.2, and a released-nav2-1.5.1 image:
//
//   nav2_ros_common/subscription.hpp:30   using Subscription = rclcpp::Subscription<MessageT>;
//   nav2_ros_common/publisher.hpp:29      using Publisher =
//                                           rclcpp_lifecycle::LifecyclePublisher<MessageT>;
//   nav2_ros_common/qos_profiles.hpp:70-84 class LatchedSubscriptionQoS : public rclcpp::QoS
//                                           { KeepLast(depth); reliable(); transient_local(); }
//   nav2_ros_common/tf2_factories.hpp:37  using TransformBuffer = tf2_ros::Buffer;
//
// Three are literally type aliases and one is a three-line QoS preset. And
// `rclcpp_lifecycle::LifecycleNode::create_publisher` / `::create_subscription`
// are BYTE-IDENTICAL across jazzy, kilted and lyrical -- the same declarations
// at the same line numbers in lifecycle_node.hpp (:222, :239) and the same
// definition in lifecycle_node_impl.hpp (:51-66), including the
// `add_managed_entity(pub)` that keeps a publisher lifecycle-managed. Using the
// base class's factories is not a downgrade; it is the same code nav2's wrapper
// reaches one frame later.
//
// WHERE A FORK REMAINS, IT MEASURES THE FACT IT GUARDS. Not a version string:
// release tag 1.5.1 and branch HEAD both declare "1.5.1", so a version number
// cannot separate them -- the same reasoning CMakeLists.txt already applies to
// the ZONE_PARAMETER_FILTER probe. And two of the forks this file started with
// turned out not to need a guard at all, because the base class states the
// answer in its own signature (see FilterPose below).

#include <memory>
#include <string>
#include <type_traits>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/version.h"  // RCLCPP_VERSION_GTE; not pulled in by rclcpp.hpp on jazzy
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/create_timer_ros.hpp"

#include "nav2_costmap_2d/costmap_filters/costmap_filter.hpp"

// The lifecycle-node class a `Layer` accepts moved packages at nav2 1.5.0.
// HUBOT_NAV2_HAS_ROS_COMMON is set by CMakeLists.txt from
// `find_package(nav2_ros_common QUIET)` -- the availability of the package IS
// the fact, so the probe and the fact are the same thing.
#if HUBOT_NAV2_HAS_ROS_COMMON
#include "nav2_ros_common/lifecycle_node.hpp"
#else
#include "nav2_util/lifecycle_node.hpp"
#endif

namespace hubot
{

// ─────────────────────────────────────────────────────────────────────────────
// (1) THE POSE THE COSTMAP HANDS US.  A REAL ABI CHANGE, AND NO GUARD NEEDED.
//
// nav2 1.5.0 changed the pure virtual `CostmapFilter::process()` from
// `const geometry_msgs::msg::Pose2D &` to `const geometry_msgs::msg::Pose &`
// (costmap_filter.hpp:150-153 on jazzy and kilted, :152-155 on lyrical). One
// `process()` cannot override both, so this difference is not a convenience and
// no amount of decoupling removes it. It is also the only one that reaches this
// filter's own logic.
//
// ⚑ BUT IT NEEDS NO `#if`, BECAUSE THE BASE CLASS ALREADY SAYS WHICH IT IS.
// `FilterPose` is DEDUCED from the signature of the very function we override.
// A preprocessor guard would have been a second, independent statement of the
// same fact -- and a second statement of a fact is a thing that can disagree
// with the first. This one cannot: if upstream changes the signature again,
// the override follows it on the next compile or fails loudly at the `override`
// keyword. `geometry_msgs::msg::Pose2D` is not even named here, which matters
// on its own: measured 2026-09-07, geometry_msgs at lyrical no longer SHIPS
// pose2_d.hpp, so a file that named the type would not compile there.
// ─────────────────────────────────────────────────────────────────────────────
namespace detail
{
template<typename T>
struct filter_pose_of;

template<typename ClassT, typename PoseT>
struct filter_pose_of<void (ClassT::*)(
    nav2_costmap_2d::Costmap2D &, int, int, int, int, const PoseT &)>
{
  using type = PoseT;
};
}  // namespace detail

/// The pose type THIS nav2's `CostmapFilter::process()` is declared with.
using FilterPose =
  typename detail::filter_pose_of<decltype(&nav2_costmap_2d::CostmapFilter::process)>::type;

// The two accessors are ordinary overloads resolved by SFINAE, not branches:
// `Pose` has `.position` and no `.x`, `Pose2D` has `.x` and no `.position`, so
// exactly one of each pair is viable and the other never enters the program.
template<typename PoseT>
inline auto poseX(const PoseT & p)->decltype(p.position.x) {return p.position.x;}
template<typename PoseT>
inline auto poseX(const PoseT & p)->decltype(p.x) {return p.x;}

template<typename PoseT>
inline auto poseY(const PoseT & p)->decltype(p.position.y) {return p.position.y;}
template<typename PoseT>
inline auto poseY(const PoseT & p)->decltype(p.y) {return p.y;}

/// Build the pose THIS nav2's `process()` accepts, from plain coordinates.
///
/// For callers that DRIVE a filter -- the tests, and anything else standing in
/// for a costmap. Same two-overload shape as the accessors: the `Pose` form sets
/// an identity orientation, the `Pose2D` form sets `theta`, and a caller writes
/// neither. Without this a test has to name `geometry_msgs::msg::Pose2D`, which
/// geometry_msgs no longer ships at lyrical.
namespace detail
{
template<typename PoseT>
inline auto setPose(PoseT & p, double x, double y, double)->decltype(p.position.x, void())
{
  p.position.x = x;
  p.position.y = y;
  p.orientation.w = 1.0;
}
template<typename PoseT>
inline auto setPose(PoseT & p, double x, double y, double theta)->decltype(p.x, void())
{
  p.x = x;
  p.y = y;
  p.theta = theta;
}
}  // namespace detail

inline FilterPose makeFilterPose(double x, double y, double theta = 0.0)
{
  FilterPose p;
  detail::setPose(p, x, y, theta);
  return p;
}

// ─────────────────────────────────────────────────────────────────────────────
// (2) THE LIFECYCLE NODE A `Layer` ACCEPTS.  A REAL RENAME.
//
// `Layer::initialize()` takes `const nav2_util::LifecycleNode::WeakPtr &` on
// jazzy and kilted (layer.hpp:77) and `const nav2::LifecycleNode::WeakPtr &` on
// 1.5.x (layer.hpp:78). Same arity, same order, same everything else -- upstream
// moved the class out of nav2_util into the new nav2_ros_common package and
// renamed its namespace. Anything that HOSTS this filter has to construct
// whichever one its `Layer` will accept, so the alias exists for the tests.
//
// ⚑ THE FILTER ITSELF DOES NOT USE THIS. `Layer::node_` is a weak pointer to the
// distro's own type; `node_.lock()` is taken with `auto`, and one line converts
// it to the common base -- an identity conversion on jazzy and kilted, an upcast
// on 1.5.x. That single line is what lets every create_subscription,
// create_publisher and parameter call in the filter be spelled once.
//
// CONSTRUCT WITH THE THREE-ARGUMENT FORM `(name, ns, options)`. It is the only
// one both classes have: `nav2_util::LifecycleNode` declares just
// `(name, ns = "", options = {})` (lifecycle_node.hpp:47-50), and the
// two-argument `(name, options)` overload exists only on 1.5.x
// (nav2_ros_common/lifecycle_node.hpp:87-90). `make_shared<HostNode>(name, opts)`
// therefore compiles on one line of three and fails on the other two.
// ─────────────────────────────────────────────────────────────────────────────
#if HUBOT_NAV2_HAS_ROS_COMMON
using HostNode = nav2::LifecycleNode;
#else
using HostNode = nav2_util::LifecycleNode;
#endif

// ─────────────────────────────────────────────────────────────────────────────
// (3) THE TRANSFORM BUFFER.  NOT a nav2 difference -- an rclcpp deprecation.
//
// `nav2::create_transform_buffer()` is ten lines that make a `tf2_ros::Buffer`
// (which is what `nav2::TransformBuffer` IS -- tf2_factories.hpp:37) and give it
// a timer interface. Reproduced here rather than depended on, because
// nav2_ros_common does not exist before 1.5.0.
//
// The one fork inside it is upstream's own, and it is about rclcpp rather than
// nav2: `tf2_ros::CreateTimerROS`'s two-interface constructor is `[[deprecated]]`
// at rclcpp 32 (lyrical) in favour of a NodeInterfaces one that does not exist
// at rclcpp 28 (jazzy) or 29 (kilted) -- create_timer_ros.hpp:54-57 on the older
// lines, :62-71 on lyrical. The guard is copied from nav2's own
// tf2_factories.hpp:88-93 so the two cannot drift in opposite directions.
// ─────────────────────────────────────────────────────────────────────────────
template<typename NodeT>
inline std::shared_ptr<tf2_ros::Buffer> createTransformBuffer(
  const NodeT & node,
  rclcpp::CallbackGroup::SharedPtr callback_group = nullptr)
{
  auto buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
#if RCLCPP_VERSION_GTE(30, 0, 0)
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(*node, callback_group);
#else
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node->get_node_base_interface(), node->get_node_timers_interface(), callback_group);
#endif
  buffer->setCreateTimerInterface(timer_interface);
  return buffer;
}

// ─────────────────────────────────────────────────────────────────────────────
// (3b) JOINING A RELATIVE NAME TO THE COSTMAP'S PARENT NAMESPACE.
//
// ⚑ THIS ONE SPLITS jazzy FROM kilted, AND IT IS THE ONLY DIFFERENCE ON THIS
// LIST THAT CHANGES WHAT THE FILTER DOES RATHER THAN HOW IT IS SPELLED.
//
// `Layer::joinWithParentNamespace()` is public at kilted (layer.hpp:180) and
// lyrical (:179) and DOES NOT EXIST ANYWHERE IN jazzy's nav2 headers -- grepped
// across the whole of /opt/ros/jazzy/include, 2026-09-07, zero hits. So the
// port is not one boundary at 1.5.0; the pre-1.5.0 line is itself two lines.
//
// The filter needs it. A costmap layer is hosted by a node whose namespace
// already carries the costmap's own name (`/tb4/global_costmap`), so a relative
// name written by an integrator resolves one level too deep unless it is joined
// against the PARENT. Dropping the join on jazzy would not be a smaller
// feature; it would silently point four topic names and every configured target
// node at addresses nobody subscribes to -- the exact defect this package fixed
// on 2026-09-06 and documented at zone_parameter_filter.cpp:429.
//
// So on jazzy it is reproduced, and reproduced from UPSTREAM'S OWN BODY rather
// than reinvented -- nav2_costmap_2d/src/layer.cpp:121-135 at release tag
// 1.4.2, read this turn. Where the member exists it is CALLED, not copied, so
// two of the three distros track upstream and only the one that has nothing to
// track runs our transcription. `namespaced_target_resolution_test.cpp` is the
// oracle for it and runs on all three.
//
// The pair below is ranked-overload dispatch, not a preprocessor guard: the
// `int` overload is viable only where the member exists and wins when it is,
// and the `long` overload is the fallback. That keeps the choice on the same
// fact the compiler is already checking, and needs no probe.
// ─────────────────────────────────────────────────────────────────────────────
namespace detail
{
template<typename LayerT, typename NodeT>
inline auto joinParentNamespace(LayerT * self, const NodeT &, const std::string & topic, int)
->decltype(self->joinWithParentNamespace(topic))
{
  return self->joinWithParentNamespace(topic);
}

template<typename LayerT, typename NodeT>
inline std::string joinParentNamespace(
  LayerT *, const NodeT & node, const std::string & topic, long)
{
  // Verbatim from nav2_costmap_2d/src/layer.cpp:128-134 at tag 1.4.2, including
  // the `topic[0]` on a possibly-empty string: `operator[](0)` on an empty
  // std::string is defined and yields '\0', which is not '/', so an empty name
  // takes the join branch on every distro. Callers guard emptiness before
  // calling for exactly that reason.
  if (topic[0] != '/') {
    std::string node_namespace = node->get_namespace();
    std::string parent_namespace = node_namespace.substr(0, node_namespace.rfind("/"));
    return parent_namespace + "/" + topic;
  }
  return topic;
}
}  // namespace detail

/// Join `topic` to the parent namespace of the costmap node hosting `self`.
template<typename LayerT, typename NodeT>
inline std::string joinParentNamespace(
  LayerT * self, const NodeT & node, const std::string & topic)
{
  return detail::joinParentNamespace(self, node, topic, 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// (4) LATCHED SUBSCRIPTION QoS.  A CONVENIENCE, WRITTEN OUT.
//
// Byte-for-byte what `nav2::qos::LatchedSubscriptionQoS` constructs
// (qos_profiles.hpp:70-84). Written as three explicit calls rather than a class
// name, because the calls say what the subscription actually asks for and the
// name does not: a reader had to open a nav2 header to learn that "latched"
// here means reliable + transient-local at a given depth.
// ─────────────────────────────────────────────────────────────────────────────
inline rclcpp::QoS latchedSubscriptionQoS(int depth = 10)
{
  // Braces, not parentheses: `rclcpp::QoS qos(rclcpp::KeepLast(depth));` is a
  // most-vexing-parse and declares a function. The compiler said so and this
  // draft had it wrong first.
  rclcpp::QoS qos{rclcpp::KeepLast(depth)};
  qos.reliable();
  qos.transient_local();
  return qos;
}

// ─────────────────────────────────────────────────────────────────────────────
// (5) DECLARE-OR-GET A PARAMETER.  A CONVENIENCE, WRITTEN OUT.
//
// `nav2::LifecycleNode::declare_or_get_parameter<T>()` exists only on 1.5.x
// (nav2_ros_common/lifecycle_node.hpp:115-141). The behaviour it names is three
// lines of rclcpp that have been stable across every distro this package
// targets: declare it if it is not declared, then read it. A costmap filter is
// loaded into somebody else's node, and the same parameter may already have
// been declared by another plugin or set by an override -- which is the whole
// reason the "or get" half exists.
//
// Deliberately a free function taking the node, not a member: a caller then
// cannot accidentally reach a derived class's shadowing overload instead.
// ─────────────────────────────────────────────────────────────────────────────
template<typename T, typename NodeT>
inline T declareOrGetParameter(
  const NodeT & node,
  const std::string & name,
  const T & default_value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter(name, rclcpp::ParameterValue(default_value));
  }
  return node->get_parameter(name).template get_value<T>();
}

}  // namespace hubot

#endif  // HUBOT__NAV2_COMPAT_HPP_
