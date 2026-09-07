// hubot_live_stack — A REAL COSTMAP LAYER THAT DECLARES THE FULL WINDOW.
//
// ⚑ WITHOUT THIS, THE HARNESS RUNS AND DRIVES THE FILTER ZERO TIMES.
//
// Measured in nav2 lyrical, layered_costmap.cpp:
//
//   * `CostmapFilter::updateBounds()` takes min_x/min_y/max_x/max_y and IGNORES
//     all four (costmap_filter.cpp -- the parameters are literally commented
//     out in the signature). A filter never expands the update window.
//   * `LayeredCostmap::updateMap()` therefore leaves minx_ at
//     numeric_limits::max() and maxx_ at numeric_limits::lowest() when no
//     PLUGIN declared bounds, which clamps to x0 = size-1, xn = 1.
//   * Every call to `(*filter)->updateCosts(...)` sits inside
//     `if (xn >= x0 && yn >= y0)`. With x0=9, xn=1 that branch is never taken.
//
// So a live stack with `plugins: []` comes up, spins map_update_thread_ at
// 5 Hz, publishes a costmap, and exercises the filter NOT ONCE -- while looking
// entirely healthy. That is the same defect class this whole harness exists to
// remove, wearing a different costume, and it would have been invisible.
//
// hubot's own in-process suite already guards this with a hand-added
// `FullWindowBoundsLayer` (pluginlib_live_costmap_test.cpp:226 --
// "Production always has a layer ... window and never calls updateCosts() at
// all"). The difference here is that this one is loaded BY the real
// Costmap2DROS THROUGH pluginlib from the `plugins:` list, not appended by a
// fixture.
//
// It writes no costs. It only declares the window, so every observed effect
// stays attributable to the filter under test.

#include <algorithm>

#include <stdexcept>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace hubot_live_stack
{

class FullWindowBoundsLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override
  {
    // ⚑ THIS LINE IS NOT BOILERPLATE. `Layer::initialize()` (layer.cpp) sets
    // layered_costmap_, name_, tf_, node_, clock_, logger_ and then calls
    // onInitialize() -- it NEVER touches `enabled_`, which is declared
    // `bool enabled_;` at layer.hpp:200, uninitialized. Every concrete nav2
    // layer declares it itself (inflation_layer.cpp:94). A layer that skips
    // this reads an indeterminate bool.
    //
    // Measured here 2026-09-06: it read FALSE, so updateBounds() returned
    // early, LayeredCostmap computed `Updating area x: [9, 1] y: [9, 1]`, and
    // the `if (xn >= x0 && yn >= y0)` guard was never satisfied. The whole
    // stack came up green -- lifecycle configure and activate rc=0,
    // map_update_thread_ logging "Map update time" every 200ms at a measured
    // 5.000 Hz, footprint publishing -- and the filter was driven ZERO times.
    // hubot's own liveness signal is what caught it, from inside the stack:
    //   "NOT WATCHING. This filter has not been driven even once".
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error{"FullWindowBoundsLayer: failed to lock node"};
    }
    enabled_ = node->declare_or_get_parameter(name_ + "." + "enabled", true);
    current_ = true;
  }

  void reset() override {}

  bool isClearable() override {return false;}

  // Declare the whole rolling window around the robot. This is what an
  // obstacle/static layer does in production; here it is the ONLY thing done,
  // so nothing but the filter can move a cost.
  void updateBounds(
    double robot_x, double robot_y, double /*robot_yaw*/,
    double * min_x, double * min_y, double * max_x, double * max_y) override
  {
    if (!enabled_) {
      return;
    }
    const nav2_costmap_2d::Costmap2D * cm = layered_costmap_->getCostmap();
    const double half_x = cm->getSizeInMetersX() / 2.0;
    const double half_y = cm->getSizeInMetersY() / 2.0;
    // Only ever WIDEN. Narrowing is what triggers nav2's "Illegal bounds
    // change" warning (layered_costmap.cpp), and a layer that trips it would
    // put noise in the log this harness exists to read.
    *min_x = std::min(*min_x, robot_x - half_x);
    *min_y = std::min(*min_y, robot_y - half_y);
    *max_x = std::max(*max_x, robot_x + half_x);
    *max_y = std::max(*max_y, robot_y + half_y);
    current_ = true;
  }

  void updateCosts(
    nav2_costmap_2d::Costmap2D & /*master_grid*/,
    int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/) override
  {
    // Deliberately empty. See the file header.
  }
};

}  // namespace hubot_live_stack

PLUGINLIB_EXPORT_CLASS(hubot_live_stack::FullWindowBoundsLayer, nav2_costmap_2d::Layer)
