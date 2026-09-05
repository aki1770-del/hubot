// Copyright (c) 2026 Komada (aki1770-del)
// SPDX-License-Identifier: Apache-2.0
//
// ⚑ WHY THIS TEST EXISTS (written before the act).
//
// The upstream nav2 filter threw std::runtime_error from
// checkPendingParameterUpdates(), which is called at the top of process().
// process() is reached from CostmapFilter::updateCosts() — bare — from
// LayeredCostmap, which has no try/catch anywhere. Measured on the released
// `lyrical` branch, 2026-09-05. So a failed parameter set aborted the whole
// navigation node.
//
// ⚑ HONEST BOUND, stated here rather than in a commit message: the case that
// matters is an INTEGRATION case — a real target node that rejects a parameter,
// driven through a live executor. That test is OWED and is NOT here. What is here
// is the construction-level guarantee plus the degrade flag's contract. A suite
// that cannot fail on the real defect has measured nothing (CLAUDE.md §0), so
// this file must NOT be read as proving the abort is gone.

#include <gtest/gtest.h>
#include <memory>
#include "hubot/zone_parameter_filter.hpp"

TEST(ZoneParameterFilterDegrade, StartsUndegraded)
{
  hubot::ZoneParameterFilter filter;
  EXPECT_FALSE(filter.enforcementDegraded())
    << "a freshly constructed filter must not claim the zone is unenforced";
}

TEST(ZoneParameterFilterDegrade, AccessorIsConstAndCheap)
{
  const hubot::ZoneParameterFilter filter;
  EXPECT_FALSE(filter.enforcementDegraded());
}

// ⚑ OWED, and named so nobody mistakes its absence for a pass:
//   1. a target node that returns successful=false -> enforcementDegraded() true,
//      an ERROR on the log, and NO exception escaping process();
//   2. the same, with the service throwing, via a rejected future;
//   3. a reload clears the flag because the configuration it referred to is gone.
// These need a live executor and a second node. FBR/BDE own the reproduction.
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
