// Copyright (c) 2026 Komada (aki1770-del)
// SPDX-License-Identifier: Apache-2.0
//
// ⚑ THE PARENT. It exists to be ALIVE when the child is not.
//
// WHY (written before the act):
//
// hubot's whole claim is that it DEGRADES where upstream ABORTS, and upstream's abort
// IS process death. Every oracle we had for that claim lived inside the process whose
// survival was the question:
//
//   test/degrade_at_production_caller_test.cpp  EXPECT_NO_THROW around updateCosts()
//
// That assertion is only reached if control returns to it. abort(), exit(), a
// terminate raised on another thread, or a deadlock skip it entirely — the binary
// dies or hangs and the assertion renders no verdict at all. And its topology drives
// updateCosts() on the gtest main thread with the executors spun on that same thread,
// while production drives it on Costmap2DROS's map-update thread with the executor
// elsewhere; the exception that kills the node escapes THE COSTMAP THREAD, which that
// test does not have.
//
// The obvious repair — a gtest death test asserting ExitedWithCode(0) — is the one
// that cannot work, and we measured why on 2026-09-05: EXPECT_EXIT forks WITHOUT
// exec, fork clones only the calling thread, and every mutex another thread held is
// held forever in the child. A death test asserting a CRASH survives that because its
// child aborts before it needs those mutexes. A death test asserting SURVIVAL needs
// the child to keep working. It hung for three minutes.
//
//     A death test can reliably observe a CRASH. It cannot reliably observe SURVIVAL.
//
// The missing primitive is therefore not "a subprocess" — we had that. It is
//
//     A PROCESS BOUNDARY CREATED BY exec, NOT BY fork,
//
// so the child starts with one thread and a clean process image and builds its own
// ROS graph. fork+execv gives exactly that. The child (zpf_survival_probe) drives the
// real CostmapFilter::updateCosts() on a dedicated bare thread and reports through an
// exit-code contract; this parent adjudicates it from outside, where a SIGABRT, a
// silent exit, and a hang are three DIFFERENT and individually nameable outcomes
// rather than one indistinguishable red.
//
// WHAT THIS DOES NOT CLAIM. The child is a gtest-free ROS process, not a running
// controller_server and not a robot. Fidelity gained over the in-process test: the
// real production caller, the production two-thread topology, no handler anywhere in
// the escape path, and an oracle that outlives the failure. Fidelity still missing: a
// real Costmap2DROS update loop, a real navigation stack, hardware. UNVERIFIED there,
// never cleared.

#include <sys/types.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>

#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#ifndef ZPF_SURVIVAL_PROBE_PATH
#error "ZPF_SURVIVAL_PROBE_PATH must be defined by CMake -- the parent must know its child"
#endif

namespace
{

constexpr char kBeacon[] = "ZPF_SURVIVAL_BEACON_OK";

struct ProbeResult
{
  bool exited_normally = false;   // WIFEXITED
  int exit_code = -1;             // WEXITSTATUS, valid only if exited_normally
  bool killed_by_signal = false;  // WIFSIGNALED
  int signal_number = 0;          // WTERMSIG
  bool timed_out = false;         // we killed it; it never finished
  std::string output;

  bool hasBeacon() const {return output.find(kBeacon) != std::string::npos;}

  // One line, so a red is diagnosable rather than merely red.
  std::string describe() const
  {
    if (timed_out) {return "TIMED OUT (hung) -- killed by the harness";}
    if (killed_by_signal) {
      return std::string("DIED by signal ") + std::to_string(signal_number) +
             " (" + strsignal(signal_number) + ")";
    }
    if (exited_normally) {return "exited with code " + std::to_string(exit_code);}
    return "unknown wait status";
  }
};

// fork + execv. The exec is the load-bearing half: it replaces the process image, so
// the child has one thread and none of this process's held mutexes. That is the
// difference between this and a gtest death test.
ProbeResult runProbe(const std::string & mode, std::chrono::seconds timeout)
{
  ProbeResult r;

  char tmpl[] = "/tmp/zpf_probe_out_XXXXXX";
  const int out_fd = mkstemp(tmpl);
  if (out_fd < 0) {
    r.output = "harness: mkstemp failed";
    return r;
  }
  const std::string out_path = tmpl;

  const pid_t pid = fork();
  if (pid < 0) {
    close(out_fd);
    unlink(out_path.c_str());
    r.output = "harness: fork failed";
    return r;
  }

  if (pid == 0) {
    // Child, pre-exec. Only async-signal-safe work here.
    dup2(out_fd, STDOUT_FILENO);
    dup2(out_fd, STDERR_FILENO);
    close(out_fd);
    std::string path = ZPF_SURVIVAL_PROBE_PATH;
    std::string arg = mode;
    char * const argv[] = {path.data(), arg.data(), nullptr};
    execv(path.c_str(), argv);
    _exit(127);  // exec failed
  }

  close(out_fd);

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  int status = 0;
  bool reaped = false;
  while (std::chrono::steady_clock::now() < deadline) {
    const pid_t w = waitpid(pid, &status, WNOHANG);
    if (w == pid) {reaped = true; break;}
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  if (!reaped) {
    kill(pid, SIGKILL);
    waitpid(pid, &status, 0);
    r.timed_out = true;
  } else if (WIFEXITED(status)) {
    r.exited_normally = true;
    r.exit_code = WEXITSTATUS(status);
  } else if (WIFSIGNALED(status)) {
    r.killed_by_signal = true;
    r.signal_number = WTERMSIG(status);
  }

  if (FILE * f = fopen(out_path.c_str(), "r")) {
    char buf[4096];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {r.output.append(buf, n);}
    fclose(f);
  }
  unlink(out_path.c_str());
  return r;
}

}  // namespace

// ── THE TEST THE PACKAGE'S CENTRAL CLAIM WAS MISSING ─────────────────────────────
//
// It goes red on ALL of: the child aborts, the child exits non-zero, the child hangs,
// the child survives but stops working, and the child survives without ever having
// been rejected. The in-process oracle can report only the last two, and only if it
// is reached.
TEST(ZpfSurvival, TheProcessSurvivesTheRejection_AndIsStillEnforcingZones)
{
  const auto r = runProbe("reject", std::chrono::seconds(90));

  EXPECT_FALSE(r.timed_out)
    << "the child hung. Survival is not 'did not crash' -- a wedged navigation node "
       "helps her no more than a dead one.\n" << r.output;

  EXPECT_FALSE(r.killed_by_signal)
    << "THE PROCESS DIED. This is upstream's behaviour and the exact thing hubot "
       "exists to abolish: " << r.describe() << "\n" << r.output;

  ASSERT_TRUE(r.exited_normally)
    << "no clean exit -- " << r.describe() << "\n" << r.output;

  EXPECT_EQ(r.exit_code, 0)
    << "probe contract: 10=never active 11=update thread stopped turning 12=rejection never "
       "exercised (vacuous) 13=read-only parameter modified 14=alive but no longer "
       "enforcing.\n" << r.output;

  EXPECT_TRUE(r.hasBeacon())
    << "the beacon is printed only after every check passes; its absence means the "
       "run did not reach the end.\n" << r.output;
}

// ── ⚑ THE SECOND ABORT PATH, WHICH IS NOT A MUTANT AND IS NOT COVERED ────────────
//
// CPP 2026-09-05, found by reading src/zone_parameter_filter.cpp this turn:
// applyState() still THROWS std::runtime_error on an unknown state (:394), and
// process() calls it bare (:333, :369). checkPendingParameterUpdates() was made to
// degrade; THIS path was not. So the same kill chain is still open on a second entrance:
// a mask cell carrying an id no state declares.
//
// That is reachable by an ordinary mask-authoring mistake, and the package's README
// claim ("degrades where upstream aborts") does not hold for it. This test is written
// to STATE the truth it finds rather than to assert a wish: it is the instrument
// above, pointed at the other path.
TEST(ZpfSurvival, TheProcessSurvivesAnUndeclaredMaskState)
{
  const auto r = runProbe("unknown-state", std::chrono::seconds(90));

  EXPECT_FALSE(r.killed_by_signal)
    << "an UNDECLARED MASK VALUE still kills the process. applyState() throws at "
       "zone_parameter_filter.cpp:394 and process() calls it bare -- the same chain "
       "the degrade fix closed for parameter-set failures was left open here: "
    << r.describe() << "\n" << r.output;

  EXPECT_FALSE(r.timed_out) << "the child hung.\n" << r.output;

  ASSERT_TRUE(r.exited_normally) << r.describe() << "\n" << r.output;
  EXPECT_EQ(r.exit_code, 0) << r.output;
  EXPECT_TRUE(r.hasBeacon()) << r.output;
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
