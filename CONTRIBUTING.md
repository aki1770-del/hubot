# Contributing

**Report anything here: https://github.com/aki1770-del/hubot/issues** — including the demo
on the front page not doing what the front page says. No issue has ever been opened on this
repository, so you would be the first; the forms exist so you do not have to guess what we
need. **Not security.** A vulnerability goes to [SECURITY.md](SECURITY.md) instead, not to a
public tracker.

## Four facts make a report actionable without a round trip

| | how to get it |
|---|---|
| **ROS 2 distribution** | `echo $ROS_DISTRO` |
| **nav2 version** | `ros2 pkg xml nav2_costmap_2d \| grep -m1 version` — or `apt list --installed 2>/dev/null \| grep nav2-costmap-2d` |
| **architecture** | `uname -srm` |
| **the `zone_decision` transcript** | `ros2 topic echo /zone_decision` — paste what it printed |

The transcript is the one that saves the most time. Reporting what it knows is this
package's entire job, so what it said is usually the shortest route to what went wrong.
**If it printed nothing at all, say that** — silence on that topic is a finding, not a
missing field, and it is one of the harder cases to reach any other way.

## Pull requests

Welcome. `.github/workflows/gate.yml` runs on every pull request and has to pass: it builds
against a released nav2, runs the tests, and runs `test/prose_matches_tree.py`, which
re-derives from the tree the figures the documents claim about it — so a number you change
in the code has to be right in the prose too. Run both locally first if you can.

## What you get back

An answer from a person, and a straight one, including "no" or "we do not know". **We are
not promising a turnaround**, because nothing has ever been reported here and there is no
record to promise from. A slow reply is not a verdict on your report.
