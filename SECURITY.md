# Security

This is a nav2 costmap filter. It sits on a safety path — it is the thing that says whether a
speed limit a mapped zone asked for actually took — so a defect here can end with a robot
running at a speed someone believed had been reduced. **It is QM-class software, it is not
certified to anything, and it does not stop your robot**; it reports, and you hold the stop.

## Reporting

**Email aki1770@gmail.com.** That is the maintainer address already published in
`package.xml`, and it is the channel that works today. Please do not open a public issue for
a vulnerability first.

If you would rather use GitHub's private advisory form, look for **Report a vulnerability**
under this repository's **Security** tab. If it is not there, it is not switched on, and
email is the way in. That is written as something you check rather than something we assert,
so this page cannot go quietly out of date.

Useful in a report: what an attacker or a fault would have to control, what the component
then reports, and what a consumer acting on that report would do. A reproduction is welcome
and is not required — a clear description of the reachable path is enough to start.

## What you can expect

**No response time is promised, because there is nothing to promise from**: no vulnerability
has ever been reported here, and no advisory has ever been published. Stating a target we
have never met would be worth less than saying so.

What we will do: read it, tell you what we found, and tell you plainly if we disagree that
it is a vulnerability or if we are not going to fix it. If a fix ships, the CHANGELOG says
what it was — this project does not ship silent security fixes, because an integrator who
cannot see the fix cannot tell whether they need it.

## Versions

`0.1.1` is the only release that gets fixes. **`0.1.0` should not be used**: it names a tree
twelve commits older, it carries an undisclosed breaking change to the parameter namespace
join, and no automated gate ever ran on it.

## The failure class most worth reporting

The one that matters most here is **anything that makes the component report `enforced: yes`
when the limit is not on a target** — a value that is wrong in the direction of looking
healthy. Most of the design exists to close paths like that, several were found and closed
after they had shipped, and they are the ones a reader is least likely to notice unaided.
