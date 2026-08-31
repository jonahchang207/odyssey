# Odyssey — Speaker Notes

Talking points for the presentation in `presentation/index.html` (15 slides).
Plain language, meant to be spoken, not read off the screen. Aim for about
7–9 minutes of talking, then take questions. Approximate times per slide are in
brackets; adjust as needed. The same notes are also available inside the
slideshow by pressing **N**.

Delivery reminders:
- Talk to the audience, not the slides. Glance, then look up.
- Slide 9 is the live demo. Leave time for it and invite someone up to drive.
- Press **F** for fullscreen, arrow keys to move, **N** to toggle these notes.

---

## Slide 1 — Title  [15 sec]
- Introduce yourself and the project in one sentence: Odyssey is a custom C++
  library that lets a VEX V5 robot track its own position on the field and
  drive itself to target coordinates.
- Mention there is a live demo later that the class can try.

## Slide 2 — Project overview  [45 sec]
- Odyssey is a library written in C++ for a VEX V5 competition robot running
  PROS (the programming environment for the V5 brain).
- It continuously tracks the robot's position as (x, y, heading) and turns
  that into simple commands such as "drive to this coordinate facing this
  direction."
- The end result is reusable: it installs on any team robot and has its own
  documentation site.

## Slide 3 — Motivation  [60 sec]
- Explain the core problem: most autonomous code just runs the motors for a
  fixed time or distance. There is no feedback, so a bump, a low battery, or a
  slightly crooked start ruins the whole routine.
- The objective was a robot that knows its actual position and corrects toward
  the target every time.
- Three reasons for choosing it: reliability in matches, wanting to understand
  the math rather than use a black-box library, and making something reusable
  for the team.

## Slide 4 — Project goals  [45 sec]
- Walk the six goals briefly: track position accurately (about one inch over a
  routine), drive to coordinates, stay working if a sensor fails, install
  easily, be documented, and be customizable.
- Note that these are the goals defined before starting, and the presentation
  comes back to whether they were met.

## Slide 5 — Learning on my own  [60 sec]
- This material is not taught in any class taken so far, so it was
  self-directed.
- Sources: the 5225A Pilons position-tracking paper for the theory, the
  open-source LemLib and EZ-Template libraries for how to structure things,
  and the PROS documentation for the hardware API.
- The standard followed: do not use a formula that cannot be explained or
  derived. Re-derived the math by hand before trusting it.

## Slide 6 — How it works  [75 sec]
- Give the intuition, not the code. Two free-spinning tracking wheels (one
  forward, one sideways) and an inertial sensor measure the motion.
- Every 10 milliseconds the program reads how far each wheel rolled and how
  much the robot turned, models that small movement as a slice of a circle (an
  arc), rotates it into field coordinates, and adds it to the running total.
- This happens 100 times a second. It builds a position estimate from geometry
  alone, with no external signal — point at the diagram while explaining.

## Slide 7 — Readable code excerpts  [45 sec]
- Do not read the code line by line. Make one point: the hard math is hidden
  inside, and what a teammate actually writes is a few plain commands.
- Left: the core step that turns one tiny arc into a position update. Right:
  the same library used in five readable lines.

## Slide 8 — Current capabilities  [60 sec]
- Quickly name the features: odometry with sensor fallback, PID motions, the
  boomerang controller for reaching a position and heading at once, pure
  pursuit path following, asynchronous motions, and driver control.
- Then the bigger point: this is past a minimum product. It is a real,
  installable library with versioned releases, an automated build pipeline,
  and a documentation website. The original goals were met.

## Slide 9 — Interactive simulator (live demo)  [2–3 min]
- This is the main demo. State clearly: this runs the same arc-based odometry
  algorithm as the C++ library, in the browser.
- Explain the two robots: the copper one is the true position; the blue dashed
  one is where odometry thinks it is. With clean sensors they overlap.
- Drive it with the arrow keys to show tracking working. Then turn on Sensor
  noise and drive a lap to show the estimate drift apart — this is the
  real-world problem the library is built to fight.
- Optionally press Self-drive to show the robot navigating a square using only
  its own estimate.
- Invite a student up to drive. This satisfies the "let the class control it"
  requirement.

## Slide 10 — Obstacles and solutions  [75 sec]
- Be honest and specific. Four obstacles:
  1. The laptop cannot compile the robot code at all (no ARM toolchain) — set
     up a cloud build pipeline that compiles every change.
  2. Two coordinate systems (math radians vs. compass degrees) caused wrong
     motion — fixed by using one convention internally and converting only at
     the edges.
  3. AI suggestions often used the wrong conventions or an old API — used them
     to understand ideas, then reimplemented within the project.
  4. Small heading errors compound into large position errors — handled with
     the IMU plus fallbacks (this is what the noise demo showed).

## Slide 11 — Testing and iteration  [60 sec]
- Odometry is only useful if accurate, so it was tested against physical
  measurements.
- Tests: the square test (drive a known square, measure the real end position
  vs. the reported one), the spin-in-place test (position should not move,
  which isolates the wheel offsets), iterative PID tuning, the simulator as a
  visual test bench, and automated build and documentation checks.
- Key diagnostic to mention: drift while spinning means the wheel offsets are
  wrong; drift while driving straight means the wheel size is wrong.

## Slide 12 — What I learned  [45 sec]
- Broaden out: applied trigonometry and coordinate transforms, concurrency and
  mutexes, PID control theory, how to package and distribute software, writing
  documentation for other users, and reading other people's source code.

## Slide 13 — Future work  [45 sec]
- The current version meets its goals, but there is a clear next version.
- Mention: motion profiling for smoother motion, expanding the simulator into
  a real off-robot tool, automated unit tests, and combining all sensors with
  a Kalman filter instead of using one at a time. Be willing to say some parts
  would be rebuilt from scratch.

## Slide 14 — References and AI use  [30 sec]
- Credit the sources: the 5225A Pilons paper, LemLib and EZ-Template, the PROS
  docs, the path editor, and the tooling.
- State plainly: AI was used, with careful prompting, to help build most of the
  program and this presentation — explaining concepts, drafting code and
  content, and finding bugs — while the design, conventions, and final
  implementation were directed and reviewed throughout.

## Slide 15 — Thank you / Questions  [remaining time]
- One-sentence summary: a robot that knows where it is, built from
  trigonometry, an inertial sensor, and iterative testing.
- Invite questions, and offer to let anyone drive the simulator.

---

## Anticipated questions (prepare answers)

- **What is odometry?** Tracking the robot's position over time by adding up
  small measured movements, instead of using an external positioning system.
- **Why not just use GPS or the VEX GPS sensor?** The GPS sensor needs the
  field wall codes and can be blocked or noisy; odometry works anywhere from
  the robot's own wheels and gyro, and the two can be combined later.
- **Why arcs instead of straight lines each step?** A robot turning while
  moving follows a curve. Treating each tick as a tiny arc (with the chord
  correction) is far more accurate than assuming straight segments.
- **What happens if a wheel slips or the robot is hit?** Tracking wheels are
  unpowered, so they slip far less than drive wheels; a hard hit still adds
  error, which is why heading comes from the IMU and errors are minimized but
  not eliminated.
- **How accurate is it?** About an inch over a full routine with a good sensor
  setup; the square and spin tests are how that was verified.
- **What is PID?** A control method that converts how far off the target you
  are into a smooth motor command that settles without overshooting.
- **How long did it take / how is it shared?** It is distributed as an
  installable PROS template through GitHub releases, so any team can add it in
  two commands.
- **Did the AI write it for you?** AI helped explain concepts, draft pieces,
  and find bugs, but it frequently produced the wrong coordinate convention or
  outdated API; the design and final code were directed and rewritten to fit
  the project.
