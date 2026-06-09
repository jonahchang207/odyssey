# Pure Pursuit Paths

Pure pursuit follows a pre-drawn path by continuously steering toward a
"lookahead point" — the spot where a circle around the robot intersects the
path. The result is smooth, fast, multi-point motion that a chain of
`moveToPoint` calls can't match.

## Drawing a path

Use [path.jerryio](https://path.jerryio.com/):

1. Set the format to **LemLib Odom Code Gen v0.4** (Odyssey reads the same
   format: `x, y, speed` lines ending with `endData`).
2. Draw your path on the field image.
3. Adjust the speed profile (the generator bakes deceleration into the
   waypoint speeds).
4. Download the `.txt` file.

## Running a path from the SD card

Copy the file to a micro SD card (FAT32), insert it into the brain, and:

```cpp
void autonomous() {
    chassis.setPose(/* match the path's starting point! */);

    //             file          lookahead  timeout
    chassis.follow("skills.txt", 12,        15000);
}
```

To run a path **backwards** (robot drives in reverse along it):

```cpp
chassis.follow("retreat.txt", 12, 5000, false);
```

## Paths in code

No SD card? Define waypoints directly — each is `(x, y, speed)` with speed
in motor power (-127 to 127):

```cpp
std::vector<odyssey::Waypoint> path = {
    {0, 0, 80},
    {0, 24, 100},
    {12, 36, 100},
    {24, 40, 60},
    {36, 40, 40}, // slow into the final point
};
chassis.follow(path, 12, 8000);
```

Space waypoints a few inches apart — pure pursuit interpolates between them,
but speed lookup uses the nearest point.

## Choosing the lookahead distance

The lookahead is the radius of the circle used to pick the steering target,
in inches:

- **Smaller (6-10)** — hugs the path tightly, but can oscillate or feel
  jerky.
- **Larger (15-20)** — smooth and stable, but cuts corners.
- **Start at 12** and adjust per path. Fast, gentle paths tolerate large
  lookaheads; tight maneuvers need small ones.

!!! warning "Pure pursuit does not stop precisely"
    The follower ends when the robot reaches the neighborhood of the final
    waypoint, while still moving at the path's final speed. End your paths
    at low speed, and follow with a `moveToPoint`/`moveToPose` if you need
    an exact final position.

## Mid-path actions

`follow` is async like every other motion, so the `waitUntil` pattern works
on distance traveled along the path:

```cpp
chassis.follow("score.txt", 12, 10000);
chassis.waitUntil(30); // 30 inches into the path
intake.move(127);
chassis.waitUntilDone();
```
