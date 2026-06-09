# Installation

Odyssey is a source template: you copy its files into a PROS project. This
keeps everything readable and hackable — there is no precompiled library to
fight with.

## Prerequisites

- [PROS](https://pros.cs.purdue.edu/v5/getting-started/index.html) installed
  (the VS Code extension is the easiest way)
- A VEX V5 brain and at least a drivetrain with left/right motors
- Strongly recommended: a V5 inertial sensor (IMU)

## 1. Create a PROS project

Using the PROS VS Code extension: **PROS → Create Project → V5**.

Or from a terminal:

```sh
pros c create my-robot v5
```

## 2. Copy Odyssey in

From this repository, copy:

| From (this repo)    | To (your project)     |
| ------------------- | --------------------- |
| `include/odyssey/`  | `include/odyssey/`    |
| `src/odyssey/`      | `src/odyssey/`        |
| `example/main.cpp`  | `src/main.cpp`        |

The PROS build system compiles everything under `src/` automatically — no
Makefile changes are needed.

## 3. Build

```sh
pros make
```

The example `main.cpp` will not match your robot yet — that's the next step:
[Configuration](configuration.md).

!!! tip "One include"
    Everything in the library is available through a single header:

    ```cpp
    #include "odyssey/api.hpp"
    ```

## Updating

Because Odyssey is plain source, updating is just re-copying
`include/odyssey/` and `src/odyssey/` from a newer version of this
repository. Your configuration lives in `main.cpp`, so it is never
overwritten.
