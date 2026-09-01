# Installation

Odyssey installs like LemLib does: as a **PROS template** — a versioned
package containing the compiled library and its headers that the PROS
conductor manages for you. You can also vendor the raw source if you want to
hack on the library itself.

!!! abstract "You will finish this page with"
    Odyssey applied to an existing PROS project, ready for you to copy and
    adapt the example chassis configuration. No local ARM toolchain is needed
    when you use the released template.

## Prerequisites

- [PROS](https://pros.cs.purdue.edu/v5/getting-started/index.html) installed
  (the VS Code extension is the easiest way)
- A VEX V5 brain and at least a drivetrain with left/right motors
- Strongly recommended: a V5 inertial sensor (IMU)

## Option A — depot install (recommended)

Register the Odyssey depot once, then install it into any PROS project:

```sh
# one-time: tell the PROS conductor where Odyssey lives
pros c add-depot odyssey https://raw.githubusercontent.com/jonahchang207/odyssey/depot/stable.json

# inside your PROS project:
pros c apply odyssey
```

The first command registers Odyssey's release feed on your computer. Run it
once. The second applies the latest stable template to the PROS project in your
current directory.

!!! success "Check the install"
    Open the project's `include/odyssey/` folder and confirm the public headers
    are present, then add `#include "odyssey/api.hpp"` to `src/main.cpp`.
    `pros make` should recognize the library when you build the project.

Upgrading later is one command:

```sh
pros c upgrade odyssey
```

## Option B — manual template install

1. Download `odyssey@X.Y.Z.zip` from the
   [latest release](https://github.com/jonahchang207/odyssey/releases/latest).
2. Register it with the conductor:

    ```sh
    pros c fetch odyssey@X.Y.Z.zip
    ```

3. Apply it inside your PROS project:

    ```sh
    pros c apply odyssey
    ```

## Option C — vendor the source

If you want to modify the library itself, copy the source straight into your
project instead of using the template:

| From (this repo)    | To (your project)     |
| ------------------- | --------------------- |
| `include/odyssey/`  | `include/odyssey/`    |
| `src/odyssey/`      | `src/odyssey/`        |

The PROS build system compiles everything under `src/` automatically.

## Set up `main.cpp`

However you installed, copy the example robot configuration from
[`src/main.cpp`](https://github.com/jonahchang207/odyssey/blob/main/src/main.cpp)
in this repository into your project's `src/main.cpp`, and continue to
[Configuration](configuration.md) to adapt it.

```sh
pros make
```

!!! tip "One include"
    Everything in the library is available through a single header:

    ```cpp
    #include "odyssey/api.hpp"
    ```

!!! warning "Apply the template inside a PROS project"
    If the conductor cannot find a project, confirm your terminal is open in
    the directory containing `project.pros`, then run `pros c apply odyssey`
    again.

## How the template is built

Every GitHub release automatically builds `odyssey@X.Y.Z.zip` (headers +
compiled `firmware/odyssey.a`) with the `Build PROS template` workflow and
updates the depot, so the depot always points at the newest stable release.
You never need a local ARM toolchain unless you're developing Odyssey itself.

<div class="odyssey-next" markdown>

<p><strong>Installed?</strong><br>Choose your sensor layout and take the measurements the configuration needs.</p>

[Continue to hardware setup →](hardware.md)

</div>
