# Boids: Flocking Simulation

A real-time, two-dimensional simulation of collective behaviour (flocks of birds, schools of fish, swarms of insects) written in C++ with SFML for the graphics. Each boid follows three local rules, **separation**, **alignment** and **cohesion**, and flocks emerge spontaneously from these interactions.


University of Bologna, programming project. Authors: Leandro Milazzo, Raffaele Milia, Alessandro Morselli, Luca Oliverio.

## Features

- Agent-based model with separation, alignment and cohesion rules, with user-defined parameters.
- Interactive mouse force: attractive by default, repulsive while the left button is held down. A coloured circle around the cursor shows its area of influence (green = attractive, red = repulsive).
- Boids can be added (`Space`) or removed (`R`) while the simulation is running.
- Boids turn from white to red as their speed increases.
- Frame-rate independent motion through a time step `dt`.
- Summary statistics of the system printed to the terminal once per second.
- Unit tests written with [doctest](https://github.com/doctest/doctest).

## Requirements

- A C++ compiler
- [CMake](https://cmake.org/) (the commands below use the Ninja Multi-Config generator, so Ninja is needed as well)
- [SFML](https://www.sfml-dev.org/)

## Build

From the project folder (`Progetto_RLLA`):

```bash
cmake -S . -B build -G "Ninja Multi-Config"
cmake --build build --config Debug
```

Use `--config Release` for the optimised version. This produces two executables:

| Executable | Purpose |
|---|---|
| `boids_sim` | the graphical simulation |
| `boids_sim.t` | the unit tests |

## Run

```bash
build/Debug/boids_sim      # simulation (use build/Release/... for the Release build)
build/Debug/boids_sim.t    # unit tests
```

At start-up the program asks for the following parameters in the terminal (suggested values in brackets). Invalid values are rejected.

| Parameter | Meaning |
|---|---|
| `n_b` (200) | number of boids (positive integer) |
| `d` (100) | interaction distance: boids within this distance influence each other |
| `d_s` (20) | separation distance: below this distance two boids repel each other |
| `s` (1.5) | strength of the separation (repulsive) force |
| `a` (0.04) | strength of the alignment force |
| `c` (0.3) | strength of the attraction towards the centre of mass |

A 1600×900 window then opens, limited to 90 FPS (easy to change in `main.cpp`).

### Controls

| Input | Action |
|---|---|
| Mouse (no button) | attracts nearby boids |
| Left mouse button (held) | repels nearby boids |
| `Space` | add boids |
| `R` | remove boids |

## Code structure

| File | Content |
|---|---|
| `boids_logic.hpp` | `Boid` struct (position and velocity as `std::array<double, 2>`), `Movement` class, declarations of the logic and drawing functions |
| `boids_logic.cpp` | implementation of the rules (`rule1`, `rule2`, `rule3`), `update`, mouse force and boundary/velocity limits |
| `boids.test.cpp` | doctest unit tests |
| `main.cpp` | parameter input and validation, main loop, graphics and user interaction |

## Testing

The tests cover the helper functions (`diff_pos2`, `get_speed`, `add_inplace`, `is_neighbor`), the three movement rules, the dynamic update (`apply_neighbor_rules`, `apply_mouse_force`, `update_pos_vel`, `update`) and the limit checks (`check_sides`, `limit_velocity`), including edge cases.

## Notes

Generative AI (ChatGPT) was used for the SFML syntax, for the mouse-interaction functions and for some additional tests.
