# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

ROS 2 (Humble) Python package that drives a BlinkStick USB LED strip via the `/led` topic. The package depends on the `susumu_ros2_interfaces` package for the `LED` message type — this is an external sibling package that must be cloned into the same workspace (`ros2_ws/src`) before building.

## Build / Run / Test

Workspace-level commands (run from `~/ros2_ws`):

```bash
# Build only this package
colcon build --packages-select susumu_blinkstick_ros2
source install/setup.bash

# Run the node
ros2 run susumu_blinkstick_ros2 blinkstick_node
# or via launch (sets led_count=8 by default)
ros2 launch susumu_blinkstick_ros2 blinkstick_node.launch.py

# Lint / style tests (ament_flake8, ament_pep257, ament_copyright)
colcon test --packages-select susumu_blinkstick_ros2
colcon test-result --verbose

# Run a single test file directly
python -m pytest test/test_flake8.py
```

Runtime parameters (declared in `blinkstick_node.py`):
- `led_count` (int, default 8) — set at startup only; not honored on dynamic param updates.
- `brightness` (double, default 0.5) — supports dynamic update via `ros2 param set /blinkstick_node brightness <0.0–1.0>`; out-of-range values are rejected by `parameter_callback`.

## Architecture

Three-layer pipeline, all wired together in `blinkstick_node.py`:

1. **`BlinkstickNode`** (`blinkstick_node.py`) — ROS 2 node. Subscribes `/led` (`susumu_ros2_interfaces/LED`), parses `color1`/`color2` strings (named colors + hex) into RGB tuples, instantiates the matching `BaseAnimation` subclass, and hands it to the manager. The pattern string is matched via a long `if/elif` chain — add new patterns there *and* in `animations.py`.

2. **`AnimationManager`** (`animation_manager.py`) — owns a background daemon thread (`_worker_loop`) that runs a fixed-interval (`frame_interval=0.05`) frame loop. New animations arrive via a `queue.Queue`; the manager only switches to the new animation if `new_anim.priority > current.priority` (strictly greater — equal priority does **not** preempt). When an animation finishes (`elapsed > duration`) or is preempted, `turn_off()` is called before the next one starts. `STOP` is implemented as a no-op animation with a very high priority sent by the user — there is no dedicated stop API; it is just the priority rule applied.

3. **`LEDController`** (`animation_manager.py`) — thin wrapper around `blinkstick.find_first()`. Maintains an in-memory `colors` buffer and only writes to hardware in `update_hardware()`, which throttles writes to ~33 Hz (30 ms min interval) unless `force_update=True`. `apply_trail(decay_rate)` is what produces the "afterglow" effect — `BaseAnimation.draw_frame` calls it every frame before delegating to the subclass's `_draw_pattern`.

4. **`animations.py`** — `BaseAnimation` defines the `draw_frame → apply_trail → _draw_pattern` template. Subclasses implement `_draw_pattern(dt, led_controller, elapsed)`. Movement-based animations (`LeftToRight`, `RightToLeft`, `Wave`, `Inward`, `Outward`) advance `self._old_pos` by `speed * dt` and sub-step the path to avoid skipping LEDs at high speed. `_apply_brightness` is applied by each animation before writing — `brightness` is not applied centrally.

### Adding a new animation

1. Add a subclass of `BaseAnimation` in `animations.py` implementing `_draw_pattern`.
2. Import it in `blinkstick_node.py` and add an `elif pat == "NEW_NAME":` branch.
3. Document it in `README.md` (the `/led` topic doc is the user-facing API surface).

## External dependencies & hardware setup

- Requires the `blinkstick` Python package installed via pip from the upstream repo (not apt). See `README.md` § "BlinkStickを使うための準備".
- USB access without root requires the udev rule for vendor `20a0:41e5` documented in `README.md`.
- Known intermittent USB error: `[Errno 5] Input/Output Error` from `blinkstick._usb_ctrl_transfer`. The project's empirical workaround is to connect via a USB hub rather than directly to the PC port; root cause is unknown. There is a commented-out `set_error_reporting(False)` in `LEDController.__init__` that silences (but does not fix) it.
