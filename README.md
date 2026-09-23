# CommonRoad-CXX-Port

A C++20 port of the CommonRoad vehicle models from the Technical University of Munich (dynamic single-track, single-track drift and multi-body), with a small SDL2/ImGui viewer.
The CMake project and the static library it builds are both called `velox`.
On top of the ported models, `velox` adds a simulation daemon that runs steering and longitudinal controllers and a low-speed safety layer, and returns a telemetry struct after every step.
The library also contains a standalone loss-of-control detector (`lib/simulation/loss_of_control_detector.*`), which the daemon does not use.

## What is ported

| Upstream Python (`PYTHON/vehiclemodels/`) | C++ in this repository |
| --- | --- |
| `vehicle_dynamics_st`, `vehicle_dynamics_std`, `vehicle_dynamics_mb` (7, 9 and 29 states) | `lib/models/vehiclemodels/src/vehicle_dynamics_{st,std,mb}.cpp` |
| `init_st`, `init_std`, `init_mb` | `lib/models/vehiclemodels/src/init_{st,std,mb}.cpp` |
| `utils/tire_model.py`, `utils/vehicle_dynamics_ks_cog.py` | `lib/models/tire_model.cpp`, `lib/models/vehicle_dynamics_ks_cog.cpp` |
| steering and acceleration constraints, parameter classes | header-only equivalents in `lib/models/*.hpp` |
| vehicle parameter sets 1 to 4, tyre parameters | `parameters/vehicle/`, `parameters/tire/` |

The controllers, safety layer, detector, model timing, telemetry and daemon were written for this project and are not part of upstream CommonRoad.

## Package layout

- `CMakeLists.txt`: builds the `velox` static library, the examples, the optional tests and the SDL2/ImGui demo app.
- `lib/`: headers and implementation of the public API (controllers, models, simulation, telemetry, logging and error types).
- `parameters/`: vehicle parameter sets 1 to 4 (YAML files and C++ headers), the tyre parameters and the parameter loader.
- `config/`: YAML files for the controllers, the low-speed safety profiles and the loss-of-control detector.
- `app/`: SDL2 + ImGui demo that drives the simulation only through the daemon API.
- `examples/`: minimal headless programs built against `velox`.
- `tests/`: C++ tests, the Python/C++ scenario comparison and its reference data.
- `PYTHON/`: the upstream CommonRoad Python package (version 3.0.2) used as the numerical reference, trimmed to the ST, STD and MB models, plus Python prototypes of the controllers and safety layer (`vehiclemodels/sim/`, `vehiclemodels/config/`) written for this project.
- `docs/`: `safety_pipeline.md`, earlier notes on the safety layer that are partly out of date (they describe a model the repository does not have).
- `web-sdk/`: an in-progress TypeScript port of the runtime. No `package.json` or build configuration is committed yet.
- `vehicleModels_commonRoad.pdf`: the upstream model documentation.

## Building

Requirements:

- CMake 3.20 or newer and a C++20 compiler.
- yaml-cpp.
- SDL2. `CMakeLists.txt` calls `find_package(SDL2 REQUIRED)` unconditionally, so SDL2 must be installed even for a library-only build.
- Python 3 when tests are enabled. The `scenario_consistency` test also imports `omegaconf` (`pip install -r PYTHON/requirements.txt`).
- Dear ImGui sources in `third_party/imgui`, for `commonroad_app` only (see below).

```bash
cmake -S . -B build -DBUILD_VELOX_TESTS=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

| CMake option | Default | Effect |
| --- | --- | --- |
| `BUILD_VELOX_EXAMPLES` | `ON` | Builds `basic_sim_daemon`, `direct_control_demo` and `drift_mode_demo`. |
| `BUILD_VELOX_TESTS` | `OFF` | Builds the test executables and `scenario_simulator`, and registers the CTest tests. |

Targets:

- `velox`: static library containing the daemon, controllers, telemetry and utility code.
- `commonroad_app`: SDL2/ImGui viewer that runs the simulation through `SimulationDaemon`. Built only when the ImGui sources are present.
- `basic_sim_daemon`: minimal example that steps the daemon and prints telemetry. It requests a 0.2 s `dt` on purpose, so the sub-stepping warning appears.
- `direct_control_demo`: drives the ST model in direct control mode with a fixed steering angle and axle torque.
- `drift_mode_demo`: headless example that flips the drift safety profile on and off to show the oversteer allowances.
- `scenario_simulator` and the `test_*` executables: built only with `BUILD_VELOX_TESTS=ON`.

`basic_sim_daemon` and `direct_control_demo` pass relative `config` and `parameters` paths to the daemon, so run them from the repository root, for example `./build/basic_sim_daemon`.
`drift_mode_demo` sets no roots and throws `ConfigError` wherever it is run (see the status table).

### ImGui and the demo app

`third_party/imgui` is committed as a gitlink to [ocornut/imgui](https://github.com/ocornut/imgui) commit `c254db7`, but the repository has no `.gitmodules`, so a clone leaves the folder empty and `git submodule` commands cannot fetch it.
CMake then prints `ImGui sources not present; skipping commonroad_app build` and builds everything else.
To build the app, supply ImGui at that commit yourself:

```bash
git clone https://github.com/ocornut/imgui.git third_party/imgui
git -C third_party/imgui checkout c254db763709e12eeabea54c52631c2b972b7d58
cmake -S . -B build
cmake --build build --target commonroad_app
```

## Verification

Two tests check the ported models against the upstream Python implementation in `PYTHON/`, whose ST, STD and MB model files are unchanged from upstream.

- `test_derivatives` evaluates the ST, STD and MB right-hand sides at the state and input from the upstream unit test (`PYTHON/unit_tests/test_derivatives.py`: vehicle 2 parameters, steering rate 0.15 rad/s, acceleration 0.63 g) and compares every element with the reference vector. The absolute tolerance per element is 1e-13 for ST and STD and 1e-7 for MB. The ST and MB references are upstream's own. Upstream's STD reference does not match upstream's `vehicle_dynamics_std.py`, so the STD reference here was regenerated by running that unmodified function.
- `scenario_consistency` runs `scenario_simulator` and the Python reference on the ST model only (vehicle 2, initial speed 15 m/s) over four 1 s input traces (coasting, braking, accelerating, and a sinusoidal steering-rate and acceleration sweep; 100 samples at 10 ms) with the same RK4 step, and fails if any state or derivative differs by more than 5e-8. `scenario_simulator` writes its CSV to 10 decimal places, so the test cannot resolve differences below 5e-11. In the run below, all four scenarios sat at that floor (largest 5.0e-11).

`test_zeroInitialVelocity` follows the upstream zero-initial-velocity scenarios: MB and ST start from standstill and run for 1 s (RK4, `dt` = 1e-4 s) under rolling, braking, accelerating and steering inputs, and each final state must be within 1e-2 of a stored reference.
The ST roll-out with zero input must stay exactly at its initial state.
Only the ST braking and accelerating references match upstream, whose values come from scipy's `odeint`; the stored MB references differ from upstream's, so treat this test as a regression check on the C++ code.

The other tests cover `init_mb` and `init_std`, the steering and longitudinal controllers, the low-speed safety layer and its drift profile, config loading, telemetry, the daemon and the header-only model utilities.
`test_timestep_bounds` runs STD and MB for 2000 steps at their `max_dt` and checks the state stays finite, then sweeps requested `dt` values for STD in drift mode, checking that every sub-step is at most `max_dt` and that requests above `max_dt` are split into several sub-steps.
`test_public_api_no_gears` checks that `simulation_daemon.hpp` and `telemetry.hpp` do not contain the string `GearSelection`.
`test_vehicle` only writes ST trajectories to a CSV file for manual comparison and makes no assertions.

With both CMake options on, 19 tests are registered with CTest (18 with `BUILD_VELOX_EXAMPLES=OFF`, which drops `drift_mode_demo`).
`tests/test_loss_of_control_detector.cpp` exists but is not registered.

### Status

At commit `cf8989a`, 14 of the 19 tests pass (13 without `omegaconf`); the other five currently fail.

## Public API overview

### Configuration and parameters
- `velox::io::ConfigManager` loads the controller YAML files, the per-model low-speed safety profiles, an optional `model_timing.yaml` (falling back to the built-in timing table) and the vehicle parameters. Its default parameter root is the source tree's `parameters/` directory, compiled in as `VELOX_PARAM_ROOT`, and the config root defaults to the sibling `config/` directory.
  The constructor validates both roots up front, throwing `ConfigError` if either path is missing or not a directory. If `config_root` is empty, `ConfigManager` uses the `config/` directory next to `parameter_root`.
- `velox::models::VehicleParameters` holds physical properties (mass, wheelbase, tyre data, steering limits, etc.) and is populated by the configuration manager.

### Simulation daemon
- `velox::simulation::SimulationDaemon` owns the model interface, controllers, safety system, and a `VehicleSimulator`. Construct it with `SimulationDaemon::InitParams` to select the model, vehicle id, configuration root, and parameter root.
- `InitParams` does not inherit the `ConfigManager` default roots. An empty `parameter_root` makes the constructor throw `ConfigError`, so always set `config_root` and `parameter_root`.
- `InitParams::control_mode` selects the default control path (keyboard-style throttle/brake + steering nudge vs. direct torque/angle inputs); it defaults to `Keyboard` and can be overridden per-reset through `ResetParams::control_mode`.
- `reset(const ResetParams&)` can change the model, vehicle, control mode, drift setting and `dt`. It rebuilds the controllers, safety layer and simulator, sets the initial state through `init_st`, `init_std` or `init_mb`, and zeroes the distance, energy and time totals.
- `step(const UserInput&)` advances the simulation by one request (internally sub-stepping as required by timing constraints) and returns a `telemetry::SimulationTelemetry` snapshot. An overload taking `std::vector<UserInput>` steps through a batch and returns one snapshot per input.
- `snapshot()` returns the last telemetry, current state vector, timestep, and accumulated simulated time without mutating the simulation.
- `telemetry()` returns a const reference to the last telemetry without stepping. It is not synchronised, so another thread must not read it while `step()` or `reset()` is running.

### User input and timing
- `UserInput` encodes throttle/brake, steering nudge, drift toggles, requested timestep `dt`, the caller's timestamp, and control-mode specific fields (steering angle and axle torques for direct control). `clamped()` calls `validate()` first, which throws `InputError` for any non-finite or out-of-range field, so input outside `UserInputLimits` (defaults in `kDefaultUserInputLimits`) is rejected rather than clamped. Always set `dt`: the default of 0 is rejected.
- The control mode is set only through `InitParams` or `ResetParams`. `step()` replaces any `control_mode` in `UserInput` with the daemon's current mode and reads the fields for that mode.
- `UserInput` has no gear or transmission fields.
- `ModelTiming` raises any requested `dt` below 1 ms (`kMinStableDt`) to 1 ms, then splits it into ceil(dt / `max_dt`) equal sub-steps, with the last one taking the rounding remainder. `max_dt` is a `float`, so a 20 ms request on STD becomes three sub-steps, not two. Built-in limits: MB 5 ms, ST 10 ms nominal and 20 ms maximum, STD 10 ms. `SimulationDaemon::step()` does this itself, so pass the frame time as `dt`; any clamping or sub-stepping is reported through the log sink.

### Telemetry schema
The daemon reports a `telemetry::SimulationTelemetry` struct on every step:
- `pose`: world-frame `x`, `y`, and `yaw`.
- `velocity`: scalar speed plus longitudinal, lateral, yaw-rate, and global-frame components.
- `acceleration`: longitudinal and lateral accelerations.
- `traction`: body, front and rear slip angles, lateral force saturation and the drift-mode flag.
- `steering`: desired/actual angles and rates after steering filtering.
- `controller`: commanded acceleration, throttle, brake, and drive, brake, regen, hydraulic, drag and rolling forces.
- `powertrain`: total, drive and regen torque, mechanical and battery power, and state of charge (`soc`).
- `front_axle`/`rear_axle`: drive, brake and regen torque and normal force per axle, plus speed, slip ratio and friction utilisation for each wheel.
- `totals`: cumulative distance travelled, energy consumption (joules), and simulated time.
- `detector_severity`, `safety_stage` (`Normal`, `Transition` or `Emergency`) and `detector_forced`: set by the low-speed safety layer. Severity is the larger of |yaw rate| / yaw-rate limit and |slip angle| / slip-angle limit for the active profile, and `detector_forced` is true when it exceeds 1.0. Despite the names, none of these come from `LossOfControlDetector`.
- `low_speed_engaged`: whether the safety latch is active. It engages below the profile's engage speed or when severity exceeds 1.0, and releases above the release speed.

Use `telemetry::to_json` to serialise the payload or `telemetry::draw_telemetry_imgui` for debug UI rendering.

### Drift mode and low-speed safety
- The low-speed safety layer clamps yaw rate and slip angle to the active profile's limits, scaled down with speed below the release speed. In the `Transition` stage it blends yaw rate toward the kinematic value implied by the steering angle, and in the `Emergency` stage it sets yaw rate to zero. It also zeroes negative wheel speeds, and wheel speeds below `stop_speed_epsilon` (0.05) while latched, in transition or below the engage speed. With drift on and the stage at `Normal`, yaw rate and slip angle are left unclamped.
- Each model has a normal and a drift profile in `config/low_speed_safety*.yaml`, and the drift one has higher yaw-rate and slip-angle limits (ST: 0.9 and 0.65, against 0.6 and 0.45). `config/low_speed_safety_std.yaml` sets `drift_enabled: true`, so STD starts in drift mode; the ST and MB files set it to false.
- Enable or disable drift mode via `SimulationDaemon::InitParams::drift_enabled`, `ResetParams::drift_enabled`, or a `UserInput::drift_toggle` value ≥ `0.5`. Calling `SimulationDaemon::reset` without a drift override restores the model's configured default and reinitialises controller integrators and safety latches.
- `drift_mode_demo` runs the STD model twice, drift off then on, and fails unless the drift pass reaches a higher peak yaw rate and slip angle. It then runs a brake-and-steer input in each mode and prints severity and safety stage. At `cf8989a`, with the roots set, the car stays below about 0.13 m/s, every logged frame is in the `Emergency` stage and peak yaw rate is 0 in both passes, so the check fails (see the status table). With tests enabled it is registered as a CTest test of the same name.

### Error handling and logging
- Errors raised by the daemon, the config manager and the timing code derive from `velox::errors::VeloxError` (`ConfigError`, `InputError`, `SimulationError`). The `VELOX_LOC`, `VELOX_MODEL` and `VELOX_CONTEXT` macros add the file and line, plus the model or extra context, to the message.
- Bad input throws `InputError` and bad configuration throws `ConfigError`. `step()` and `reset()` rethrow any other `std::exception` as a `SimulationError` naming the model and the action.
- The vehicle parameter loader (`parameters/vehicle_parameters.cpp`) throws a plain `std::runtime_error` for a missing YAML file. The `SimulationDaemon` constructor loads the parameters outside its rethrow block, so that error escapes unwrapped; catch `std::exception` as well.
- The daemon logs warnings about `dt` sub-stepping or clamping to the 1 ms minimum, and about the controllers' acceleration, steering-rate and steering-angle limits. When `InitParams::log_sink` is empty, the constructor installs `logging::make_console_log_sink()`, which writes to `std::clog`. To send warnings elsewhere, set `init.log_sink` to your own `logging::LogSinkPtr` or call `daemon.set_log_sink(...)`.

Minimal construction (run from the repository root, or use absolute roots):

```cpp
velox::simulation::SimulationDaemon::InitParams init{};
init.config_root    = "config";
init.parameter_root = "parameters";
velox::simulation::SimulationDaemon daemon(init);
```

## SDL demo controls

- **Throttle/Brake**: `W`/`↑` for throttle, `S`/`↓` for proportional braking (scaled by "Keyboard brake bias"), `Space` for full braking.
- **Steering**: `A`/`←` left, `D`/`→` right.
- **Reset**: `R` resets the simulator to the current model/vehicle; `Esc` quits.

CLI/config flags:

- `--control-mode {keyboard|direct}` (alias `--mode`) selects how inputs are interpreted by the daemon.
- `--steering-angle <rad>` supplies a fixed steering angle when running in direct mode.
- `--axle-torque <Nm>` may be provided once per driven axle (front then rear); missing entries default to zero torque.
- `--input-flags <path>` (or `--control-config`) loads the above fields from a YAML file, for example:

```yaml
control_mode: direct
steering_angle: 0.05
axle_torques: [500.0, 500.0]
```

## Attribution

The vehicle models, their parameter sets and the Python reference implementation come from the CommonRoad vehicle models by the Professorship of Cyber-Physical Systems at the Technical University of Munich:

- Source repository: [gitlab.lrz.de/tum-cps/commonroad-vehicle-models](https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models)
- Python package: [commonroad-vehicle-models on PyPI](https://pypi.org/project/commonroad-vehicle-models/) (the copy in `PYTHON/` is version 3.0.2)
- Model documentation: M. Althoff and G. Würsching, *CommonRoad: Vehicle Models (Version 2020a)*, included here as `vehicleModels_commonRoad.pdf`
- Project site: [commonroad.in.tum.de](https://commonroad.in.tum.de/)

The upstream code under `PYTHON/` keeps its own BSD 3-Clause licence in `PYTHON/LICENSE.txt` (Copyright 2020 Technical University of Munich, Professorship of Cyber-Physical Systems).
The C++ models are translations of that code and the YAML files in `parameters/vehicle/` and `parameters/tire/` are unmodified copies of upstream's; `NOTICE` carries TUM's notice for both.
Dear ImGui and SDL2 come under their own licences.

## Licence

The code written for this port is released under the BSD 3-Clause licence; see `LICENSE`, with the upstream attribution in `NOTICE`.
Upstream CommonRoad code in `PYTHON/` is covered by `PYTHON/LICENSE.txt`.
