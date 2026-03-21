# H-infinity LIMO Project Specification

## 1. Reasonability Check

This project is technically reasonable if it is treated as an integration and evaluation project, not a controller synthesis project.

Two scope corrections are important:

- The H-infinity controller itself is assumed to be provided by the professor. Controller derivation, gain design, and proof of stability are out of scope for this project unless additional materials are later supplied.
- The reusable ROS2 workspace in `../agile_ws` already provides robot bring-up, safety filtering, scenario orchestration, and rosbag recording, but it does not already contain an H-infinity control stack. It should be reused as infrastructure, not described as if the controller is already implemented there.

## 2. Project Goal

Integrate a professor-provided H-infinity controller into the AgileX LIMO ROS2 stack running on the Intel NUC mounted on the robot, then evaluate the controller's path-following performance against a practical manual baseline under repeatable test scenarios.

## 3. In Scope

- Integrate the provided controller into the existing ROS2 control pipeline on the LIMO.
- Reuse existing bring-up, safety, logging, and scenario infrastructure where practical.
- Define a canonical interface between the controller and the existing stack.
- Run repeatable experiments on the robot.
- Record and analyze controller performance from rosbag data.
- Compare controller behavior against a manual or existing non-H-infinity operating workflow.

## 4. Out of Scope

- H-infinity controller synthesis, tuning, or theoretical derivation.
- Claims of scientific superiority over PID, MPC, or other advanced controllers unless those controllers are also implemented and tested in the same ROS2 stack.
- Full localization redesign.
- Major changes to the LIMO base driver unless required for compatibility.

## 5. Deployment Context

- Target platform: Intel NUC mounted on the AgileX LIMO.
- Runtime environment: ROS2 on the robot-side computer.
- Reusable workspace source: `../agile_ws`.
- Intended execution location on the robot: user stated `~/agiles_ws`.
- Existing environment script currently sources `~/agilex_ws/install/setup.bash`.

This path mismatch must be resolved before implementation. The spec assumes the final deployed workspace path will be standardized during integration.

## 6. Existing Assets To Reuse

The following assets already exist in `../agile_ws` and should be treated as the base infrastructure for this project.

### 6.1 Bring-up and Platform Access

- `../agile_ws/start_ROS.sh`
  - Runs the environment setup and launches the robot stack.
- `../agile_ws/env_sanitizer.sh`
  - Sources ROS2.
  - Sources the installed robot workspace.
  - Sets `ROS_DOMAIN_ID=0`.
  - Sets `ROS_LOCALHOST_ONLY=1` for more reliable onboard control.

### 6.2 Robot Stack Launch

- `../agile_ws/src/limo_ros2/limo_base/launch/LIMO+MAVROS+RTK_Node_Launcher.launch.py`
  - Launches `limo_base`.
  - Remaps `odom` to `/wheel/odom`.
  - Launches `mavros` in namespace `pixhawk`.
  - Starts the standalone GNSS process.

### 6.3 Safety Path

- `../agile_ws/estop_cli.py`
  - Subscribes to `cmd_vel_raw`.
  - Publishes filtered `cmd_vel`.
  - Publishes `/estop`.
  - Forces zero velocity when E-stop is active.

This existing topic contract is central to the project and should be preserved unless there is a strong reason to redesign it.

### 6.4 Experiment Orchestration

- `../agile_ws/run_scenarios_from_files.py`
  - Loads scenario definitions from INI files.
  - Performs preflight checks for topics, publishers, subscribers, and message flow.
  - Can gate execution on RTK status in the existing GNSS workflow.
  - Publishes `/scenario_runner/status` and `/scenario_runner/event`.
  - Starts and stops the bag recorder.

For H-infinity controller evaluation, the existing RTK gating should be treated as optional and reused only if the experiment design actually depends on GNSS or outdoor positioning context.

### 6.5 Data Logging

- `../agile_ws/Data_Logger.py`
  - Wraps `ros2 bag record`.
  - Publishes `/data_logger/recording` and `/data_logger/health`.
  - Already records the current baseline topics:
    - `/gps_rtk_f9p_helical/gps/fix`
    - `/gps_rtk_f9p_helical/gps/nmea`
    - `/gps_rtk_f9p_helical/gps/rtk_status`
    - `/pixhawk/global_position/raw/satellites`
    - `/pixhawk/global_position/raw/fix`
    - `/pixhawk/gpsstatus/gps1/raw`
    - `/cmd_vel`
    - `/cmd_vel_raw`
    - `/wheel/odom`
    - `/imu`
    - `/estop`

### 6.6 Existing Motion Baseline

- `../agile_ws/limo_scenario_motion.py`
  - Publishes `cmd_vel_raw`.
  - Uses `/wheel/odom` for simple heading-hold and motion stopping logic.
  - Represents the current non-H-infinity motion behavior that can be used as a practical baseline reference.

## 7. System Architecture

The minimum intended integration architecture is shown below.

```mermaid
flowchart LR
    referenceInput[ReferencePathOrTrajectory] --> hinfController[HinfControllerNode]
    wheelOdom["/wheel/odom"] --> hinfController
    hinfController -->|"publishes /cmd_vel_raw"| estopNode[SafetyEstopNode]
    estopNode -->|"publishes /cmd_vel"| limoBase[LIMOBaseDriver]
    estopSignal["/estop"] --> orchestrator[ScenarioOrchestrator]
    hinfController --> statusTopics[ControllerStatusAndDebugTopics]
    orchestrator --> dataLogger[DataLogger]
    statusTopics --> dataLogger
    wheelOdom --> dataLogger
    estopSignal --> dataLogger
```

## 8. Software Architecture Decision

The project should preserve the existing command chain:

1. The H-infinity controller publishes `cmd_vel_raw`.
2. The existing safety layer filters that command and publishes `cmd_vel`.
3. The LIMO base consumes `cmd_vel`.

This design is preferred because it keeps the safety/E-stop logic outside the controller and reduces the risk that a controller-side fault bypasses stopping authority.

## 9. New Components Required

The following functionality is not yet present and must be added during implementation.

### 9.1 H-infinity Controller Runtime Node

A new node or adapter process is required to:

- receive the professor-provided controller implementation
- subscribe to robot state feedback
- consume a path or trajectory reference
- generate velocity commands for the existing control path
- publish diagnostics for evaluation

### 9.2 Reference Input Source

The H-infinity controller needs a reference to follow. The spec assumes one of the following:

- a path topic generated by a scenario runner
- a trajectory topic generated offline and published during tests
- a wrapper that converts the existing scenario format into a controller reference stream

If the professor-supplied controller expects a different interface, an adapter layer shall be used rather than rewriting the existing safety pipeline.

### 9.3 Evaluation Topic Extensions

The current logger is GNSS-oriented. Controller evaluation requires additional topics to be recorded.

## 10. Canonical Controller Interface Contract

This section defines the expected minimum ROS2 interface for integration. If the delivered controller uses different topic names or message types, a compatibility wrapper should be added so the rest of the system can remain stable.

### 10.1 Required Controller Inputs

| Topic | Type | Producer | Purpose |
|---|---|---|---|
| `/wheel/odom` | `nav_msgs/msg/Odometry` | `limo_base` | Main onboard feedback for pose, heading, and velocity |
| `/estop` | `std_msgs/msg/Bool` | `estop_cli.py` | Optional stop awareness inside the controller or wrapper |
| `/scenario_runner/reference_path` | TBD | scenario/reference publisher | Path or trajectory to follow |

Notes:

- The exact reference message may be `nav_msgs/msg/Path`, a custom waypoint message, or another professor-defined format.
- Because the delivered controller interface is not yet known, the reference topic name and type remain provisional.

### 10.2 Required Controller Output

| Topic | Type | Consumer | Purpose |
|---|---|---|---|
| `cmd_vel_raw` | `geometry_msgs/msg/Twist` | `estop_cli.py` | Raw motion command from controller to safety layer |

### 10.3 Recommended Diagnostic Outputs

| Topic | Type | Purpose |
|---|---|---|
| `/hinf_controller/status` | `std_msgs/msg/String` or custom status | Human-readable state and mode |
| `/hinf_controller/tracking_error` | custom or scalar messages | Cross-track and heading error logging |
| `/hinf_controller/reference` | same as reference input or derived | Records the active target used by the controller |
| `/hinf_controller/debug_cmd` | `geometry_msgs/msg/Twist` or custom | Optional pre-saturation or internal command debug |

These diagnostics are recommended because controller evaluation is much weaker if only `cmd_vel_raw` and odometry are logged.

## 11. Launch and Execution Contract

The expected run sequence on the Intel NUC is:

1. Start the ROS2 environment and robot bring-up.
2. Start the safety/E-stop node.
3. Start the H-infinity controller node or wrapper.
4. Start the scenario/reference publisher.
5. Start the recorder.
6. Execute the experiment.

### 11.1 Existing Launch Pieces To Preserve

- `start_ROS.sh` for platform bring-up.
- `estop_cli.py` for command filtering and stopping authority.
- `run_scenarios_from_files.py` for orchestration, if adapted for controller experiments.
- `Data_Logger.py` for bag recording.

### 11.2 Likely Integration Options

- Option A: extend the existing launch flow with an additional controller launch file.
- Option B: keep controller startup as a separate terminal/process during early integration.

Option B is acceptable for initial bring-up. Option A is preferred for repeatable evaluation runs.

If `run_scenarios_from_files.py` is reused for controller experiments, its GNSS-oriented preflight defaults should be adapted so that controller runs are not blocked by irrelevant RTK requirements.

## 12. Baseline Definition

The default comparison baseline for this project is a practical manual baseline, not a formal controller benchmark suite.

Recommended baseline order:

1. Existing scripted motion behavior from `limo_scenario_motion.py` where applicable.
2. Operator-guided manual path execution if scripted comparison is not sufficient.

This is acceptable for an engineering integration study, but it is scientifically weaker than comparison against well-defined automatic baselines such as PID or MPC under the same path-tracking task. The final report should state this limitation explicitly.

## 13. Experiment Design

The evaluation should progress in stages.

### 13.1 Stage 0: Static Safety Validation

Purpose:

- verify that the controller starts correctly
- verify that E-stop always overrides controller output
- verify that zero or bounded commands are produced when reference input is absent or invalid

### 13.2 Stage 1: Low-Speed Functional Tracking

Purpose:

- verify basic path following at conservative speed
- confirm that the controller can complete runs without safety intervention

Candidate scenarios:

- straight line path
- constant curvature path
- gentle lane-change or slalom path

### 13.3 Stage 2: Baseline Comparison

Purpose:

- compare H-infinity behavior against the practical manual baseline under the same route and speed envelope

Candidate comparisons:

- same start and end points
- same nominal path geometry
- same approximate speed envelope
- same test surface and environment

### 13.4 Stage 3: Robustness Checks

Purpose:

- observe controller behavior under less ideal conditions without intentionally creating unsafe situations

Candidate checks:

- moderate speed increase
- repeated runs across batteries or sessions
- mild trajectory changes
- temporary reference disturbances or restart conditions if safely supported

## 14. Performance Metrics

The project shall evaluate controller performance using controller-relevant metrics rather than GNSS-fix metrics.

### 14.1 Primary Metrics

- lateral tracking error
  - RMS error
  - maximum absolute error
- heading error
  - RMS error
  - maximum absolute error
- path completion success
  - completed or aborted
- settling or recovery time
  - after path curvature change, startup, or disturbance
- E-stop interventions
  - count and cause

### 14.2 Secondary Metrics

- control smoothness
  - total variation of linear velocity command
  - total variation of angular velocity command
- oscillation indicators
  - repeated sign changes or peak counts in heading error and angular command
- command effort
  - average and peak commanded speed and yaw rate
- run repeatability
  - variation of key metrics across repeated trials

### 14.3 Optional Metrics

- integrated absolute lateral error
- time inside a tolerance band
- energy or battery impact if instrumentation is available

## 15. Logging Requirements

The rosbag recorder should keep the current topics and add controller-evaluation topics.

### 15.1 Minimum Required Topics For Controller Evaluation

- `/cmd_vel`
- `/cmd_vel_raw`
- `/wheel/odom`
- `/imu`
- `/estop`
- `/scenario_runner/event`
- `/scenario_runner/status`
- controller reference topic
- controller status topic
- controller error topic or equivalent diagnostics

### 15.2 Recommended Additional Topics

- `/data_logger/recording`
- `/data_logger/health`
- GNSS topics if outdoor repeatability or location context is useful

### 15.3 Data Quality Rule

A run should not be treated as valid for analysis unless:

- the bag starts before motion begins
- the bag ends after motion finishes or aborts
- all required evaluation topics are present
- message flow is continuous enough for analysis
- the reason for abort, if any, is recorded

## 16. Scenario Specification Requirement

Controller evaluation scenarios should be stored as data, not hardcoded logic.

The scenario format should include:

- scenario name
- path type
- path parameters
- nominal speed
- maximum run time
- safety limits
- notes for experiment execution

If the current INI-based scenario files are reused, they may need to be extended to include path-following references rather than only motion parameters.

## 17. Safety Requirements

The project shall preserve the following safety rules.

- E-stop must have higher authority than the controller.
- The controller must not publish directly to the final base command topic unless the safety layer is explicitly redesigned and revalidated.
- Missing reference data shall default to safe behavior.
- Controller startup and shutdown behavior must be bounded and predictable.
- Test runs shall begin at low speed before expanding the operating envelope.

## 18. Acceptance Criteria

The project is considered successfully implemented when all of the following are satisfied.

### 18.1 Integration Acceptance

- The H-infinity controller can be started on the Intel NUC together with the existing LIMO ROS2 stack.
- The controller receives required feedback and reference inputs.
- The controller publishes `cmd_vel_raw`.
- The safety node continues to arbitrate motion through `cmd_vel`.

### 18.2 Safety Acceptance

- Manual E-stop always stops the robot.
- E-stop state is logged.
- Controller behavior during startup, stop, and abort is bounded and documented.

### 18.3 Data Acceptance

- Rosbags contain all required motion, controller, and event topics.
- Runs can be replayed offline for metric extraction.
- Scenario identity and run outcome are traceable from recorded data.

### 18.4 Evaluation Acceptance

- At least one low-speed path-following scenario is completed repeatedly without unsafe behavior.
- The same scenario is executed under the selected manual baseline.
- The final analysis reports the agreed tracking metrics and explicitly states the baseline limitations.

## 19. Risks and Constraints

- The professor-provided controller interface is currently unknown.
  - Mitigation: add a wrapper node rather than rewriting the surrounding ROS2 stack.
- The workspace path naming is inconsistent (`agile_ws`, `agiles_ws`, `agilex_ws`).
  - Mitigation: standardize deployment paths before implementation.
- `/wheel/odom` may drift and is not a high-accuracy ground truth source.
  - Mitigation: treat it as onboard feedback and relative evaluation data, not as absolute truth.
- The existing workspace contains GNSS-oriented evaluation logic.
  - Mitigation: reuse infrastructure only, and define new controller-specific metrics.
- A manual baseline is less rigorous than controller-to-controller comparison.
  - Mitigation: present results as an engineering evaluation unless stronger baselines are later implemented.

## 20. Deliverables

- H-infinity controller integration into the existing LIMO ROS2 stack
- repeatable launch and execution procedure on the Intel NUC
- rosbag dataset for controller evaluation
- offline analysis outputs for tracking metrics
- brief evaluation report summarizing controller behavior and comparison against the practical baseline

## 21. Immediate Next Implementation Targets

1. Confirm the professor-provided controller runtime interface.
2. Standardize the robot workspace path and environment sourcing.
3. Create the controller wrapper node if message types or topic names do not match the canonical contract.
4. Extend the recorder topic list for controller evaluation.
5. Define one low-speed reference path and complete an end-to-end trial.
