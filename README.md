my mental model of system so far.
there is premade ros nodes that are setup to run in sim env.
we're in the process of making each nodes in sim stanalone in hardware.
there is

## repo map
DOC: documents, referece
scalecar-vfg-h-infinite: sim code from Prof. we're refactoring/modifying this codes to work on actual hardware.
tools/path_gen: parametric path generator + lat/lon overlay viewer for the rooftop test track.
tools/indoor_test: short indoor sample curves for shaking out the system before going to the rooftop.

## indoor sample curve

A 5x5 m square workspace with 0.5 m margin on every edge (usable area
4x4 m). The robot starts at the left side at (0.5, 2.5) in the local
`odom` frame, facing +x, and follows a sinusoid to (4.5, 2.5).

Geometry
- y(x) = 2.5 + 0.7 * sin(2*pi*(x - 0.5) / 4),  x in [0.5, 4.5]
- one full period over 4 m of forward travel
- y stays in [1.8, 3.2]  (1.3 m clear of every wall)
- arc length 5.02 m
- |kappa|_max = 1.727  ->  R_min = 0.579 m  (above LIMO mechanical
  steering limit ~0.37 m, but with margin)
- rho_max = |kappa| * v <= 1.73 even at v = 1.0 m/s, well within the
  LPV scheduling envelope (rho_max = 5)

Files
- `tools/indoor_test/publish_indoor_path.py` -- ROS2 publisher, latched
  QoS (transient_local + reliable + keep_last 1) on `/reference_path`
  in frame `odom`, 81 waypoints.
- `tools/indoor_test/run_indoor.sh` -- launches `path_follower_node`
  with `v_const:=<speed>` and the publisher together.
- `tools/indoor_test/out/indoor_path.png` -- workspace overlay +
  curvature plot.

Run on the NUC (after rsync):
```
cd /home/agilex/H-infinity/tools/indoor_test
./run_indoor.sh 0.3        # speed in m/s, default 0.3
```

Place the robot at roughly (x=0.5, y=2.5) facing +x in the `odom`
frame before starting. Ctrl+C stops both the controller and the path
publisher.
