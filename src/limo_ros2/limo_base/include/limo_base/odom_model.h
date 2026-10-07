// H-infinity patch (2026-10-07): odometry for odom_model=hinf.
//
// Header-only and ROS-free so the same code can be compiled and replayed
// against recorded bags on a machine without ROS.
//
// The stock path (odom_model=agilex, limo_driver.cpp) has two defects, both
// measured against RTK on the June 2026 rooftop bags:
//  1. Pose yaw is the chassis IMU Euler yaw with every per-frame step under
//     0.1 deg dropped. At 100 Hz that loses about half of any heading change
//     slower than ~15 deg/s.
//  2. Position integrates the body velocity (v cos d, v sin d), where d is the
//     steering angle the driver believes (x2.47 too large in stock steering),
//     so the published pose is no fixed point of the body. After one 90 deg
//     step it sat 0.2-0.7 m from RTK while reporting "on the reference".
//
// This model uses measured quantities only: the chassis speed v and the IMU
// yaw psi (unwrapped, no deadband). The rear-axle centre of an Ackermann car
// has no lateral velocity, so it moves along psi; the published point sits
// point_x metres ahead of it on the body x axis. The default L/2 = 0.1 m is
// the CG the vendor simulator tracks (vfg_pathfollowing models/kinematic.py,
// l_r = 0.1). No steering angle enters, so the pose is the same in
// steering_mode agilex and direct.

#ifndef LIMO_BASE_ODOM_MODEL_H
#define LIMO_BASE_ODOM_MODEL_H

#include <cmath>

namespace AgileX {

class HinfOdometry {
public:
    explicit HinfOdometry(double point_x = 0.1) : point_x_(point_x) {}

    // Every IMU Euler yaw sample [deg, CCW positive], in arrival order.
    void updateYawDeg(double yaw_deg) {
        if (!yaw_init_) {
            yaw_last_deg_ = yaw_deg;
            yaw_unwrapped_deg_ = yaw_deg;
            yaw_init_ = true;
            return;
        }
        double d = yaw_deg - yaw_last_deg_;
        while (d > 180.0) d -= 360.0;
        while (d <= -180.0) d += 360.0;
        yaw_unwrapped_deg_ += d;
        yaw_last_deg_ = yaw_deg;
    }

    // One motion-state frame: chassis speed v [m/s], dt [s] since the previous
    // frame. Holds the origin until the first yaw sample has arrived.
    void step(double v, double dt) {
        if (!yaw_init_) return;
        const double psi = yawRad();
        if (!started_) {
            psi_prev_ = psi;
            started_ = true;
        }
        if (dt > 0.0) {
            const double psi_mid = 0.5 * (psi_prev_ + psi);
            // Rear-axle displacement along the mean heading, plus the change in
            // the lever arm from the rear axle to the published point.
            x_ += v * dt * std::cos(psi_mid) + point_x_ * (std::cos(psi) - std::cos(psi_prev_));
            y_ += v * dt * std::sin(psi_mid) + point_x_ * (std::sin(psi) - std::sin(psi_prev_));
        }
        psi_prev_ = psi;
    }

    double x() const { return x_; }
    double y() const { return y_; }
    double yawRad() const { return yaw_unwrapped_deg_ / 180.0 * M_PI; }
    bool ready() const { return yaw_init_; }

private:
    double point_x_;
    bool yaw_init_ = false;
    bool started_ = false;
    double yaw_last_deg_ = 0.0;
    double yaw_unwrapped_deg_ = 0.0;
    double psi_prev_ = 0.0;
    double x_ = 0.0;
    double y_ = 0.0;
};

}  // namespace AgileX

#endif  // LIMO_BASE_ODOM_MODEL_H
