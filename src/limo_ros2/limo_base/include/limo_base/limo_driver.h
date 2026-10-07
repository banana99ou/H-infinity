/*
 * Copyright (c) 2021, Agilex Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef LIMO_DRIVER_H
#define LIMO_DRIVER_H

#include <iostream>
#include <thread>
#include <memory>
#include <atomic>
#include <cstdlib>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/executor.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "std_msgs/msg/string.hpp"
#include "limo_msgs/msg/limo_status.hpp"
// #include <ros/ros.h>
// #include <tf/transform_broadcaster.h>
// #include <tf/tf.h>
// #include <nav_msgs/Odometry.h>
// #include <sensor_msgs/Imu.h>
// #include <limo_base/LimoStatus.h>
#include "limo_base/serial_port.h"
#include "limo_base/limo_protocol.h"
#include "limo_base/odom_model.h"

namespace AgileX {

class LimoDriver : public rclcpp::Node{
public:
    LimoDriver(std::string node_name);
    ~LimoDriver();
    void run();

private:
    
    void connect(std::string dev_name, uint32_t bouadrate);
    void readData();
    void processRxData(uint8_t data);
    void parseFrame(const LimoFrame& frame);
    void sendFrame(const LimoFrame& frame);
    void setMotionCommand(double linear_vel, double steer_angle,
                          double lateral_vel, double angular_vel);
    void enableCommandedMode();
    void processErrorCode(uint16_t error_code);
    void twistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    double normalizeAngle(double angle);
    double degToRad(double deg);
    double convertInnerAngleToCentral(double inner_angle);
    double convertCentralAngleToInner(double central_angle);
    void publishOdometry(double stamp, double linear_velocity,
                         double angular_velocity, double lateral_velocity,
                         double steering_angle);
    void publishLimoState(double stamp, uint8_t vehicle_state, uint8_t control_mode,
                          double battery_voltage, uint16_t error_code, int8_t motion_mode);
    void publishIMUData(double stamp);
    void publishConfig();

private:
    rclcpp::Node *node_;
    std::shared_ptr<SerialPort> port_;
    std::shared_ptr<std::thread> read_data_thread_;

    std::atomic<bool> keep_running_;

    std::string port_name_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string odom_topic_name_;

    bool pub_odom_tf_ = false;
    bool use_mcnamu_ = false;
    // H-infinity patch (2026-10-04). "agilex" = stock Ackermann path (inner
    // wheel angle, 28 deg clamp, / angle_scale). "direct" = send the bicycle
    // steering angle itself: this chassis steers to the raw value it receives
    // (fit 0.98x over 8894 bag samples), so the stock /2.47 delivered every
    // steering command ~2.5x too small and capped full lock at R ~1.0 m.
    // Default "direct" since 2026-10-07: every launch path (and every
    // odom_watchdog respawn) must come back with the experiment's steering,
    // not silently revert to stock.
    std::string steering_mode_ = "direct";
    double max_steering_rad_ = 0.408;   // direct mode clamp (stock 28 deg inner ~ 0.408 central)
    std::atomic<bool> direct_steering_{true};   // read by the serial thread (parseFrame)
    // H-infinity patch (2026-10-07). "agilex" = stock odometry (deadbanded IMU
    // yaw, position crabbed by the believed steering angle). "hinf" = rear axle
    // integrated along the raw IMU yaw, published at odom_point_x_ ahead of it
    // (see odom_model.h). Read-only: switching mid-run would jump the pose.
    std::string odom_model_ = "hinf";
    double odom_point_x_ = 0.1;         // m ahead of the rear axle (sim CG, l_r = L/2)
    bool hinf_odom_ = true;
    HinfOdometry hinf_odometry_;
    double node_start_unix_ = 0.0;      // in /limo_base/config: a respawn shows as a new value
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr config_publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
    // Stock yaw state. Zero-initialised: the vendor left these indeterminate
    // (a shadowed local in publishIMUData meant to seed them never did).
    double present_theta_ = 0.0, last_theta_ = 0.0, delta_theta_ = 0.0,
           real_theta_ = 0.0, rad = 0.0;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<limo_msgs::msg::LimoStatus>::SharedPtr status_publisher_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_cmd_sub_;
    // rclcpp::Subscription<scout_msgs::msg::ScoutLightCmd>::SharedPtr
    //   light_cmd_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

    // ros::Publisher odom_publisher_;
    // ros::Publisher status_publisher_;
    // ros::Publisher imu_publisher_;
    // ros::Subscriber motion_cmd_sub_;
    // tf::TransformBroadcaster tf_broadcaster_;

    double position_x_ = 0.0;
    double position_y_ = 0.0;
    double theta_ = 0.0;

    ImuData imu_data_;
    uint8_t motion_mode_;  // current motion type

    static constexpr double max_inner_angle_ = 0.48869;  // 28 degree
    static constexpr double track_ = 0.172;           // m (left right wheel distance)
    static constexpr double wheelbase_ = 0.2;         // m (front rear wheel distance)
    static constexpr double left_angle_scale_ = 2.47;
    static constexpr double right_angle_scale_ = 2.47;
};

}

#endif // LIMO_DRIVER_H
