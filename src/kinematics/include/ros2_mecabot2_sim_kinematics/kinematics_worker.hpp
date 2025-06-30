/**
 * @file kinematics_worker.hpp
 * @brief KinematicsWorker node for mecanum drive kinematics calculation 
 *        on Mecabot2.  
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>

/**
 * @brief ROS 2 Node for computing and publishing mecanum wheel velocities.
 */
class KinematicsWorker : public rclcpp::Node {
public:
    using Twist = geometry_msgs::msg::Twist;
    using Float64MultiArray = std_msgs::msg::Float64MultiArray;

    KinematicsWorker();

private:
    rclcpp::Publisher<Float64MultiArray>::SharedPtr motion_publisher_;
    rclcpp::Subscription<Twist>::SharedPtr command_listener_; 
    rclcpp::TimerBase::SharedPtr kinematics_worker_; 
    Float64MultiArray veloc_msg_; 

    Twist twist_msg_;

    void kinematicsRoutine_();

    /**
     * @brief Calculates wheel velocities for a mecanum drive 
     *        from a Cartesian velocity command.
     *
     * Converts linear (x, y) and angular (z) velocities into individual 
     * wheel velocities, applying normalization if necessary.
     * Check project README for algorithm explanation.
     *
     * @param magnitude The linear velocity magnitude in robot frame.
     * @param heading The angle (in radians) of motion relative to x-axis.
     * @param angular The angular velocity (rotation rate).
     * @param m_1 Output velocity for front-left wheel.
     * @param m_2 Output velocity for front-right wheel.
     * @param m_3 Output velocity for rear-left wheel.
     * @param m_4 Output velocity for rear-right wheel.
     */
    void calcWheelVelocity_(
        const float magnitude, const float heading, const double angular,
        float& m_1, float& m_2, float& m_3, float& m_4
    );
};