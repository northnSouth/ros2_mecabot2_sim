/**
 * @file passive_wheel_odometry.hpp
 * @brief PassiveWheelOdometry node for dead reckoning using passive
 *        omni wheels attached to rotary encoders on Mecabot2.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/joint_state.hpp>

#include <memory>

/**
 * @brief ROS 2 Node for computing and publishing dead reckoning via passive
 *        omni wheels attached to rotary encoders.
 */
class PassiveWheelOdometry : public rclcpp::Node {
public:
    using JointState = sensor_msgs::msg::JointState;
    using TransformBroadcaster = tf2_ros::TransformBroadcaster;

    PassiveWheelOdometry();

private:
    rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
    std::unique_ptr<TransformBroadcaster> tf2_broadcaster_;

    /**
     * @brief Minimal struct for robot 2D pose (x, y, heading).
     *
     * Used instead of ROS2 types like geometry_msgs::Pose 
     * to avoid bloat.
     */
    struct XYTheta {
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;
    };  

    double old_x_left_{};  ///< Last reading from left X-direction encoder
    double old_x_right_{}; ///< Last reading from right X-direction encoder
    double old_y_front_{}; ///< Last reading from front Y-direction encoder

    const double enc_x_to_center_{0.4}; ///< Distance (m) from X encoders to robot center.
    const double enc_y_to_center_{0.4}; ///< Distance (m) from Y encoders to robot center.

    XYTheta global_pose_{0, 0, 0}; ///< Robot's global 2D pose.

    void updateOdometryFromJointStates_(const JointState&);

    /**
     * @brief Parse rotary encoder states from a JointState message.
     *
     * Pick out states which names matches the specifiec suffix, then take the
     * readings as X direction left, right, and Y direction front encoders.
     *
     * @param states The message to parse.
     * @param x_left Readings output for the left encoder.
     * @param x_right Readings output for the right encoder.
     * @param y_front Readings output for the front encoder.
     * @param suffix The suffix to filter with.
     */
    int parseRotaryEncoders_(
        const JointState& states, 
        double& x_left, double& x_right, double& y_front
    );
};