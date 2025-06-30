/**
 * @file kinematics_worker.cpp
 * @brief Implements the Mecabot2 kinematics calculator node.
 *
 * This node subscribes to /cmd_vel and calculates normalized wheel velocities
 * for a mecanum-based robot. It periodically publishes motor commands to
 * /velo_c/commands using a joint group velocity controller.
 *
 * The velocity computation is based on standard mecanum kinematics, taking
 * into account linear direction and angular rotation, with normalization to
 * keep the output in range [-1, 1].
 */

#include "ros2_mecabot2_sim_kinematics/kinematics_worker.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <cmath>

using time_ms = std::chrono::milliseconds;
using time_s = std::chrono::seconds;

namespace {
    constexpr const char* NODE_NAME = "mecabot2_kinematics_worker";
    constexpr const char* CMD_TOPIC = "/cmd_vel";
    constexpr const char* MOTION_CONTROL_TOPIC = "/velo_c/commands";
    constexpr time_ms KINEMATICS_ROUTINE_PERIOD(10);
    constexpr time_s CONTROLLER_WAIT_PERIOD(3);
    const auto QOS = rclcpp::QoS(1);
}

KinematicsWorker::KinematicsWorker() : Node(NODE_NAME) {
    RCLCPP_INFO(this->get_logger(), 
        "\033[32mStarting Mecabot2 Kinematics Worker\033[0m"
    );

    this->declare_parameter<float>("speed_multiplier", 1);

    motion_publisher_ = this->create_publisher<Float64MultiArray>(
        MOTION_CONTROL_TOPIC,
        QOS
    );

    command_listener_ = this->create_subscription<Twist>(
        CMD_TOPIC, 
        QOS, 
        [this](Twist::UniquePtr msg) { twist_msg_ = *msg; }
    );

    kinematics_worker_ = this->create_wall_timer(
        KINEMATICS_ROUTINE_PERIOD, 
        std::bind(&KinematicsWorker::kinematicsRoutine_, this)
    );

    /** 
     * The Joint Group Velocity Controller seem to start responding 
     * in a few seconds after the first message is published to its 
     * advertised topic.
     */
    veloc_msg_.data.assign(4, 0);
    motion_publisher_->publish(veloc_msg_);
    RCLCPP_INFO(this->get_logger(), "Starting kinematics...");
    rclcpp::sleep_for(CONTROLLER_WAIT_PERIOD);
    RCLCPP_INFO(this->get_logger(), "Kinematics online");
}

void KinematicsWorker::kinematicsRoutine_() {
    float speed_multiplier;
    this->get_parameter("speed_multiplier", speed_multiplier);

    // Calculate polar coordinates
    double cmd_x = this->twist_msg_.linear.x;
    double cmd_y = this->twist_msg_.linear.y;
    double cmd_angular = this->twist_msg_.angular.z;

    float polar_heading = atan2(cmd_y, cmd_x) + M_PI/4;
    float polar_magnitude = hypot(cmd_x, cmd_y);

    float m_1, m_2, m_3, m_4;
    calcWheelVelocity_(
        polar_magnitude, 
        polar_heading, 
        cmd_angular, 
        m_1, m_2, m_3, m_4
    );

    veloc_msg_.data = {
        m_1 * speed_multiplier, // Front left
        m_2 * speed_multiplier, // Front right
        m_3 * speed_multiplier, // Rear left
        m_4 * speed_multiplier // Rear right
    };

    motion_publisher_->publish(veloc_msg_);
}

void KinematicsWorker::calcWheelVelocity_(
    const float magnitude, const float heading, const double angular,
    float& m_1, float& m_2, float& m_3, float& m_4
) {
    const float vector_a = magnitude * cos(heading);
    const float vector_b = magnitude * sin(heading);

    m_1 = vector_a - angular;
    m_2 = vector_b + angular;
    m_3 = vector_b - angular;
    m_4 = vector_a + angular;

    // Limit output to wheels between -1 to 1 (normalization)
    float sum_magnitude = magnitude + abs(angular);
    if (sum_magnitude > 1) {
        m_1 /= sum_magnitude;
        m_2 /= sum_magnitude;
        m_3 /= sum_magnitude;
        m_4 /= sum_magnitude;
    }
}

int main (int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinematicsWorker>());
    rclcpp::shutdown();
    return 0;
}