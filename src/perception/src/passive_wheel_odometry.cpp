/**
 * @file kinematics_worker.cpp
 * @brief Implements the Mecabot2 dead reckoning (passive wheel odometry) node.
 *
 */

#include "ros2_mecabot2_sim_perception/passive_wheel_odometry.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <functional>
#include <vector>
#include <string>

namespace {
    constexpr const char* NODE_NAME = "mecabot2_passive_wheel_odometry";
    constexpr const char* JOINT_STATE_TOPIC = "/joint_states";
    const auto QOS = rclcpp::QoS(1);
}

PassiveWheelOdometry::PassiveWheelOdometry() : Node(NODE_NAME) {
    RCLCPP_INFO(this->get_logger(), 
        "\033[32mStarting Mecabot2 Passive Wheel Odometry\033[0m"
    );
    
    this->declare_parameters<std::string>("", {
        {"origin_frame", "world"},
        {"base_link_frame", "base_link"},
        {"left_encoder_frame", "encoder_left"},
        {"right_encoder_frame", "encoder_right"},
        {"front_encoder_frame", "encoder_front"}
    });

    joint_state_sub_ = this->create_subscription<JointState>(
        JOINT_STATE_TOPIC,
        QOS,
        std::bind(
            &PassiveWheelOdometry::updateOdometryFromJointStates_, 
            this, 
            std::placeholders::_1
        )
    );

    tf2_broadcaster_ = std::make_unique<TransformBroadcaster>(*this);
}

void PassiveWheelOdometry::updateOdometryFromJointStates_(const JointState& states) {
    double x_left {}, x_right {}, y_front {};

    if (parseRotaryEncoders_(states, x_left, x_right, y_front)) {
        RCLCPP_ERROR(this->get_logger(), 
        "[Rotary Parser]: Encoder(s) failed to be parsed!");

        const std::string left_encoder = this->get_parameter("left_encoder_frame").as_string();
        const std::string right_encoder = this->get_parameter("right_encoder_frame").as_string();
        const std::string front_encoder = this->get_parameter("front_encoder_frame").as_string();

        RCLCPP_ERROR(this->get_logger(), "encoder_left: %s", left_encoder.c_str());
        RCLCPP_ERROR(this->get_logger(), "encoder_right: %s", right_encoder.c_str());
        RCLCPP_ERROR(this->get_logger(), "encoder_front: %s", front_encoder.c_str());
    }

    XYTheta local_pose_update;

    double delta_x1 = x_left - old_x_left_;
    double delta_x2 = x_right - old_x_right_;
    double delta_y = y_front - old_y_front_;

    local_pose_update.x = (delta_x1 + delta_x2) / 2;
    // Negated, to fix reversed orientation.
    local_pose_update.theta = (delta_x2 - delta_x1 ) / (enc_x_to_center_ * 2) * -1;
    local_pose_update.y = delta_y - enc_y_to_center_ * local_pose_update.theta;

    old_x_left_ = x_left;
    old_x_right_ = x_right;
    old_y_front_ = y_front;

    double& pose_x = global_pose_.x;
    double& pose_y = global_pose_.y;
    double& pose_orient = global_pose_.theta;

    pose_orient += local_pose_update.theta;
    double cos_orient = cos(pose_orient);
    double sin_orient = sin(pose_orient);

    pose_x += local_pose_update.x * cos_orient - local_pose_update.y * sin_orient;
    pose_y += local_pose_update.x * sin_orient + local_pose_update.y * cos_orient;

    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = states.header.stamp;
    t.header.frame_id = this->get_parameter("origin_frame").as_string();
    t.child_frame_id = this->get_parameter("base_link_frame").as_string();

    t.transform.translation.x = pose_x;
    t.transform.translation.y = pose_y;

    tf2::Quaternion q;
    q.setRPY(0, 0, pose_orient);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf2_broadcaster_->sendTransform(t);
}

int PassiveWheelOdometry::parseRotaryEncoders_(
    const JointState& states, 
    double& x_left, double& x_right, double& y_front
) {
    const std::string left_encoder = this->get_parameter("left_encoder_frame").as_string();
    const std::string right_encoder = this->get_parameter("right_encoder_frame").as_string();
    const std::string front_encoder = this->get_parameter("front_encoder_frame").as_string();

    const auto& name_list = states.name;
    const auto& position_list = states.position;

    bool found_left = false, found_right = false, found_front = false;

    for (size_t i = 0; i < name_list.size(); ++i) {
        const std::string& joint = name_list[i];
        double position = position_list[i];

        /**
         * x_left and y_front are negated so they will update in the
         * correct direction.
         */
        if (joint == left_encoder) {
            x_left = position * -0.1;
            found_left = true;
        } else if (joint == right_encoder) {
            x_right = position * 0.1;
            found_right = true;
        } else if (joint == front_encoder) {
            y_front = position * -0.1;
            found_front = true;
        }

        if (found_left && found_right && found_front) break;
    }

    if (!found_left || !found_right || !found_front) return -1;

    return 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PassiveWheelOdometry>());
    rclcpp::shutdown();
    return 0;
}