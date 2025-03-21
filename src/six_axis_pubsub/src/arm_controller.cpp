#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class ArmController : public rclcpp::Node
{
public:
    ArmController() : Node("arm_controller")
    {
        // Subscribe to the joint trajectory topic
        subscription_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/arm_controller/joint_trajectory", 10,
            std::bind(&ArmController::trajectory_callback, this, std::placeholders::_1));

        // Publisher for joint states
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        // Initialize joint states
        joint_states_.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
        joint_states_.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }

private:
    void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received joint trajectory command:");
        for (size_t i = 0; i < msg->joint_names.size(); ++i)
        {
            // Log joint name and position
            RCLCPP_INFO(this->get_logger(), "Joint: %s, Position: %f",
                        msg->joint_names[i].c_str(),
                        msg->points[0].positions[i]);

            // Update joint states
            auto it = std::find(joint_states_.name.begin(), joint_states_.name.end(), msg->joint_names[i]);
            if (it != joint_states_.name.end())
            {
                size_t index = std::distance(joint_states_.name.begin(), it);
                joint_states_.position[index] = msg->points[0].positions[i];
            }
        }

        // Publish updated joint states
        joint_states_.header.stamp = this->get_clock()->now();
        joint_state_publisher_->publish(joint_states_);
    }
    
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    sensor_msgs::msg::JointState joint_states_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmController>());
    rclcpp::shutdown();
    return 0;
}
