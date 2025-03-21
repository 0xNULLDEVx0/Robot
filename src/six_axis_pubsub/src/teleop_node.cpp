#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <iostream>
#include <vector>
#include <string>

class TeleopNode : public rclcpp::Node
{
public:
    TeleopNode() : Node("teleop_node"), selected_joint_(0)
    {
        // Define joint names and limits
        joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        joint_limits_ = {
            {-2.0, 2.0}, // Limits for joint1
            {-1.5, 1.5}, // Limits for joint2
            {-3.0, 3.0}, // Limits for joint3
            {-2.5, 2.5}, // Limits for joint4
            {-1.0, 1.0}, // Limits for joint5
            {-1.5, 1.5}  // Limits for joint6
        };

        // Initialize joint positions
        joint_positions_ = std::vector<double>(joint_names_.size(), 0.0);

        // Create a publisher for joint trajectories
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/arm_controller/joint_trajectory", 10);

        // Start the input loop in a separate thread
        input_thread_ = std::thread(&TeleopNode::input_loop, this);
    }

    ~TeleopNode()
    {
        if (input_thread_.joinable())
        {
            input_thread_.join();
        }
    }

private:
    struct JointLimits
    {
        double lower;
        double upper;
    };

    std::vector<std::string> joint_names_;
    std::vector<JointLimits> joint_limits_;
    std::vector<double> joint_positions_;
    int selected_joint_;
    std::thread input_thread_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;

    void input_loop()
{
    while (rclcpp::ok())
    {
        std::cout << "Select a joint (1-" << joint_names_.size() << "): ";
        int joint;
        std::cin >> joint;

        if (joint < 1 || joint > static_cast<int>(joint_names_.size()))
        {
            std::cout << "Invalid joint number. Please select between 1 and " << joint_names_.size() << "." << std::endl;
            continue;
        }

        selected_joint_ = joint - 1; // Convert to zero-based index
        std::cout << "Selected joint: " << joint_names_[selected_joint_] << std::endl;

        char command;
        while (true)
        {
            std::cout << "Press 'w' to raise the joint, 's' to lower the joint, 'q' to quit: ";
            std::cin >> command;

            if (command == 'w') // Raise the joint
            {
                if (joint_positions_[selected_joint_] + 0.1 <= joint_limits_[selected_joint_].upper)
                {
                    joint_positions_[selected_joint_] += 0.1;
                    publish_trajectory();
                }
                else
                {
                    std::cout << "Cannot raise joint " << joint_names_[selected_joint_] << " beyond its upper limit: "
                              << joint_limits_[selected_joint_].upper << std::endl;
                }
            }
            else if (command == 's') // Lower the joint
            {
                if (joint_positions_[selected_joint_] - 0.1 >= joint_limits_[selected_joint_].lower)
                {
                    joint_positions_[selected_joint_] -= 0.1;
                    publish_trajectory();
                }
                else
                {
                    std::cout << "Cannot lower joint " << joint_names_[selected_joint_] << " beyond its lower limit: "
                              << joint_limits_[selected_joint_].lower << std::endl;
                }
            }
            else if (command == 'q') // Quit
            {
                break;
            }
            else
            {
                std::cout << "Invalid command. Use 'w', 's', or 'q'." << std::endl;
            }
        }
    }
}

    void publish_trajectory()
    {
        auto message = trajectory_msgs::msg::JointTrajectory();
        message.joint_names = joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = joint_positions_;
        point.time_from_start = rclcpp::Duration::from_seconds(1.0);

        message.points.push_back(point);

        publisher_->publish(message);

        std::cout << "Published joint positions: ";
        for (const auto &pos : joint_positions_)
        {
            std::cout << pos << " ";
        }
        std::cout << std::endl;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopNode>());
    rclcpp::shutdown();
    return 0;
}
