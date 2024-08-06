#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>

using namespace std::chrono_literals;

class TopicsPractice : public rclcpp::Node
{
public:
    TopicsPractice()
    : Node("topics_practice"), countdown_(5), state_("countdown"), spiral_size_(0.5)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&TopicsPractice::timer_callback, this));
        pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&TopicsPractice::pose_callback, this, std::placeholders::_1));
    }

private:
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        pose_ = msg;
    }

    void timer_callback()
    {
        if (state_ == "countdown")
        {
            RCLCPP_INFO(this->get_logger(), "Countdown: %d", countdown_);
            countdown_--;
            if (countdown_ < 0)
            {
                state_ = "drawing_spiral";
                RCLCPP_INFO(this->get_logger(), "Drawing spiral");
            }
        }
        else if (state_ == "drawing_spiral")
        {
            RCLCPP_INFO(this->get_logger(), "Drawing spiral");
            auto msg = geometry_msgs::msg::Twist();
            msg.linear.x = spiral_size_;
            msg.angular.z = 1.5;
            spiral_size_ += 0.05;
            publisher_->publish(msg);

            if (pose_ && (pose_->x < 1.0 || pose_->x > 7.0 || pose_->y < 1.0 || pose_->y > 7.0))
            {
                state_ = "going_straight";
                RCLCPP_INFO(this->get_logger(), "Going straight");
            }
        }
        else if (state_ == "going_straight")
        {
            RCLCPP_INFO(this->get_logger(), "Going straight");
            auto msg = geometry_msgs::msg::Twist();
            msg.linear.x = 2.0;
            msg.angular.z = 0.0;
            publisher_->publish(msg);
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    turtlesim::msg::Pose::SharedPtr pose_;
    int countdown_;
    std::string state_;
    double spiral_size_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TopicsPractice>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
