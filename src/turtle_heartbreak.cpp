#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "turtlesim/msg/pose.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class TurtlesimHeartBreak : public rclcpp::Node
{
private:
    void set_pen(bool enable, int r = 255, int g = 0, int b = 0, int width = 5)
    {
        auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
        request->r = r; request->g = g; request->b = b; request->width = width; request->off = !enable;
        set_pen_client_->async_send_request(request, [this](rclcpp::Client<turtlesim::srv::SetPen>::SharedFuture)
        {
            RCLCPP_INFO(this->get_logger(), "Pen settings applied.");
        });
    }

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) { current_pose_ = *msg; }

    void draw_heart()
    {
        if (move_index_ < heart_directions_.size())
        {
            move_turtle(heart_directions_);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Heart drawing finished.");
            set_pen(false);
            set_pen(true); 
            move_index_ = 0;
            timer_->cancel();  
            timer_ = this->create_wall_timer(2s, std::bind(&TurtlesimHeartBreak::draw_zigzag, this));  
        }
    }

    void draw_zigzag()
    {
        set_pen(true, 0, 0, 0, 3.5);  
        if (move_index_ < zigzag_directions_.size())
        {
            move_turtle(zigzag_directions_);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Zigzag drawing finished.");
            set_pen(false);
        }
    }

    void move_turtle(const std::vector<std::pair<double, double>>& directions)
    {
        double target_x = directions[move_index_].first;
        double target_y = directions[move_index_].second;
        double final_target_x = current_pose_.x + target_x;
        double final_target_y = current_pose_.y + target_y;

        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = target_x;  
        twist_msg.linear.y = target_y; 
        pub_->publish(twist_msg);

        rclcpp::sleep_for(1s); 
        double distance_moved = std::hypot(current_pose_.x - final_target_x, current_pose_.y - final_target_y);
        double required_distance = std::hypot(target_x, target_y);
        if (distance_moved < 0.1 && required_distance > 0.1)
        {
            double remaining_distance_x = target_x - (final_target_x - current_pose_.x);
            double remaining_distance_y = target_y - (final_target_y - current_pose_.y);

            if (remaining_distance_x != 0.0 || remaining_distance_y != 0.0)
            {
                twist_msg.linear.x = remaining_distance_x;
                twist_msg.linear.y = remaining_distance_y;
                pub_->publish(twist_msg);
            }
        }
        else { move_index_++; }
    }

    void teleport_to_start()
    {
        auto teleport_request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
        teleport_request->x = 3; teleport_request->y = 7; teleport_request->theta = 0.0; 
        teleport_client_->async_send_request(teleport_request, [this](rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedFuture /*result*/)
        {
            RCLCPP_INFO(this->get_logger(), "Teleported to start.");
            rclcpp::shutdown();
        });
    }

protected:
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr set_pen_client_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t move_index_;
    std::vector<std::pair<double, double>> heart_directions_;
    std::vector<std::pair<double, double>> zigzag_directions_;
    turtlesim::msg::Pose current_pose_; 

public:
    TurtlesimHeartBreak() : Node("turtlesim_HeartBreak"), move_index_(0)
    {
        set_pen_client_ = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
        teleport_client_ = this->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        
        heart_directions_ = {
            {0.0, 2.0}, {1.0, 1.0}, {2.0, 0.0}, {0.5, -1.0}, {0.5, 1.0},
            {2.0, 0.0}, {1.0, -1.0}, {0.0, -2.0}, {-3.5, -4.0}, {-3.5, 4.0},
            {3.5, -4.0}            
        };

        
        zigzag_directions_ = {
            {0.0, 1.0}, {0.5, 1.0}, {-1.0, 1.0}, 
            {1.0, 1.0}, {-1.0, 1.0}, {0.5, 1.0}
        };

        while (!teleport_client_->wait_for_service(5s) || !set_pen_client_->wait_for_service(5s))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for turtlesim services...\n>> set_pen\n>> teleport_absolute");
        }

        set_pen(false);
        auto teleport_request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
        teleport_request->x = 2.0; teleport_request->y = 6.0; teleport_request->theta = 0.0;
        teleport_client_->async_send_request(teleport_request, [this](rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedFuture /*result*/)
        {
            RCLCPP_INFO(this->get_logger(), "Initial teleportation done.");
            set_pen(true);
        });

        auto pose_subscriber = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&TurtlesimHeartBreak::pose_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(2.25s, std::bind(&TurtlesimHeartBreak::draw_heart, this));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtlesimHeartBreak>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
