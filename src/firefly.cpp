#include <cstdio>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/twist.hpp>
using namespace std::chrono_literals;
// Note: count_ is needed for the shared pointer (probably?)
class VelocityPublisher : public rclcpp::Node
{
  public:
    VelocityPublisher()
    : Node("firefly"), count_(0)
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      timer_ = this->create_wall_timer(
        500ms, std::bind(&VelocityPublisher::timer_callback, this));
    }

    private:
      void timer_callback()
      {
        auto message = geometry_msgs::msg::Twist();
        
        message.linear.x = (double(rand()))/double(RAND_MAX);
        message.linear.y = (double(rand()))/double(RAND_MAX);
        message.angular.z = (2*double(rand()))/double(RAND_MAX) - 1;

        RCLCPP_INFO(this->get_logger(), "Linear: <%lf,%lf,%lf> | Angular: <%lf,%lf,%lf>", 
                      message.linear.x, message.linear.y, message.linear.z, 
                      message.angular.x, message.angular.y, message.angular.z);
        publisher_->publish(message);
      }

      rclcpp::TimerBase::SharedPtr timer_;
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
      size_t count_;


};


int main(int argc, char ** argv)
{
  std::srand(std::time(nullptr));
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityPublisher>());
  rclcpp::shutdown();
  return 0;
}