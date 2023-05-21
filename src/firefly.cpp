#include <cstdio>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;


// Note: count_ is needed for the shared pointer (probably?)
class FireFly : public rclcpp::Node
{
  public:
    FireFly()
    : Node("firefly"), count_(0)
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      timer_ = this->create_wall_timer(
        500ms, std::bind(&FireFly::timer_callback, this));

      topic = "/world/swarm_world/model/turtle5/link/camera_link/sensor/wide_angle_camera/image";
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic, rclcpp::SensorDataQoS(), std::bind(&FireFly::topic_callback, this, _1));
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
      void topic_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) const
      {
          RCLCPP_INFO(this->get_logger(), "Image Received");
          cv_bridge::CvImagePtr cv_ptr;
          try
          {
              cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
          }
          catch (cv_bridge::Exception& e)
          {
              RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
              return;
          }

          if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
              cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
          cv::Mat resized_up;
          cv::resize(cv_ptr->image, resized_up, cv::Size(msg->width, msg->height), cv::INTER_LINEAR);

          cv::imshow("Image", resized_up);
          cv::waitKey(10);
      }

      std::string topic;
      rclcpp::TimerBase::SharedPtr timer_;
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
      rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
      size_t count_;
};


int main(int argc, char ** argv)
{
  cv::namedWindow("Image");
  cv::startWindowThread();

  std::srand(std::time(nullptr));
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FireFly>());
  rclcpp::shutdown();
  
  cv::destroyWindow("Image");
  return 0;
}