#include <cstdio>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <chrono>

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

static const auto CAMERA_REFRESH_INTERVAL = 500ms;

// Note: count_ is needed for the shared pointer (probably?)
class FireFly : public rclcpp::Node
{
  public:
    FireFly()
    : Node("firefly"), count_(0)
    {
      this->declare_parameter("model_name", "turtle1");
      this->model_name = "/" + this->get_parameter("model_name").as_string();

      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/model" + this->model_name  + "/cmd_vel", 10);
      timer_ = this->create_wall_timer(
        500ms, std::bind(&FireFly::timer_callback, this));

      topic = "/world/swarm_world/model";
      topic += this->model_name;
      topic += "/link/camera_link/sensor/wide_angle_camera/image";

      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic, rclcpp::SensorDataQoS(), std::bind(&FireFly::topic_callback, this, _1));
    }

    private:
      void timer_callback()
      {
        auto message = geometry_msgs::msg::Twist();
        std::string cmd_vel_top = "/model" + this->model_name  + "/cmd_vel";
        message.linear.x = (double(rand()))/double(RAND_MAX) * 2;
        message.linear.y = (double(rand()))/double(RAND_MAX);
        message.angular.x = (double(rand()))/double(RAND_MAX) * 3 - 1;

        /* RCLCPP_INFO(this->get_logger(), "Linear: <%lf,%lf,%lf> | Angular: <%lf,%lf,%lf> to [%s]", 
                      message.linear.x, message.linear.y, message.linear.z, 
                      message.angular.x, message.angular.y, message.angular.z, cmd_vel_top.c_str()); */
          
        // publisher_->publish(message);
      }
      
      void topic_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
      {
        // RCLCPP_INFO(this->get_logger(), "Image Received");
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
          cv::Mat resizedImg;
          cv::resize(cv_ptr->image, resizedImg, cv::Size(msg->width, msg->height), cv::INTER_LINEAR);

          this->extract_green(resizedImg);
      }

      void extract_green(const cv::Mat& img) const
      {
        // Credit: 
        // https://techvidvan.com/tutorials/detect-objects-of-similar-color-using-opencv-in-python/
        // https://www.opencv-srf.com/2010/09/object-detection-using-color-seperation.html
        // convert to hsv colorspace
        cv::Mat hsv;
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

        int iLowH = 45;
        int iHighH = 90;

        int iLowS = 50; 
        int iHighS = 255;

        int iLowV = 50;
        int iHighV = 255;

        cv::Mat mask;
        cv::inRange(hsv, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), mask);

        cv::Mat Kernel = cv::Mat(cv::Size(7,7),CV_8UC1,cv::Scalar(255));

        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, Kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, Kernel);

        cv::Mat merged;
        cv::bitwise_and(img, img, merged, mask);

        // Convert to binary (Just black or white)
        cv::Mat grayscaled;
        cv::cvtColor(merged, grayscaled, cv::COLOR_BGR2GRAY);
        cv::Mat thresh;
        cv::threshold(grayscaled, thresh, 2, 255, cv::THRESH_BINARY);
        int pix_count = cv::countNonZero(grayscaled);
        if (pix_count > 0) {
          RCLCPP_INFO(this->get_logger(), "Robot %s detected flashing with %d pixels", model_name.c_str(), pix_count);        
        }

        cv::imshow(this->model_name, thresh);
        cv::waitKey(10);
      }

      std::string topic;
      rclcpp::TimerBase::SharedPtr timer_;
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
      rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
      std::string model_name;
      size_t count_;
};


int main(int argc, char ** argv)
{
  cv::startWindowThread();

  std::srand(std::time(nullptr));
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FireFly>());
  rclcpp::shutdown();
  
  return 0;
}