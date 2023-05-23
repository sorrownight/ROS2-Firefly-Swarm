#include <cstdio>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <chrono>
#include <vector>
#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

static const int ACTIVATION_THRESHOLD = 1500;
static const int ACTIVATION_EPSILON = 100;
static const int ACTIVATION_TIME_INCR = 10;
static const auto FLASH_DURATION = 1s;

// Note: count_ is needed for the shared pointer (probably?)
class FireFly : public rclcpp::Node
{
  public:
    FireFly()
    : Node("firefly"), count_(0)
    {
      this->previous_flashes_seen = 0;
      this->declare_parameter("model_name", "turtle1");
      this->model_name = "/" + this->get_parameter("model_name").as_string();
      std::srand(std::time(nullptr) + model_name[model_name.size()-1]);

      this->geometry_pub = this->create_publisher<geometry_msgs::msg::Twist>("/model" + this->model_name  + "/cmd_vel", 10);
      this->publisher_timer = this->create_wall_timer(
        500ms, std::bind(&FireFly::publish_callback, this));

      this->activation = (double(rand()))/double(RAND_MAX) * 1000; // this denotes the initial activation - which every firefly should differ in

      RCLCPP_INFO(this->get_logger(), "Robot %s starts with initial activation: %d", model_name.c_str(), this->activation);

      this->activation_timer = this->create_wall_timer(
        100ms, std::bind(&FireFly::activation_buildup, this)); // 10 buildups/s

      this->camera_topic = "/world/swarm_world/model";
      this->camera_topic += this->model_name;
      this->camera_topic += "/link/camera_link/sensor/wide_angle_camera/image";

      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      this->camera_topic, rclcpp::SensorDataQoS(), std::bind(&FireFly::topic_callback, this, _1));

      this->flash_pub = this->create_publisher<std_msgs::msg::Empty>("/model" + this->model_name + "/LED_mode", 10);
    }

    private:
      void activation_buildup()
      {
        // This is a simplication of the discrete function presented in the IEEE paper
        // We interpreted the control cycles as the frequency at which the activation will build up
        // For our purpose, we assume, as was interpreted by the paper's results, that 
        // the rate at which the activation builds up (h(x_i)) is the same across all fireflies
        // We further question this constraint, however. Perhaps it can be removed. Further testing is required.
        // With the initial activation being 0, a firefly shall flash after exactly 15s
        const std::lock_guard<std::mutex> lock(this->activation_lock);
        this->activation += ACTIVATION_TIME_INCR; // 1s == 100 => 15s == 1500

        if (activation > ACTIVATION_THRESHOLD) {
          // Only one flash should occur at any given time!
          std::thread(&FireFly::flash, this).detach();
          activation = 0;
        }
      }
      void flash()
      {
        const std::lock_guard<std::mutex> lock(this->flash_lock);
        RCLCPP_INFO(this->get_logger(), "Lo! Robot %s is flashing!", model_name.c_str());
        this->flash_pub->publish(std_msgs::msg::Empty()); // LED ON
        std::this_thread::sleep_for(FLASH_DURATION);
        this->flash_pub->publish(std_msgs::msg::Empty()); // LED OFF
      }

      void publish_callback()
      {
        auto message = geometry_msgs::msg::Twist();
        std::string cmd_vel_top = "/model" + this->model_name  + "/cmd_vel";
        message.linear.x = (double(rand()))/double(RAND_MAX) * 2;
        message.linear.y = (double(rand()))/double(RAND_MAX);
        message.angular.x = (double(rand()))/double(RAND_MAX) * 3 - 1;

        /* RCLCPP_INFO(this->get_logger(), "Linear: <%lf,%lf,%lf> | Angular: <%lf,%lf,%lf> to [%s]", 
                      message.linear.x, message.linear.y, message.linear.z, 
                      message.angular.x, message.angular.y, message.angular.z, cmd_vel_top.c_str()); */
          
        // geometry_pub->publish(message);
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

      void extract_green(const cv::Mat& img)
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
          // Reference: https://stackoverflow.com/questions/44749735/count-red-color-object-from-video-opencv-python
          std::vector<std::vector<cv::Point> > contours;
          std::vector<cv::Vec4i> hierarchy;

          cv::findContours(thresh, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
          if (contours.size() > this->previous_flashes_seen) {
            RCLCPP_INFO(this->get_logger(), "Robot %s detected %ld flashings vs %d in the last frame", 
                                        model_name.c_str(), contours.size(), this->previous_flashes_seen);

            const std::lock_guard<std::mutex> lock(this->activation_lock);            
            this->activation += ACTIVATION_EPSILON;

            this->previous_flashes_seen = contours.size();
          }
        } 
        else this->previous_flashes_seen = 0;

        cv::imshow(this->model_name, thresh);
        cv::waitKey(10);
      }

      std::string camera_topic;
      rclcpp::TimerBase::SharedPtr publisher_timer;
      rclcpp::TimerBase::SharedPtr activation_timer;
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr geometry_pub;
      rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr flash_pub;
      rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
      std::string model_name;
      unsigned int previous_flashes_seen;
      int activation;
      std::mutex activation_lock;
      std::mutex flash_lock;
      size_t count_;
};


int main(int argc, char ** argv)
{
  cv::startWindowThread();

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FireFly>());
  rclcpp::shutdown();
  
  return 0;
}