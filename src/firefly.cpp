#include <cstdio>
#include <chrono>
#include <cstdlib>
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
#include "sensor_msgs/msg/laser_scan.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

static const int ACTIVATION_THRESHOLD = 1500;
static const int ACTIVATION_EPSILON = 250;
static const int ACTIVATION_TIME_INCR = 10;
static const auto FLASH_DURATION = 1.5s;
static const float SAFE_DISTANCE = 0.5;

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
        50ms, std::bind(&FireFly::publish_callback, this));

      this->activation = (double(rand()))/double(RAND_MAX) * 1000; // this denotes the initial activation - which every firefly should differ in

      RCLCPP_INFO(this->get_logger(), "Robot %s starts with initial activation: %d", model_name.c_str(), this->activation);

      this->activation_timer = this->create_wall_timer(
        100ms, std::bind(&FireFly::activation_buildup, this)); // 10 buildups/s

      this->camera_topic1 = "/world/swarm_world/model";
      this->camera_topic1 += this->model_name;
      this->camera_topic1 += "/link/camera_link/sensor/wide_angle_camera1/image";

      this->camera_topic2 = "/world/swarm_world/model";
      this->camera_topic2 += this->model_name;
      this->camera_topic2 += "/link/camera_link/sensor/wide_angle_camera2/image";

      this->lidar_topic = "/world/swarm_world/model";
      this->lidar_topic += this->model_name;
      this->lidar_topic += "/link/lidar/sensor/hls_lfcd_lds/scan";


      this->camera_sub_1 = this->create_subscription<sensor_msgs::msg::Image>(
      this->camera_topic1, rclcpp::SensorDataQoS(), std::bind(&FireFly::camera_callback, this, _1));

      this->camera_sub_2 = this->create_subscription<sensor_msgs::msg::Image>(
      this->camera_topic2, rclcpp::SensorDataQoS(), std::bind(&FireFly::camera_callback, this, _1));

      this->flash_pub = this->create_publisher<std_msgs::msg::Empty>("/model" + this->model_name + "/LED_mode", 10);

      this->lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
      this->lidar_topic, rclcpp::SensorDataQoS(), std::bind(&FireFly::lidar_callback, this, _1));
    }

    private:
      void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) // Producer: Queue size is only 1
      {       
        float min_range = msg->range_max;
        int i = 0;
        int minIdx = -1;
        for (auto& range : msg->ranges) {
          if (range >= msg->range_min && range < min_range) {
            min_range = range;
            minIdx = i;
          }
          i++;
        }

        float turn_angle = 0;
        geometry_msgs::msg::Twist motion;
        if (min_range < SAFE_DISTANCE) {
          float obstacle_angle = minIdx * msg->angle_increment; // in rad
          // Turn 180 or -180 from the obstacle -> should be < 6.14
          turn_angle = obstacle_angle + (obstacle_angle > 1.57 ? 3.14 : -3.14); 
          /* RCLCPP_INFO(this->get_logger(), "Robot %s found obstacle at range: %f | Turning: %f"
              , model_name.c_str(), min_range, turn_angle); */
          motion.linear.x = 0;
          motion.angular.z = 3.14;
        } else {
          motion.linear.x = 2;
          motion.angular.z = 0;
        }
        std::lock_guard<std::mutex> lck(geo_work_lck);
        this->turn_angle = turn_angle;

        geometry_pub->publish(motion);
        
      }

      void publish_callback() // Consumer
      { 
        /* geometry_msgs::msg::Twist msg;
        msg.linear.x = 2;
        msg.linear.y = 0;
        msg.angular.z = this->turn_angle; // We don't care about stale data here!

        geometry_pub->publish(msg); */
      }

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
          activation -= ACTIVATION_THRESHOLD;
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
      
      void camera_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
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

        int pix_count = cv::countNonZero(mask);
        if (pix_count > 0) {
          // Reference: https://stackoverflow.com/questions/44749735/count-red-color-object-from-video-opencv-python
          std::vector<std::vector<cv::Point> > contours;
          std::vector<cv::Vec4i> hierarchy;

          cv::findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
          if (contours.size() > this->previous_flashes_seen) {
            /* RCLCPP_INFO(this->get_logger(), "Robot %s detected %ld flashings vs %d in the last frame", 
                                        model_name.c_str(), contours.size(), this->previous_flashes_seen); */

            const std::lock_guard<std::mutex> lock(this->activation_lock);            
            this->activation += (ACTIVATION_EPSILON * (contours.size() - this->previous_flashes_seen));

            this->previous_flashes_seen = contours.size();
          }
        } 
        else {
          const std::lock_guard<std::mutex> lock(this->activation_lock);            
          this->previous_flashes_seen = 0;
        }

        //cv::imshow(this->model_name, thresh);
        cv::waitKey(10);
      }

      std::string camera_topic1;
      std::string camera_topic2;
      std::string lidar_topic;
      rclcpp::TimerBase::SharedPtr publisher_timer;
      rclcpp::TimerBase::SharedPtr activation_timer;
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr geometry_pub;
      rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr flash_pub;
      rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_1;
      rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_2;
      rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;
      std::string model_name;
      unsigned int previous_flashes_seen;
      int activation;
      std::mutex activation_lock;
      std::mutex flash_lock;
      std::mutex geo_work_lck;
      float turn_angle;
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