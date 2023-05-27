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
#include "rclcpp/time.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

static const int ACTIVATION_THRESHOLD = 1500;
static const int ACTIVATION_EPSILON = 1250;
static const int ACTIVATION_TIME_INCR = 5;
static const auto FLASH_DURATION = std::chrono::duration_cast<std::chrono::nanoseconds>(1.5s);
static const float SAFE_DISTANCE = 0.5;
static const auto SUSPICIOUS_DURATION = 2s;
static const double POS_MARGIN = 0.1;
static const int SUS_FLASH_CYCLE = 2; // How many times we need to flash until we're sure that a peer is broken

// Note: count_ is needed for the shared pointer (probably?)
class FireFly : public rclcpp::Node
{
  public:
    FireFly()
    : Node("firefly"), count_(0)
    {
      this->cam1_prev_flashes = 0;
      this->declare_parameter("model_name", "turtle1");
      this->model_name = "/" + this->get_parameter("model_name").as_string();
      std::srand(std::time(nullptr) + model_name[model_name.size()-1]);

      // this denotes the initial activation - which every firefly should differ in
      this->activation = (double(rand()))/double(RAND_MAX) * 1000; 

      RCLCPP_INFO(this->get_logger(), "Robot %s starts with initial activation: %d", model_name.c_str(), this->activation);

      // Default Mutually exclusive group. Passing a new group here seems to be bugged
      this->activation_timer = this->create_wall_timer(
        50ms, std::bind(&FireFly::activation_buildup, this)); // 10 buildups/s

      this->camera_topic1 = "/world/swarm_world/model";
      this->camera_topic1 += this->model_name;
      this->camera_topic1 += "/link/camera_link/sensor/wide_angle_camera1/image";

      this->camera_topic2 = "/world/swarm_world/model";
      this->camera_topic2 += this->model_name;
      this->camera_topic2 += "/link/camera_link/sensor/wide_angle_camera2/image";

      this->lidar_topic = "/world/swarm_world/model";
      this->lidar_topic += this->model_name;
      this->lidar_topic += "/link/lidar/sensor/hls_lfcd_lds/scan";

      // Must enforce ordering for camera. Publishers can go ham.
      auto camera_grp_1 = this->create_callback_group((rclcpp::CallbackGroupType::MutuallyExclusive));
      auto camera_grp_2 = this->create_callback_group((rclcpp::CallbackGroupType::MutuallyExclusive));
      auto geom_grp = this->create_callback_group((rclcpp::CallbackGroupType::Reentrant));
      rclcpp::SubscriptionOptions camera_opt_1;
      rclcpp::SubscriptionOptions camera_opt_2;
      rclcpp::PublisherOptions geom_opt;
      camera_opt_1.callback_group = camera_grp_1;
      camera_opt_2.callback_group = camera_grp_2;
      geom_opt.callback_group = geom_grp;

      this->camera_sub_1 = this->create_subscription<sensor_msgs::msg::Image>(
      this->camera_topic1, rclcpp::SensorDataQoS(), std::bind(&FireFly::camera1_callback, this, _1), camera_opt_1);

      this->camera_sub_2 = this->create_subscription<sensor_msgs::msg::Image>(
      this->camera_topic2, rclcpp::SensorDataQoS(), std::bind(&FireFly::camera2_callback, this, _1), camera_opt_2);

      this->geometry_pub = this->create_publisher<geometry_msgs::msg::Twist>("/model" + this->model_name  + "/cmd_vel", 10, geom_opt);

      auto flash_grp = this->create_callback_group((rclcpp::CallbackGroupType::Reentrant));
      rclcpp::PublisherOptions flash_opt;
      flash_opt.callback_group = flash_grp;
      // We should never have 2 flashes on the same robot at the same time!
      this->flash_pub = this->create_publisher<std_msgs::msg::Empty>("/model" + this->model_name + "/LED_mode", 1, flash_opt);

      auto lidar_grp = this->create_callback_group((rclcpp::CallbackGroupType::MutuallyExclusive));
      rclcpp::SubscriptionOptions lidar_opt;
      lidar_opt.callback_group = lidar_grp;

      this->lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
      this->lidar_topic, rclcpp::SensorDataQoS(), std::bind(&FireFly::lidar_callback, this, _1), lidar_opt);


      auto color_grp = this->create_callback_group((rclcpp::CallbackGroupType::MutuallyExclusive));
      rclcpp::PublisherOptions color_opt;
      color_opt.callback_group = color_grp;
      this->color_pub = this->create_publisher<std_msgs::msg::ColorRGBA>("/model" + this->model_name + "/LED_color", 1, color_opt);

      std::string murder_topic = "/model" + this->model_name + "/kill";

      auto kill_grp = this->create_callback_group((rclcpp::CallbackGroupType::MutuallyExclusive));
      rclcpp::SubscriptionOptions kill_opt;
      kill_opt.callback_group = kill_grp;
      this->kill_switch_sub = this->create_subscription<std_msgs::msg::Empty>(
      murder_topic, 10, std::bind(&FireFly::die, this, _1), kill_opt);
    }

    private:
      bool rescue_mode()
      {
        RCLCPP_INFO(this->get_logger(), "Robot %s is rescuing another robot", model_name.c_str());
        if (this->found_red) return false;
        if (!this->flash_lock.try_lock())
          return false; // We are flashing
                
        std_msgs::msg::ColorRGBA red;
        red.r = 1.0;
        this->color_pub->publish(red);
        this->rescuing = true;
        this->flash_lock.unlock();
        return true;
      }

      bool normal_mode(bool flag = false) // Sorry, I'm not very creative at this point
      {        
        if (!this->rescuing || flag) return false;
        if (!this->flash_lock.try_lock()) return false; // We are flashing       

        RCLCPP_INFO(this->get_logger(), "Robot %s is changing back to blue", model_name.c_str());
        std_msgs::msg::ColorRGBA blue;
        blue.b = 1.0;
        this->color_pub->publish(blue);
        this->rescuing = false; // Let it race! Let it race! Can't hold it back anymore! Actually maybe it won't.
        this->flash_lock.unlock();
        return true;
      }

      void die(const std_msgs::msg::Empty::ConstSharedPtr)
      {
        this->dead = true;
        while (!this->normal_mode(true));       
        RCLCPP_INFO(this->get_logger(), "Robot %s was murdered by a higher power. RIP!", model_name.c_str());
        rclcpp::sleep_for(2s);
        geometry_msgs::msg::Twist motion;
        geometry_pub->publish(motion);
        rclcpp::sleep_for(2s);
        geometry_pub->publish(motion);
        rclcpp::sleep_for(2s);
        geometry_pub->publish(motion);
      }

      void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
      { 
        if (this->dead) return;
        auto current_time = this->get_clock()->now();
        geometry_msgs::msg::Twist motion;
        // Make sure to use copies here else we might segfault
        for (auto sus : suspicions) { // Prob fine to use stale data here
          if (sus.first_spotted + SUSPICIOUS_DURATION > current_time && sus.flash_cycle > this->flash_count + SUS_FLASH_CYCLE) {
            RCLCPP_INFO(this->get_logger(), "Robot %s flash cycle: %d| Sus flash cycle: %d!", model_name.c_str(), this->flash_count, sus.flash_cycle);
            // Uhoh. We have a rogue 
            // We are pretty sure now. A lot of time has passed + we have flashed a couple of times but they haven't
            // WEEWOOOWEEEWOOWEEEEWOOOO!
            if (!rescue_mode()) break; // Woops. Not yet.

            motion.linear.x = 4; // RACE TO THE RESCUE!!!!

            if (sus.pos > 0.4 && sus.pos < 0.6) {
              // They are kinda in the middle so we'll just drive straight
              motion.angular.z = 0;
            } else {
              // Decide to turn left or right
              motion.angular.z = 3.14 * (sus.pos < 0.5 ? 1 : -1);
            }

            geometry_pub->publish(motion);
            return; // Short-circuit it bc I'm too lazy to refactor this hot mess
          }
        }

        // No suspect detected. Go on with our lives
        normal_mode();      
        float min_range = msg->range_max;
        int i = 0;
        for (auto& range : msg->ranges) {
          if (range >= msg->range_min && range < min_range) {
            min_range = range;
          }
          i++;
        }

        if (min_range < SAFE_DISTANCE) {
          // Turn 180 or -180 from the obstacle -> should be < 6.14
          /* RCLCPP_INFO(this->get_logger(), "Robot %s found obstacle at range: %f | Turning: %f"
              , model_name.c_str(), min_range, turn_angle); */
          motion.linear.x = 0;
          motion.angular.z = 3.14;
        } else {
          motion.linear.x = 2;
          motion.angular.z = 0;
        }

        geometry_pub->publish(motion);        
      }

      void activation_buildup()
      {
        if (this->dead) return;
        // This is a simplication of the discrete function presented in the IEEE paper
        // We interpreted the control cycles as the frequency at which the activation will build up
        // For our purpose, we assume, as was interpreted by the paper's results, that 
        // the rate at which the activation builds up (h(x_i)) is the same across all fireflies
        // We further question this constraint, however. Perhaps it can be removed. Further testing is required.
        // With the initial activation being 0, a firefly shall flash after exactly 15s

        // Sorry, Bjarne! Gotta ditch RAII here... too sensitive to latency
        if (!this->activation_lock.try_lock()) return; 
        this->activation += ACTIVATION_TIME_INCR;

        if (activation > ACTIVATION_THRESHOLD) {
          std::thread(&FireFly::flash, this).detach();
          activation = 0;
        }
        this->activation_lock.unlock();
      }
      void flash()
      {
        // We are already flashing or we are rescuing someone or we died
        if (!flash_lock.try_lock() || this->rescuing || this->dead) return; 
        this->detect_failure = true; // When we're flashing, we are very suspicious of people who don't flash

        RCLCPP_INFO(this->get_logger(), "Lo! Robot %s is flashing!", model_name.c_str());
        this->flash_pub->publish(std_msgs::msg::Empty()); // LED ON        
        this->flash_count++;
        rclcpp::sleep_for(FLASH_DURATION);
        this->flash_pub->publish(std_msgs::msg::Empty()); // LED OFF

        this->detect_failure = false; 

        flash_lock.unlock();
      }
      
      void camera1_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
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

        this->extract_green(cv_ptr->image, true);
        this->extract_red(cv_ptr->image);
        this->extract_blue(cv_ptr->image);
      }

      void camera2_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
      {
        if (this->dead) return;
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
        
        this->extract_green(cv_ptr->image, false);
      }

      void extract_color_mask(const cv::Mat& img, cv::Mat& mask, int lowH, int highH)
      {
        // Credit: 
        // https://techvidvan.com/tutorials/detect-objects-of-similar-color-using-opencv-in-python/
        // https://www.opencv-srf.com/2010/09/object-detection-using-color-seperation.html
        // convert to hsv colorspace
        cv::Mat hsv;
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
        
        int iLowS = 20; 
        int iHighS = 255;

        int iLowV = 20;
        int iHighV = 255;

        cv::inRange(hsv, cv::Scalar(lowH, iLowS, iLowV), cv::Scalar(highH, iHighS, iHighV), mask);

        cv::Mat Kernel = cv::Mat(cv::Size(5,5),CV_8UC1,cv::Scalar(255));

        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, Kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, Kernel);
      }

      // Poor practice with this bool flag here. DO NOT ATTEMPT AT HOME!
      void extract_green(const cv::Mat& img, bool is_cam1)
      {
        cv::Mat mask;

        extract_color_mask(img, mask, 45, 90);

        int pix_count = cv::countNonZero(mask);

        unsigned int* prev_flash_ptr = (is_cam1 ? &this->cam1_prev_flashes : &this->cam2_prev_flashes);
        if (pix_count > 0) {
          // Reference: https://stackoverflow.com/questions/44749735/count-red-color-object-from-video-opencv-python
          std::vector<std::vector<cv::Point> > contours;
          std::vector<cv::Vec4i> hierarchy;

          cv::findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
          /* if (is_cam1) {
            double width = mask.size().width;
            std::vector<Suspect> still_sus;
            for (auto& sus : suspicions) {
              bool found = false;
              for (auto& contour : contours) { 
                auto moments = cv::moments(contour);
                double x = moments.m10 / moments.m00;
                double pos = x/width;
                if (sus.pos + POS_MARGIN > pos && sus.pos - POS_MARGIN < pos)
                  found = true;              
              }
              if (!found) {
                still_sus.push_back(sus); // This one still hasn't flashed again
              }
            }
            this->suspicions = std::move(still_sus);
          } */
          if (contours.size() > *prev_flash_ptr) {
            /* RCLCPP_INFO(this->get_logger(), "Robot %s detected %ld flashings vs %d in the last frame", 
                                        model_name.c_str(), contours.size(), *prev_flash_ptr); */

            const std::lock_guard<std::mutex> lock(this->activation_lock);            
            this->activation += (ACTIVATION_EPSILON * (contours.size() - *prev_flash_ptr));
            if (activation > ACTIVATION_THRESHOLD) {
              std::thread(&FireFly::flash, this).detach();
              activation = 0;
            }
            
            *prev_flash_ptr = contours.size();
          }
        } 
        else {
          *prev_flash_ptr = 0;
        }

        /* cv::imshow(this->model_name + (is_cam1 ? "1" : "2"), mask);
        cv::waitKey(10); */
      }

      void extract_blue(const cv::Mat& img)
      {
        // We'll just assume that dead robots don't flash
        // while acknowledging that this is insufficient to detect all types of failures
        // This project needs to end at one point
        // Should be no race here since we are mutually exclusive
        cv::Mat mask;

        extract_color_mask(img, mask, 95, 130);
        int pix_count = cv::countNonZero(mask);
        double width = mask.size().width;

        if (pix_count > 0) {
          std::vector<std::vector<cv::Point> > contours;
          std::vector<cv::Vec4i> hierarchy;
          cv::findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
          auto current_time = this->get_clock()->now();
          
          // First, for all our suspects, we need to confirm that they are still there
          std::vector<Suspect> still_sus;
          for (auto& sus : suspicions) {
            for (auto& contour : contours) { 
              auto moments = cv::moments(contour);
              double x = moments.m10 / moments.m00; // We only care about X for our purpose
              double pos = x/width;
              
              // We assume that each suspect might have moved a bit or we may have moved a bit
              if (sus.pos + POS_MARGIN > pos && sus.pos - POS_MARGIN < pos)
                still_sus.push_back({pos, sus.first_spotted, sus.flash_cycle});                
            }
          }

          if (this->detect_failure) {
            // We don't need to check timestamp here. LIDAR callback will do it for us!
            // Now, check to see if there are new suspects. 
            // My brain died at this point so here's an inverted loop
            for (auto& contour : contours) { 
              auto moments = cv::moments(contour);
              double x = moments.m10 / moments.m00; // We only care about X for our purpose
              double pos = x/width;
              bool found = false; // This robot is still in frame
              for (auto& sus : still_sus) 
                if (sus.pos == pos) found = true;
              
              if (!found) {
                // We don't care about stale flash_count. Prob
                still_sus.push_back({pos, current_time, this->flash_count});
                RCLCPP_INFO(this->get_logger(), "Robot %s became suspicious of peer at x = %lf", 
                                        model_name.c_str(), pos);
              }
            }
          }
          
          suspicions = std::move(still_sus);
        } 

      }

      void extract_red(const cv::Mat& img)
      {
        // Should be no race here since we are mutually exclusive
        cv::Mat mask;

        extract_color_mask(img, mask, 155, 180);
        int pix_count = cv::countNonZero(mask);
        found_red = (pix_count > 0);
      }

      std::string camera_topic1;
      std::string camera_topic2;
      std::string lidar_topic;
      rclcpp::TimerBase::SharedPtr publisher_timer;
      rclcpp::TimerBase::SharedPtr activation_timer;
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr geometry_pub;
      rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr flash_pub;
      rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr color_pub;
      rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_1;
      rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_2;
      rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr kill_switch_sub;
      rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;
      std::string model_name;
      unsigned int cam1_prev_flashes;
      unsigned int cam2_prev_flashes;
      int activation;
      std::mutex activation_lock;
      std::mutex flash_lock;
      bool rescuing = false;
      bool detect_failure = false;
      bool found_red = false;
      int flash_count;
      bool dead = false;

      size_t count_;

      struct Suspect {
        double pos; // Where did we see the out-of-sync robot?
        rclcpp::Time first_spotted;
        int flash_cycle; // We detected them at which flash_count?
      };

      std::vector<Suspect> suspicions;
};


int main(int argc, char ** argv)
{
  cv::startWindowThread();

  rclcpp::init(argc, argv);
  auto node = std::make_shared<FireFly>();
  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 4);
  exe.add_node(node);
  exe.spin();
  rclcpp::shutdown();
  
  return 0;
}