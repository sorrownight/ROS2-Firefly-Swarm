/**
 * @file camera_sub.cpp
 * @author Luan (Remi) Ta (luan.remita@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>

using std::placeholders::_1;

static const std::string OPENCV_WINDOW = "Image window";

class CameraSubscriber : public rclcpp::Node
{
  public:
    CameraSubscriber()
    : Node("camera_subscriber")
    {
        topic = "/world/swarm_world/model/turtle1/link/camera_link/sensor/wide_angle_camera/image";
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        topic, 10, std::bind(&CameraSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Image Received");
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
            cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);

    }
    std::string topic;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraSubscriber>());
    rclcpp::shutdown();
    return 0;
}