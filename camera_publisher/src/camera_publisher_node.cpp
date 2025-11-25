#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/opencv.hpp>

class CameraPublisherNode : public rclcpp::Node
{
public:
  CameraPublisherNode()
  : Node("camera_publisher_node")
  {
    // 要发布的图像 topic（和人脸识别节点保持一致）
    topic_name_ = "/camera/image_raw";
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name_, 10);

    // 打开默认摄像头（索引 0）
    cap_.open(0);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(get_logger(),
                   "Failed to open camera (index 0). Is /dev/video0 available?");
      return;
    }

    double fps = cap_.get(cv::CAP_PROP_FPS);
    if (fps <= 0.0 || std::isnan(fps)) {
      fps = 30.0;  // 获取不到就假定 30FPS
    }
    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / fps));

    RCLCPP_INFO(get_logger(),
                "CameraPublisherNode started. Publishing on %s at ~%.1f FPS",
                topic_name_.c_str(), fps);

    timer_ = this->create_wall_timer(
      period,
      std::bind(&CameraPublisherNode::timerCallback, this));
  }

private:
  void timerCallback()
  {
    if (!cap_.isOpened()) {
      return;
    }

    cv::Mat frame;
    cap_ >> frame;
    if (frame.empty()) {
      RCLCPP_WARN(get_logger(), "Empty frame from camera");
      return;
    }

    auto msg = cv_bridge::CvImage(
      std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    msg->header.stamp = this->get_clock()->now();
    msg->header.frame_id = "camera_frame";

    publisher_->publish(*msg);
  }

  std::string topic_name_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture cap_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

