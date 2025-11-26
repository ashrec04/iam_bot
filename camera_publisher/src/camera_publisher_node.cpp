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
    // Topic name to publish the camera images
    // (kept consistent with the face_detector subscription topic)
    topic_name_ = "/camera/image_raw";

    // Create a publisher for sensor_msgs/Image messages
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name_, 10);

    // Open the default camera device (index 0 → usually /dev/video0)
    cap_.open(0);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(get_logger(),
                   "Failed to open camera (index 0). Is /dev/video0 available?");
      return;
    }

    // Attempt to retrieve the camera FPS (frames per second)
    double fps = cap_.get(cv::CAP_PROP_FPS);
    if (fps <= 0.0 || std::isnan(fps)) {
      // Many webcams do not report FPS, so default to 30 FPS
      fps = 30.0;
    }

    // Calculate timer period based on FPS
    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / fps));

    RCLCPP_INFO(get_logger(),
                "CameraPublisherNode started. Publishing on %s at ~%.1f FPS",
                topic_name_.c_str(), fps);

    // Create a wall timer that periodically grabs frames and publishes them
    timer_ = this->create_wall_timer(
      period,
      std::bind(&CameraPublisherNode::timerCallback, this));
  }

private:
  // Timer callback executed periodically based on the FPS
  void timerCallback()
  {
    if (!cap_.isOpened()) {
      return;
    }

    cv::Mat frame;
    cap_ >> frame;   // Capture one frame from the webcam
    if (frame.empty()) {
      RCLCPP_WARN(get_logger(), "Empty frame from camera");
      return;
    }

    // Convert OpenCV Mat → ROS Image message using cv_bridge
    auto msg = cv_bridge::CvImage(
      std_msgs::msg::Header(),    // Empty header, filled below
      "bgr8",                     // Standard OpenCV color format
      frame).toImageMsg();

    // Add timestamp and frame ID
    msg->header.stamp = this->get_clock()->now();
    msg->header.frame_id = "camera_frame";

    // Publish the image message
    publisher_->publish(*msg);
  }

  // Topic name for publishing
  std::string topic_name_;

  // ROS2 publisher for Image messages
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

  // Timer to control publishing rate
  rclcpp::TimerBase::SharedPtr timer_;

  // OpenCV camera capture object
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
