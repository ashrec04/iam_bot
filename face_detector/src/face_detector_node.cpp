#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>

#include <sys/stat.h> 
#include <sys/types.h>
#include <sstream>
#include <iomanip>
#include <algorithm>

// ================================================================
// face_detector package
// performs face detection using OpenCV's Haar Cascade classifier
// shows result in a window, displaying a box around the detected face
// ================================================================
class FaceDetectorNode : public rclcpp::Node
{
public:
  FaceDetectorNode() 
  : Node("face_detector_node"), image_counter_(0)
  {
    //load Haar Cascade classifier
    cascade_path_ = "/usr/share/opencv4/haarcascades/haarcascade_fullbody.xml";
    if (!face_cascade_.load(cascade_path_)) {
      RCLCPP_ERROR(get_logger(),
                   "Failed to load face cascade from: %s", cascade_path_.c_str());
    } else {
      RCLCPP_INFO(get_logger(),
                  "Loaded face cascade from: %s", cascade_path_.c_str());
    }
    std::string topic = "/camera/image_raw";

    sub_ = create_subscription<sensor_msgs::msg::Image>(
      topic,
      10,
      std::bind(&FaceDetectorNode::imageCallback,
                this,
                std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "FaceDetectorNode subscribed to %s", topic.c_str());

    cv::namedWindow("Face Detection", cv::WINDOW_AUTOSIZE);

    mkdir("saved_faces", 0777);
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    //convert ROS image to OpenCV format
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat frame = cv_ptr->image;
    if (frame.empty()) {
      RCLCPP_WARN(get_logger(), "Empty frame received");
      return;
    }

    // convert to grayscale and equalize histogram
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(gray, gray);

    // run face detection
    std::vector<cv::Rect> faces;
    face_cascade_.detectMultiScale(
      gray,
      faces,
      1.05, 
      5,
      0,
      cv::Size(30, 30));

    int max_area = 0;
    cv::Rect largest_face;

    if (!faces.empty()) {
      for (const auto &r : faces) {
        //draw rectangle around detected face
        cv::rectangle(frame, r, cv::Scalar(0, 255, 0), 2);

        int area = r.width * r.height;
        if (area > max_area) {
          max_area = area;
          largest_face = r;
        }
      }

      RCLCPP_INFO(get_logger(),
                  "find a face (count = %zu), max area = %d px",
                  faces.size(), max_area);
    }

    std::string text;
    if (max_area > 0) {
      text = "Face area: " + std::to_string(max_area) + " px";
    } else {
      text = "Face area: N/A";
    }

    cv::putText(frame,
                text,
                cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX,
                0.7,
                cv::Scalar(0, 255, 255),
                2);

    //save frame if the face area is large enough
    const int AREA_THRESHOLD = 1500;  //save threshold

    if (max_area > AREA_THRESHOLD) {
      std::stringstream ss;
      ss << "saved_faces/face_"
         << std::setw(4) << std::setfill('0') << image_counter_++
         << ".png";

      std::string filename = ss.str();

      if (cv::imwrite(filename, frame)) {
        RCLCPP_INFO(get_logger(),
                    "Saved frame to %s (area = %d px)",
                    filename.c_str(), max_area);
      } else {
        RCLCPP_WARN(get_logger(),
                    "Failed to save frame to %s", filename.c_str());
      }
    }

    // udate viz window
    cv::imshow("Face Detection", frame);
    cv::waitKey(1);
  }

  std::string cascade_path_;
  cv::CascadeClassifier face_cascade_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

  int image_counter_;  // Used to increment saved image filenames
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FaceDetectorNode>());
  rclcpp::shutdown();
  cv::destroyAllWindows();   // Close all OpenCV windows before exit
  return 0;
}

