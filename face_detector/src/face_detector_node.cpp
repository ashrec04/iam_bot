#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>

#include <sys/stat.h>   // for mkdir()
#include <sys/types.h>
#include <sstream>      // for stringstream (file names)
#include <iomanip>      // for setw, setfill
#include <algorithm>    // for std::max_element

// ================================================================
// FaceDetectorNode
// A ROS2 node that subscribes to camera images, performs face
// detection using OpenCV's Haar Cascade classifier, visualizes
// the results in a window, displays the detected face area,
// and saves frames when a face exceeds a defined pixel threshold.
// ================================================================
class FaceDetectorNode : public rclcpp::Node
{
public:
  FaceDetectorNode() 
  : Node("face_detector_node"), image_counter_(0)
  {
    // ---------------------------------------------
    // Load Haar Cascade classifier for face detection
    // ---------------------------------------------
    cascade_path_ = "/usr/share/opencv4/haarcascades/haarcascade_frontalface_alt2.xml";
    if (!face_cascade_.load(cascade_path_)) {
      RCLCPP_ERROR(get_logger(),
                   "Failed to load face cascade from: %s", cascade_path_.c_str());
    } else {
      RCLCPP_INFO(get_logger(),
                  "Loaded face cascade from: %s", cascade_path_.c_str());
    }

    // ---------------------------------------------
    // Subscription to the camera image topic
    // This is expected to be published by another node
    // (e.g., camera_publisher_node)
    // ---------------------------------------------
    std::string topic = "/camera/image_raw";

    sub_ = create_subscription<sensor_msgs::msg::Image>(
      topic,
      10,
      std::bind(&FaceDetectorNode::imageCallback,
                this,
                std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "FaceDetectorNode subscribed to %s", topic.c_str());

    // Create a window for displaying real-time output
    cv::namedWindow("Face Detection", cv::WINDOW_AUTOSIZE);

    // Create a directory for saving face images (ignore if exists)
    mkdir("saved_faces", 0777);
  }

private:

  // ================================================================
  // imageCallback()
  // Called every time a new image is received on the subscribed topic.
  // Performs:
  //   1. ROS → OpenCV image conversion
  //   2. Face detection
  //   3. Drawing bounding boxes
  //   4. Displaying face area in the window
  //   5. Saving frames when face area > threshold
  // ================================================================
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Convert ROS image message to OpenCV BGR8 format
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

    // Convert frame to grayscale and equalize histogram
    // Improves contrast and enhances detection performance
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(gray, gray);

    // ---------------------------------------------
    // Run Haar Cascade face detection
    // ---------------------------------------------
    std::vector<cv::Rect> faces;
    face_cascade_.detectMultiScale(
      gray,
      faces,
      1.05,        // scaleFactor: image is reduced by this rate at each scale
      2,          // minNeighbors: detections needed to confirm a face
      0,
      cv::Size(20, 20));  // minimum face size

    int max_area = 0;
    cv::Rect largest_face;

    // If one or more faces detected
    if (!faces.empty()) {
      for (const auto &r : faces) {
        // Draw green rectangle around the detected face
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

    // ---------------------------------------------
    // Overlay detected face area text on the frame
    // ---------------------------------------------
    std::string text;
    if (max_area > 0) {
      text = "Face area: " + std::to_string(max_area) + " px";
    } else {
      text = "Face area: N/A";
    }

    cv::putText(frame,
                text,
                cv::Point(10, 30),  // Position in the window
                cv::FONT_HERSHEY_SIMPLEX,
                0.7,
                cv::Scalar(0, 255, 255),   // Yellow-green color
                2);

    // ---------------------------------------------
    // Save the frame if the detected face area is large enough
    // Simulates “person is close enough” behavior
    // ---------------------------------------------
    const int AREA_THRESHOLD = 200;  // adjustable threshold

    if (max_area > AREA_THRESHOLD) {
      // Build filename saved_faces/face_0001.png
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

    // ---------------------------------------------
    // Update the visualisation window
    // ---------------------------------------------
    cv::imshow("Face Detection", frame);
    cv::waitKey(1);  // required for window refresh
  }

  // ================================================================
  // Private member variables
  // ================================================================
  std::string cascade_path_;
  cv::CascadeClassifier face_cascade_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

  int image_counter_;  // Used to increment saved image filenames
};

// ================================================================
// main()
// Initializes ROS, runs the node, and cleans up GUI windows.
// ================================================================
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FaceDetectorNode>());
  rclcpp::shutdown();
  cv::destroyAllWindows();   // Close all OpenCV windows before exit
  return 0;
}

