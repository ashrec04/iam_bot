// facedetect.cpp
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cout << "Usage:\n"
                  << "  ./facedetect image.jpg        # detect faces in a single image\n"
                  << "  ./facedetect video.mp4        # detect faces in a video file\n"
                  << "  ./facedetect camera           # use default webcam\n";
        return 0;
    }

    // Load Haar cascade (OpenCV provides this path)
    std::string cascade_path = cv::samples::findFile("haarcascades/haarcascade_frontalface_default.xml");
    cv::CascadeClassifier face_cascade;
    if (!face_cascade.load(cascade_path)) {
        std::cerr << "ERROR: Could not load cascade at " << cascade_path << std::endl;
        return -1;
    }

    std::string mode = argv[1];
    cv::VideoCapture cap;
    bool isImage = false;
    cv::Mat image;

    if (mode == "camera") {
        cap.open(0);
        if (!cap.isOpened()) {
            std::cerr << "ERROR: Cannot open default camera\n";
            return -1;
        }
    } else if (mode.size() >= 4 && 
              (mode.substr(mode.size()-4) == ".jpg" || mode.substr(mode.size()-4) == ".png" || mode.substr(mode.size()-4) == ".bmp")) {
        image = cv::imread(mode);
        if (image.empty()) {
            std::cerr << "ERROR: Cannot read image file: " << mode << std::endl;
            return -1;
        }
        isImage = true;
    } else {
        // treat as video file
        cap.open(mode);
        if (!cap.isOpened()) {
            std::cerr << "ERROR: Cannot open video file: " << mode << std::endl;
            return -1;
        }
    }

    cv::namedWindow("FaceDetect", cv::WINDOW_AUTOSIZE);

    auto processFrame = [&](cv::Mat &frame) {
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::equalizeHist(gray, gray);

        std::vector<cv::Rect> faces;
        face_cascade.detectMultiScale(gray, faces, 1.1, 5, 0|cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));

        for (const auto &r : faces) {
            cv::rectangle(frame, r, cv::Scalar(0,255,0), 2);
            cv::Point center(r.x + r.width/2, r.y + r.height/2);
            cv::circle(frame, center, 3, cv::Scalar(0,0,255), -1);
            cv::putText(frame, std::to_string(r.width) + "px", cv::Point(r.x, r.y - 6),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,0), 1);
        }
        cv::imshow("FaceDetect", frame);
    };

    if (isImage) {
        processFrame(image);
        std::cout << "Press any key in the image window to exit\n";
        cv::waitKey(0);
    } else {
        cv::Mat frame;
        while (true) {
            if (!cap.read(frame) || frame.empty()) break;
            processFrame(frame);
            char c = (char)cv::waitKey(10);
            if (c == 27 || c == 'q') break; // ESC or q to quit
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}

