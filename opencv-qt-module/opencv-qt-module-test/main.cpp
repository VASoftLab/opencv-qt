#include <iostream>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using namespace std;

#define TIME_DELAY 3
#define SCREENSHOT_COUNT 10

int main()
{
    cout << "Start opencv-qt-module-test" << endl;

    std::filesystem::path outputFolder = (".//output");
    std::filesystem::remove_all(outputFolder);
    std::filesystem::create_directory(outputFolder);

    cv::Mat frame;
    cv::Mat clearFrame;
    cv::VideoCapture capture;

    int deviceID = 1; // Default Camera
    int apiID = cv::CAP_ANY;
    capture.open(deviceID, apiID);

    if (!capture.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    auto startTime = std::chrono::high_resolution_clock().now();
    auto nowTime = std::chrono::high_resolution_clock().now();
    auto timeInterval = chrono::duration_cast<chrono::milliseconds>(nowTime - startTime);

    int counter = 0;
    int screenshotCounter = 0;
    bool screenshotFlagNeed = false;
    bool screenshotFlagDone = false;

    //==========================================================================
    // Rectangle coordinates
    //==========================================================================
    int x = 0;
    int y = 0;
    int width = 30;
    int height = 40;
    cv::Rect rect(x, y, width, height);

    //==========================================================================
    // GRAB AND WRITE LOOP
    //==========================================================================
    cout << "Start grabbing. Press any key to terminate..." << endl;
    for (;;)
    {
        // Wait for a new frame from camera and store it into 'frame'
        capture.read(frame);
        // Check if we succeeded
        if (frame.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            break;
        }

        //=====================================================================
        // Time calculation
        nowTime = std::chrono::high_resolution_clock().now();
        timeInterval = chrono::duration_cast<chrono::milliseconds>(nowTime - startTime);

        if (timeInterval.count() >= 1000)
        {
            startTime = nowTime;
            counter = (counter == TIME_DELAY) ? 0 : counter + 1;
        }

        clearFrame = frame.clone();

        if (TIME_DELAY - counter == 0)
        {
            cv::rectangle(frame, rect, cv::Scalar(0, 0, 255), cv::FILLED);
            cv::putText(frame, to_string(TIME_DELAY - counter), cv::Point(5, 30), cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

            screenshotFlagNeed = true;
        }
        else
        {
            cv::rectangle(frame, rect, cv::Scalar(255, 255, 255), cv::FILLED); // Background
            cv::rectangle(frame, rect, cv::Scalar(255, 0, 0), 1); // Border
            cv::putText(frame, to_string(TIME_DELAY - counter), cv::Point(5, 30), cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1, cv::Scalar(255, 0, 0), 1, cv::LINE_AA);

            screenshotFlagNeed = false;
            screenshotFlagDone = false;
        }

        //=====================================================================
        // Show live and wait for a key with timeout long enough to show images
        cv::imshow("LiveCamera", frame);

        // Make a screenshot
        if (screenshotFlagNeed && (!screenshotFlagDone))
        {
            std::filesystem::path file ("frame_" + std::to_string(++screenshotCounter) + ".jpg");
            std::filesystem::path fullPath = outputFolder / file;
            cv::imwrite(fullPath.string(), clearFrame);

            auto t = std::time(nullptr);
            auto tm = *std::localtime(&t);
            cout << "Screenshot (" + std::to_string(screenshotCounter) + "/" + std::to_string(SCREENSHOT_COUNT) + ") made at : " << std::put_time(&tm, "%H:%M:%S") << std::endl;

            screenshotFlagNeed = false;
            screenshotFlagDone = true;
        }

        if (screenshotCounter >= SCREENSHOT_COUNT)
            break;

        if (cv::waitKey(5) >= 0)
            break;
    }
    //==========================================================================
    cv::destroyAllWindows();
    cout << "Done" << endl << "Press any key to exit..." << endl;
    cin.get();

    return 0;
}

int openCVactions()
{
    //==========================================================================
    // OpenCV Image Read
    //==========================================================================
    cv::Mat image = cv::imread("..\\..\\opencv-qt-module\\opencv-qt-module-test\\images\\OpenCV.png");

    if (image.data)
    {
        cv::namedWindow("OpenCV");
        cv::imshow("OpenCV", image);
    }

    cv::waitKey();
    cv::destroyAllWindows();

    cout << "Press any key to continue" << endl;
    cin.get();

    //==========================================================================
    // OpenCV WebCam Read
    //==========================================================================

    cv::Mat frame;
    cv::VideoCapture capture;

    int deviceID = 1; // Default Camera
    int apiID = cv::CAP_ANY;
    capture.open(deviceID, apiID);

    if (!capture.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    //==========================================================================
    // GRAB AND WRITE LOOP
    //==========================================================================
    cout << "Start grabbing" << endl << "Press any key to terminate" << endl;
    for (;;)
    {
        // Wait for a new frame from camera and store it into 'frame'
        capture.read(frame);
        // Check if we succeeded
        if (frame.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            break;
        }
        // Show live and wait for a key with timeout long enough to show images
        cv::imshow("LiveCamera", frame);
        if (cv::waitKey(5) >= 0)
            break;
    }
    //==========================================================================
    cv::destroyAllWindows();

    cout << "Press any key to continue" << endl;
    cin.get();

    //==========================================================================
    // OpenCV WebCam Read
    //==========================================================================

    cv::Mat3b frameLeft;
    cv::VideoCapture captureLeft;
    cv::Mat3b frameRight;
    cv::VideoCapture captureRight;

    cv::Mat3b frameCombined;

    int deviceIDLeft    = 1; // Left Camera
    int deviceIDRight   = 2; // Right Camera

    captureLeft.open(deviceIDLeft, cv::CAP_ANY);
    captureRight.open(deviceIDRight, cv::CAP_ANY);

    if (!captureLeft.isOpened())
    {
        cerr << "ERROR! Left Camera not ready!" << endl;
        return 1;
        cin.get();
    }

    if (!captureRight.isOpened())
    {
        cerr << "ERROR! Right Camera not ready!" << endl;
        return 1;
        cin.get();
    }

    for (;;)
    {
        captureLeft >> frameLeft;
        captureRight >> frameRight;
        cv::hconcat(frameLeft, frameRight, frameCombined);

        cv::imshow("WebCamCombined", frameCombined);
        if (cv::waitKey(5) >= 0)
            break;
    }
    cv::destroyAllWindows();
    //==========================================================================

    cout << "Press any key to exit..." << endl;
    cin.get();

    return 0;

}
