#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using namespace std;

int main()
{
    cout << "Start opencv-qt-module-test" << endl;

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
