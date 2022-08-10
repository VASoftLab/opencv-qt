#include <iostream>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#define SCREEN_WIDTH    1366
#define SCREEN_HEIGHT   768

#define ID_WEBCAM_LEFT  0       // ID левой web-камеры
#define ID_WEBCAM_RIGHT 0       // ID правой web-камеры

// ============================================================================
// Раcкомментировать, если запуск под Linux
// #define IS_LINUX
// ============================================================================

using namespace std;


int main()
{
    cout << "Institution of Mechanical Engineers..." << endl;
    const cv::String winName = "Aerospace Engeneering";

    std::filesystem::path pathToImg = std::filesystem::current_path().parent_path().parent_path();
    pathToImg.append("images");
    const cv::String filName = (pathToImg / "aerospace.jpg").string();

    cv::Mat imgAero = cv::imread(filName, cv::IMREAD_COLOR);
    cv::namedWindow(winName, cv::WINDOW_NORMAL);
    cv::resizeWindow(winName, 600, 600);
    cv::moveWindow(winName,
                   (SCREEN_WIDTH - imgAero.size().width) / 2,
                   (SCREEN_HEIGHT - imgAero.size().height) / 2);
    cv::imshow(winName, imgAero);
    cv::waitKey(0);
    cv::destroyAllWindows();

    // ========================================================================
    // Camera test
    // ========================================================================
    cv::VideoCapture captureLeft;
#ifdef IS_LINUX
    captureLeft.open(ID_WEBCAM_LEFT, cv::CAP_GSTREAMER);
#else
    captureLeft.open(ID_WEBCAM_LEFT, cv::CAP_ANY);
#endif

    captureLeft.set(cv::CAP_PROP_FPS, 30);

    if (!captureLeft.isOpened())
    {
        cerr << "ERROR! Left camera not ready!" << endl;
        cin.get();
        return ID_WEBCAM_LEFT;
    }
    else
    {
        cout << "LEFT camera test -- SUCCESS" << endl;
    }

    cv::Mat frameLeft;

    for (;;)
    {
        captureLeft >> frameLeft;
        cv::imshow("Left Camera", frameLeft);

        if (cv::waitKey(5) >= 0)
            break;
    }
    cv::destroyAllWindows();

    return 0;
}
