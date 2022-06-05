#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

int main()
{
    cout << "Start opencv-qt-module-test" << endl;

    cv::Mat image = cv::imread("..\\..\\opencv-qt-module\\opencv-qt-module-test\\images\\OpenCV.png");

    if (image.data)
    {
        cv::namedWindow("OpenCV");
        cv::imshow("OpenCV", image);
    }

    cv::waitKey();
    cv::destroyAllWindows();

    cout << "Press any key to exit..." << endl;
    cin.get();

    return 0;
}
