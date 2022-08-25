#ifndef KAEHLERBRADSKICALIBRATION_H
#define KAEHLERBRADSKICALIBRATION_H

#include <iostream>
#include <filesystem>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include "Constants.h"

using namespace std;
namespace fs = std::filesystem;

enum CameraPosition
{
    Left,
    Right
};

class KaehlerBradskiCalibration
{
public:
    KaehlerBradskiCalibration(string pathtocalibrationfolder, int imagewidth, int imageheight, int imagecount);
    void calibrateCamera();
    void calibrateSingleCamera(std::vector<std::vector<cv::Vec3f>> objpoints, std::vector<std::vector<cv::Vec2f>> imgpoints, CameraPosition position);
private:
    fs::path pathToCalibration;
    fs::path pathToLeft;
    fs::path pathToRight;

    int imageWidth;
    int imageHeight;
    int imageCount;
};

#endif // KAEHLERBRADSKICALIBRATION_H
