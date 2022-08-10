#ifndef FISHEYECALIBRATION_H
#define FISHEYECALIBRATION_H

#include <iostream>
#include <filesystem>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include "Constants.h"

using namespace std;
namespace fs = std::filesystem;

class FishEyeCalibration
{
public:
    FishEyeCalibration();
    void calibrateSingleCamera(std::vector<std::vector<cv::Vec3f>> objpoints, std::vector<std::vector<cv::Vec2f>> imgpoints, std::string rightorleft, fs::path calibrationdatafolder);
    bool calibrateStereoCamera(fs::path calibrationdatafolder, int resx = IMG_WIDTH, int resy = IMG_HEIGHT);
    void cameraCalibration();
};

#endif // FISHEYECALIBRATION_H
