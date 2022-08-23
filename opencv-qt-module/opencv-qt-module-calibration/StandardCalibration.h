#ifndef STANDARDCALIBRATION_H
#define STANDARDCALIBRATION_H

#include <iostream>
#include <filesystem>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include "Constants.h"

using namespace std;
namespace fs = std::filesystem;

class StandardCalibration
{
public:
    StandardCalibration();
    void calibrateSingleCamera(std::vector<std::vector<cv::Vec3f>> objpoints, std::vector<std::vector<cv::Vec2f>> imgpoints, std::string rightorleft, fs::path calibrationdatafolder);
    bool calibrateStereoCamera(fs::path calibrationdatafolder);
    void cameraCalibration();
};

#endif // STANDARDCALIBRATION_H
