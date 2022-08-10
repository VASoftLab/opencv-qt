#include "FishEyeCalibration.h"

FishEyeCalibration::FishEyeCalibration()
{

}

void FishEyeCalibration::calibrateSingleCamera(std::vector<std::vector<cv::Vec3f>> objpoints, std::vector<std::vector<cv::Vec2f>> imgpoints, std::string rightorleft, fs::path calibrationdatafolder)
{
    int N_OK = (int)objpoints.size();
    cv::Size DIM(IMG_WIDTH, IMG_HEIGHT);

    cv::Mat K; // = cv::Mat::zeros(3, 3, CV_32FC1);
    cv::Mat D; // = cv::Mat::zeros(4, 1, CV_32FC1);

    cv::Vec3f pt (0, 0, 0);
    //    std::vector<cv::Vec3f> rvecs(N_OK, pt);
    //    std::vector<cv::Vec3f> tvecs(N_OK, pt);
    cv::Mat rvecs = cv::Mat::zeros(N_OK, 1, CV_32FC3);
    cv::Mat tvecs = cv::Mat::zeros(N_OK, 1, CV_32FC3);

    cv::TermCriteria calibCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 10, 1e-6);
    int calibrationFlags = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC |  cv::fisheye::CALIB_FIX_SKEW;
    double rms = cv::fisheye::calibrate(objpoints, imgpoints, DIM, K, D, rvecs, tvecs, calibrationFlags, calibCriteria);
    cout << "RMS: " << rms << endl;

    cv::Mat map1;
    cv::Mat map2;

    cv::fisheye::initUndistortRectifyMap(K, D, pt, K, DIM, CV_16SC2, map1, map2);

    // Результаты калибровки должны быть записаны в файл для дальнейшего использования
    std::string fileName = "calibration_camera_" + std::to_string(IMG_WIDTH) + "_" + std::to_string(IMG_HEIGHT) + "_" + rightorleft + ".yml";
    fs::path pathFullName = (calibrationdatafolder / fileName);

    cv::FileStorage fs(pathFullName.string(), cv::FileStorage::WRITE);
    if (fs.isOpened())
        fs << "map1" << map1 << "map2" << map2 << "objpoints" << objpoints << "imgpoints" << imgpoints << "camera_matrix" << K << "distortion_coeff" << D;
}

bool FishEyeCalibration::calibrateStereoCamera(fs::path calibrationdatafolder, int resx, int resy)
{
    std::vector<std::vector<cv::Vec3f>> objectPoints;

    std::vector<std::vector<cv::Vec2f>>rightImagePoints;
    cv::Mat rightCameraMatrix;
    cv::Mat rightDistortionCoefficients;

    std::vector<std::vector<cv::Vec2f>> leftImagePoints;
    cv::Mat leftCameraMatrix;
    cv::Mat leftDistortionCoefficients;

    cv::Mat rotationMatrix;
    cv::Mat translationVector;

    cv::Size imageSize(resx, resy);

    cv::TermCriteria TERMINATION_CRITERIA(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.01);
    std::string rightorleft = "";

    for (int i = 0; i <= 1; i++)
    {
        if (i == 0)
            rightorleft = "left";
        else
            rightorleft = "right";

        std::string fileName = "calibration_camera_" + std::to_string(IMG_WIDTH) + "_" + std::to_string(IMG_HEIGHT) + "_" + rightorleft + ".yml";
        fs::path pathFullName = (calibrationdatafolder / fileName);

        cv::FileStorage fs(pathFullName.string(), cv::FileStorage::READ);
        if (fs.isOpened())
        {
            cv::Mat map1, map2;
            fs["map1"] >> map1;
            fs["map2"] >> map2;
            fs["objpoints"] >> objectPoints;

            if (rightorleft == "left")
            {
                fs["imgpoints"] >> leftImagePoints;
                fs["camera_matrix"] >> leftCameraMatrix;
                fs["distortion_coeff"] >> leftDistortionCoefficients;
            }
            else
            {
                fs["imgpoints"] >> rightImagePoints;
                fs["camera_matrix"] >> rightCameraMatrix;
                fs["distortion_coeff"] >> rightDistortionCoefficients;
            }

            fs.release();
        }
        else
        {
            cout << "Camera calibration data not found in cache." << endl;
            return false;
        }
    }

    int fisheyeFlags = 0;
    fisheyeFlags |= cv::fisheye::CALIB_FIX_INTRINSIC;
    // fisheyeFlags &= cv::fisheye::CALIB_CHECK_COND;

    double rms = cv::fisheye::stereoCalibrate(
                objectPoints,
                leftImagePoints,
                rightImagePoints,
                leftCameraMatrix,
                leftDistortionCoefficients,
                rightCameraMatrix,
                rightDistortionCoefficients,
                imageSize,
                rotationMatrix,
                translationVector,
                fisheyeFlags,
                TERMINATION_CRITERIA);

    cout << "RMS: " << rms << endl;

    cv::Mat R1, R2, P1, P2, Q;
    cv::fisheye::stereoRectify(leftCameraMatrix, leftDistortionCoefficients, rightCameraMatrix, rightDistortionCoefficients,
                               imageSize, rotationMatrix, translationVector, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, cv::Size(0, 0), 0, 0);

    cout << "Saving calibration..." << endl;
    cv::Mat leftMapX, leftMapY, rightMapX, rightMapY;
    cv::fisheye::initUndistortRectifyMap(leftCameraMatrix, leftDistortionCoefficients, R1, P1, imageSize, CV_16SC2, leftMapX, leftMapY);
    cv::fisheye::initUndistortRectifyMap(rightCameraMatrix, rightDistortionCoefficients, R2, P2, imageSize, CV_16SC2, rightMapX, rightMapY);

    std::string fileNameStereo = "stereo_calibration_camera_" + std::to_string(IMG_WIDTH) + "_" + std::to_string(IMG_HEIGHT) + ".yml";
    fs::path pathFullNameStereo = (calibrationdatafolder / fileNameStereo);

    cv::FileStorage fsWrite(pathFullNameStereo.string(), cv::FileStorage::WRITE);
    if (fsWrite.isOpened())
        fsWrite << "imageSize" << imageSize << "leftMapX" << leftMapX << "leftMapY" << leftMapY << "rightMapX" << rightMapX << "rightMapY" << rightMapY << "disparityToDepthMap" << Q;
    return true;
}

void FishEyeCalibration::cameraCalibration()
{
    cv::Mat graySmallLeft;
    cv::Mat graySmallRight;

    // Размер калибровочной доски
    cv::Size CHECKERBOARD(CHESS_ROWS, CHESS_COLS);

    // Критерий прекраения поиска
    cv::TermCriteria termCriteria(
                cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1);

    // Вспомогательные объекты
    cv::Vec3f objPoint(0, 0, 0);
    std::vector<cv::Vec3f> objPoints;

    // Матрица калибровочной доски
    for (int i = 0; i < CHECKERBOARD.height; i++)
    {
        for (int j = 0; j < CHECKERBOARD.width; j++)
        {
            objPoints.push_back(cv::Vec3f(j, i, 0));
        }
    }

    // Вспомогательные объекты
    std::vector<std::vector<cv::Vec3f>> objPointsLeft;
    std::vector<std::vector<cv::Vec2f>> imgPointsLeft;
    std::vector<std::vector<cv::Vec3f>> objPointsRight;
    std::vector<std::vector<cv::Vec2f>> imgPointsRight;

    // Работа с папкам для выходных снимков
    fs::path folderParent = fs::current_path();
    fs::path folderA = folderParent.parent_path().parent_path();

    fs::path folderL = (folderA / "output\\left");    // Левая камера
    fs::path folderR = (folderA / "output\\right");   // Правая камера
    fs::path folderP = (folderA / "output\\pairs");   // Склейка камер

    fs::path folderC = (folderA / "output\\calibration");
    fs::remove_all(folderC);
    fs::create_directories(folderC);

    string winNameL;
    string winNameR;

    for (int screenshotCounter = 1; screenshotCounter <= SCREENSHOT_COUNT; screenshotCounter++)
    {
        auto start = chrono::steady_clock::now();

        cout << "Pair No " << screenshotCounter << " proceccing: ";

        fs::path fileName (
                    "frame_" + std::to_string(screenshotCounter) + ".jpg");

        fs::path fullPathL = folderL / fileName;
        fs::path fullPathR = folderR / fileName;
        fs::path fullPathP = folderP / fileName;

        cv::Mat imgL = cv::imread(fullPathL.string());
        cv::Mat imgR = cv::imread(fullPathR.string());

        if (imgR.empty() || imgL.empty())
        {
            cout << "There are no images in pair No " << screenshotCounter << endl;
            continue;
        }

        // Для полной стереопары проводим обработку
        cv::Mat grayL;
        cv::cvtColor(imgL, grayL, cv::COLOR_BGR2GRAY);
        cv::resize (grayL, graySmallLeft, cv::Size(IMG_WIDTH, IMG_HEIGHT), cv::INTER_AREA);
        cv::Mat grayR;
        cv::cvtColor(imgR, grayR, cv::COLOR_BGR2GRAY);
        cv::resize(grayR, graySmallRight, cv::Size(IMG_WIDTH, IMG_HEIGHT), cv::INTER_AREA);

        // Поиск угловых точек на калибровочной доске
        std::vector<cv::Vec2f> cornersL;
        bool retL = cv::findChessboardCorners(grayL, CHECKERBOARD, cornersL, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FAST_CHECK + cv::CALIB_CB_NORMALIZE_IMAGE);
        std::vector<cv::Vec2f> cornersR;
        bool retR = cv::findChessboardCorners(grayR, CHECKERBOARD, cornersR, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FAST_CHECK + cv::CALIB_CB_NORMALIZE_IMAGE);

        auto end = chrono::steady_clock::now();
        auto diff = end - start;

        if (retL && retR)
            cout << "\tSUCCESS\t" << chrono::duration <double, milli> (diff).count() << " ms" << endl;
        else
            cout << "\tFAIL\t" << chrono::duration <double, milli> (diff).count() << " ms" << endl;

        // Отрисовка найденых углов
        if (DRAW_CORNERS_FLAG)
        {
            cv::drawChessboardCorners(imgL, CHECKERBOARD, cornersL, retL);
            winNameL = "Corners LEFT " + to_string(screenshotCounter);
            cv::imshow(winNameL, imgL);
            cv::moveWindow(winNameL, 0, 0);

            cv::drawChessboardCorners(imgR, CHECKERBOARD, cornersR, retR);
            winNameR = "Corners RIGHT " + to_string(screenshotCounter);
            cv::imshow(winNameR, imgR);
            cv::moveWindow(winNameR, IMG_WIDTH_FULL + 5, 0);

            char key = cv::waitKey();

            if (key == 'q' || key == 'Q')
                exit(1);

            cv::destroyAllWindows();
        }

        // Авторы алгоритма предлагают использовать разные разрешения для калибровки - HI Resolution
        // и для реальной работы - LO Resolution
        if ((retL && retR) && (IMG_HEIGHT <= IMG_HEIGHT_FULL))
        {
            float scale_ratio = (float)IMG_HEIGHT/IMG_HEIGHT_FULL;
            for (unsigned int i = 0; i < cornersL.size(); i++)
                cornersL[i] *= scale_ratio;
            for (unsigned int i = 0; i < cornersR.size(); i++)
                cornersR[i] *= scale_ratio;
        }
        else if (IMG_HEIGHT > IMG_HEIGHT_FULL)
        {
            cout << "Image resolution is higher than photo resolution, upscale needed. Please check your photo and image parameters!" << endl;
            exit (0);
        }

        // Уточнение координат углов калибровки
        if (retL && retR)
        {
            objPointsLeft.push_back(objPoints);
            cv::cornerSubPix(graySmallLeft, cornersL, cv::Size(3, 3), cv::Size(-1, -1), termCriteria);
            imgPointsLeft.push_back(cornersL);
            objPointsRight.push_back(objPoints);
            cv::cornerSubPix(graySmallRight, cornersR, cv::Size(3, 3), cv::Size(-1, -1), termCriteria);
            imgPointsRight.push_back(cornersR);
        }
        else
        {
            // cout << "Pair No " << screenshotCounter << " ignored, as no chessboard found." << endl;
            continue;
        }
    }

    // Калибровка камер по отдельности и калибровка стереопары
    cout << endl;

    cout << "Left camera calibration..." << endl;
    calibrateSingleCamera(objPointsLeft, imgPointsLeft, "left", folderC);
    cout << endl;

    cout << "Right camera calibration..." << endl;
    calibrateSingleCamera(objPointsRight, imgPointsRight, "right", folderC);
    cout << endl;

    cout << "Stereo camera calibration..." << endl;
    calibrateStereoCamera(folderC);
    cout << endl;

    cout << "Calibration complete!" << endl;
}
