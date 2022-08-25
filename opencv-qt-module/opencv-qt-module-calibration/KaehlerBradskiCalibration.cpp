#include "KaehlerBradskiCalibration.h"

KaehlerBradskiCalibration::KaehlerBradskiCalibration(string pathtocalibrationfolder, int imagewidth, int imageheight, int imagecount)
{
    pathToCalibration = fs::path(pathtocalibrationfolder);
    pathToLeft = pathToCalibration / "left"; // .append("left") ;
    pathToRight = pathToCalibration / "right"; // .append("right") ;

    imageWidth = imagewidth;
    imageHeight = imageheight;
    imageCount = imagecount;
}

void KaehlerBradskiCalibration::calibrateCamera()
{
    // Размер калибровочной доски
    cv::Size CHECKERBOARD(CHESS_ROWS, CHESS_COLS);

    // Вектор векторов, каждый из которых содрежит координаты точек в одном из видов калибровочного шаблона
    // Координаты задаются в системе координат объекта, поэтому могут быть целыми числами в направлениях X и Y
    // и 0 в направлении Z (если результат калибровки нужен в физических единицах, то координаты тоже в физ. ед.)
    std::vector<cv::Vec3f> objPoints;
    for (int i = 0; i < CHECKERBOARD.height; i++)
    {
        for (int j = 0; j < CHECKERBOARD.width; j++)
        {
            objPoints.push_back(cv::Vec3f(j * CHESS_SQUARE_SIZE, i * CHESS_SQUARE_SIZE, 0));
        }
    }

    // Сохранение массива objectPoints в файл
    std::string fileObjectPoints = "objectPoints.xml";
    fs::path pathToObjPoints = (pathToCalibration / fileObjectPoints);

    cv::FileStorage fsObjPoints(pathToObjPoints.string(), cv::FileStorage::WRITE);
    if (fsObjPoints.isOpened())
        fsObjPoints << "objectPoints" << objPoints;

    // Вспомогательные объекты
    std::vector<std::vector<cv::Vec3f>> objPointsLeft;
    std::vector<std::vector<cv::Vec2f>> imgPointsLeft;
    std::vector<std::vector<cv::Vec3f>> objPointsRight;
    std::vector<std::vector<cv::Vec2f>> imgPointsRight;

    for (int imageIndex = 1; imageIndex <= imageCount; imageIndex++)
    {
        // Старт таймера
        auto start = chrono::steady_clock::now();

        cout << "Pair No " << std::setw(2) << std::setfill('0') << imageIndex << " proceccing: ";
        // Формируем имя текущего файл
        fs::path fileName ("frame_" + std::to_string(imageIndex) + ".jpg");

        // Путь к снимку с камеры
        fs::path leftFileName = pathToLeft / fileName;
        fs::path rightPathName = pathToRight / fileName;
        // Читаем файлы
        cv::Mat imgL = cv::imread(leftFileName.string());
        cv::Mat imgR = cv::imread(rightPathName.string());

        if (imgR.empty() || imgL.empty())
        {
            cout << "There are no images in pair No " << imageIndex << endl;
            continue;
        }

        // Преобразуем рисунок к оттенкам серого
        cv::Mat grayL;
        cv::cvtColor(imgL, grayL, cv::COLOR_BGR2GRAY);
        cv::Mat grayR;
        cv::cvtColor(imgR, grayR, cv::COLOR_BGR2GRAY);

        // Поиск углов шахматной доски
        std::vector<cv::Vec2f> cornersL;
        bool retL = cv::findChessboardCorners(grayL, CHECKERBOARD, cornersL, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FILTER_QUADS);
        std::vector<cv::Vec2f> cornersR;
        bool retR = cv::findChessboardCorners(grayR, CHECKERBOARD, cornersR, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FILTER_QUADS);

        if (retL && retR)
        {
            // Критерий завершения
            cv::TermCriteria termCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);

            // Корректировка углов
            cv::cornerSubPix(grayL, cornersL, cv::Size(11, 11), cv::Size(-1, -1), termCriteria);
            cv::cornerSubPix(grayR, cornersR, cv::Size(11, 11), cv::Size(-1, -1), termCriteria);

            // Сохранение точек в массивах
            objPointsLeft.push_back(objPoints);
            imgPointsLeft.push_back(cornersL);
            objPointsRight.push_back(objPoints);
            imgPointsRight.push_back(cornersR);

            auto end = chrono::steady_clock::now();
            auto diff = end - start;
            cout << "\tSUCCESS\t" << chrono::duration <double, milli> (diff).count() << " ms" << endl;
        }
        else
            cout << "\tFAIL ..." << endl;
    }

    cout << endl << "Cameras calibration ..." << endl;
    calibrateSingleCamera(objPointsLeft, imgPointsLeft, CameraPosition::Left);
    calibrateSingleCamera(objPointsRight, imgPointsRight, CameraPosition::Right);
}

void KaehlerBradskiCalibration::calibrateSingleCamera(std::vector<std::vector<cv::Vec3f>> objpoints, std::vector<std::vector<cv::Vec2f>> imgpoints, CameraPosition position)
{
    auto start = chrono::steady_clock::now();

    std::string cameraPosition;
    switch (position)
    {
    case CameraPosition::Left:
        cameraPosition = "Left";
        break;
    case CameraPosition::Right:
        cameraPosition = "Right";
        break;
    }

    cout << cameraPosition << " camera: ";

    cv::Mat intrinsic_matrix, distortion_coeffs;
    cv::Size image_size(imageWidth, imageHeight);

    // Калибровка камеры
    double rms = cv::calibrateCamera(
                objpoints,
                imgpoints,
                image_size,
                intrinsic_matrix,
                distortion_coeffs,
                cv::noArray(),
                cv::noArray(),
                cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_PRINCIPAL_POINT
                );

    // Построение карты коррекции дисторсии
    cv::Mat map1;
    cv::Mat map2;

    cv::initUndistortRectifyMap(
        intrinsic_matrix,
        distortion_coeffs,
        cv::Mat(),
        intrinsic_matrix,
        image_size,
        CV_16SC2,
        map1,
        map2
      );

    // Сохранение параметров в файл калибровки

    std::string fileCalibration = cameraPosition + "Camera.xml";

    fs::path pathToCalibrationFile = (pathToCalibration / fileCalibration);

    cv::FileStorage fsCalibrationFile(pathToCalibrationFile.string(), cv::FileStorage::WRITE);
    if (fsCalibrationFile.isOpened())
        fsCalibrationFile << "intrinsic_matrix" << intrinsic_matrix << "distortion_coeffs" << distortion_coeffs << "map1" << map1 << "map2" << map2;

    auto end = chrono::steady_clock::now();
    auto diff = end - start;
    cout << "RMS = " << rms << " (" << chrono::duration <double, milli> (diff/1000).count() << " sec)" << endl;
}
