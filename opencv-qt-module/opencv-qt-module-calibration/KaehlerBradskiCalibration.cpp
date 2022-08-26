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
    std::vector<cv::Vec3f> objPointsVector;
    for (int i = 0; i < CHECKERBOARD.height; i++)
    {
        for (int j = 0; j < CHECKERBOARD.width; j++)
        {
            objPointsVector.push_back(cv::Vec3f((float)(i * CHESS_SQUARE_SIZE), (float)(j * CHESS_SQUARE_SIZE), 0.f));
        }
    }

    // Вспомогательные объекты
    std::vector<std::vector<cv::Vec3f>> objectPoints;
    std::vector<std::vector<cv::Vec2f>> imagePointsLeft;
    std::vector<std::vector<cv::Vec2f>> imagePointsRight;

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
            cv::TermCriteria termCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 1e-6);

            // Корректировка углов
            cv::cornerSubPix(grayL, cornersL, cv::Size(11, 11), cv::Size(-1, -1), termCriteria);
            cv::cornerSubPix(grayR, cornersR, cv::Size(11, 11), cv::Size(-1, -1), termCriteria);

            // Сохранение точек в массивах
            objectPoints.push_back(objPointsVector);
            imagePointsLeft.push_back(cornersL);
            imagePointsRight.push_back(cornersR);

            auto end = chrono::steady_clock::now();
            auto diff = end - start;
            cout << "\tSUCCESS\t" << chrono::duration <double, milli> (diff).count() << " ms" << endl;
        }
        else
            cout << "\tFAIL ..." << endl;
    }

    // Сохранение массива objectPoints в файл
    std::string fileObjectPoints = "objectPoints.xml";
    fs::path pathToObjPoints = (pathToCalibration / fileObjectPoints);

    cv::FileStorage fsObjPoints(pathToObjPoints.string(), cv::FileStorage::WRITE);
    if (fsObjPoints.isOpened())
        fsObjPoints << "objectPoints" << objectPoints;

    // Раздельная калибровка камер
    cout << endl << "Separate calibration ..." << endl;
    calibrateSingleCamera(objectPoints, imagePointsLeft, CameraPosition::Left);
    calibrateSingleCamera(objectPoints, imagePointsRight, CameraPosition::Right);

    // Калибровка стереокамеры
    cout << endl << "Stereo calibration ..." << endl;
    calibrateStereoCamera(objectPoints, imagePointsLeft, imagePointsRight);
}

void KaehlerBradskiCalibration::calibrateSingleCamera(std::vector<std::vector<cv::Vec3f>> objectPoints, std::vector<std::vector<cv::Vec2f>> imagePoints, CameraPosition position)
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

    cv::Mat cameraMatrix;
    cv::Mat newCameraMatrix;
    cv::Mat distCoeffs;
    cv::Size imageSize(imageWidth, imageHeight);

    // Калибровка камеры
    // Вариант с восьмью элементами выбирается, если поднят флаг cv::CALIB_RATIONAL_MODEL
    // и применяется для прецизионной калибровки экзотических объективов (c. 565)
    double rms = cv::calibrateCamera(
                objectPoints,
                imagePoints,
                imageSize,
                cameraMatrix,
                distCoeffs,
                cv::noArray(),
                cv::noArray(),
                cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_PRINCIPAL_POINT | cv::CALIB_RATIONAL_MODEL
                );


    newCameraMatrix = cv::getOptimalNewCameraMatrix(
                cameraMatrix,
                distCoeffs,
                imageSize,
                1,
                imageSize,
                0);

    // Построение карты коррекции дисторсии
    cv::Mat map1;
    cv::Mat map2;

    cv::initUndistortRectifyMap(
        cameraMatrix,
        distCoeffs,
        cv::Mat(),
        newCameraMatrix,
        imageSize,
        CV_16SC2,
        map1,
        map2
      );

    // Сохранение параметров в файл калибровки
    std::string fileCalibration = cameraPosition + "Camera.xml";
    fs::path pathToCalibrationFile = (pathToCalibration / fileCalibration);

    cv::FileStorage fsCalibrationFile(pathToCalibrationFile.string(), cv::FileStorage::WRITE);
    if (fsCalibrationFile.isOpened())
        fsCalibrationFile << "cameraMatrix" << cameraMatrix << "newCameraMatrix" << newCameraMatrix << "distCoeffs" << distCoeffs << "map1" << map1 << "map2" << map2;

    auto end = chrono::steady_clock::now();
    auto diff = end - start;
    cout << "RMS = " << rms << " (" << chrono::duration <double, milli> (diff/1000).count() << " sec)" << endl;
}

void KaehlerBradskiCalibration::calibrateStereoCamera(std::vector<std::vector<cv::Vec3f>> objPoints, std::vector<std::vector<cv::Vec2f>> imagePointsLeft, std::vector<std::vector<cv::Vec2f>> imagePointsRight)
{
    cv::Size imageSize(imageWidth, imageHeight);

    cv::Mat cameraMatrixLeft;
    cv::Mat distCoeffsLeft;
    cv::Mat cameraMatrixRight;
    cv::Mat distCoeffsRight;

    cv::Mat R;
    cv::Mat T;
    cv::Mat E;
    cv::Mat F;

    cv::Mat perViewErrors;
    int flags = cv::CALIB_FIX_INTRINSIC;
    cv::TermCriteria criteria(cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 30, 1e-6));

    // Читаем данные калибровки камер
    std::string fileCalibrationLeft = "LeftCamera.xml";
    fs::path pathToCalibrationFileLeft = (pathToCalibration / fileCalibrationLeft);
    cv::FileStorage fsLeft(pathToCalibrationFileLeft.string(), cv::FileStorage::READ);
    if (fsLeft.isOpened())
    {
        fsLeft["cameraMatrix"] >> cameraMatrixLeft;
        fsLeft["distCoeffs"] >> distCoeffsLeft;
        fsLeft.release();
    }

    std::string fileCalibrationRight = "RightCamera.xml";
    fs::path pathToCalibrationFileRight = (pathToCalibration / fileCalibrationRight);
    cv::FileStorage fsRight(pathToCalibrationFileRight.string(), cv::FileStorage::READ);
    if (fsRight.isOpened())
    {
        fsRight["cameraMatrix"] >> cameraMatrixRight;
        fsRight["distCoeffs"] >> distCoeffsRight;
        fsRight.release();
    }

    auto start = chrono::steady_clock::now();
    double rms = cv::stereoCalibrate(
                objPoints,
                imagePointsLeft,
                imagePointsRight,
                cameraMatrixLeft,
                distCoeffsLeft,
                cameraMatrixRight,
                distCoeffsRight,
                imageSize,
                R,
                T,
                E,
                F,
                perViewErrors,
                flags,
                criteria);

    // Ректификация откалиброванной стереопары
    cv::Mat RLeft; // Матрица ректификации левой камеры
    cv::Mat RRight; // Матрица ректификации правой камеры
    cv::Mat PLeft; // Матрица проекции левой камеры
    cv::Mat PRight; // Матрица проекции правой камеры
    cv::Mat Q; // Матрица отображения диспаратности на глубину
    double alpha = 1; // Параметр кадрирования от 0 до 1

    cv::stereoRectify(
                cameraMatrixLeft,
                distCoeffsLeft,
                cameraMatrixRight,
                distCoeffsRight,
                imageSize,
                R,
                T,
                RLeft,
                RRight,
                PLeft,
                PRight,
                Q,
                cv::CALIB_ZERO_DISPARITY,
                1);


    cv::Mat leftMap1;
    cv::Mat leftMap2;
    cv::Mat rightMap1;
    cv::Mat rightMap2;

    cv::initUndistortRectifyMap(
                cameraMatrixLeft,
                distCoeffsLeft,
                RLeft,
                PLeft,
                imageSize,
                CV_16SC2,
                leftMap1,
                leftMap2);

    cv::initUndistortRectifyMap(
                cameraMatrixRight,
                distCoeffsRight,
                RRight,
                PRight,
                imageSize,
                CV_16SC2,
                rightMap1,
                rightMap2);


    // Сохранение параметров в файл калибровки
    std::string fileCalibration = "StereoCamera.xml";
    fs::path pathToCalibrationFile = (pathToCalibration / fileCalibration);

    cv::FileStorage fsCalibrationFile(pathToCalibrationFile.string(), cv::FileStorage::WRITE);
    if (fsCalibrationFile.isOpened())
        fsCalibrationFile << "cameraMatrixLeft" << cameraMatrixLeft << "distCoeffsLeft" << distCoeffsLeft
                          << "cameraMatrixRight" << cameraMatrixRight << "distCoeffsRight" << distCoeffsRight
                          << "R" << R << "T" << T << "RLeft" << RLeft << "PLeft" << PLeft << "Q" << Q
                          << "leftMap1" << leftMap1 << "leftMap2" << leftMap2
                          << "rightMap1" << rightMap1 << "rightMap2" << rightMap2;

    auto end = chrono::steady_clock::now();
    auto diff = end - start;
    cout << "RMS = " << rms << " (" << chrono::duration <double, milli> (diff/1000).count() << " sec)" << endl;
}
