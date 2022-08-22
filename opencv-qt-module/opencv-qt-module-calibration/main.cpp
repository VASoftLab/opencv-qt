//==============================================================================
// Стандартные модули
//==============================================================================
#include <iostream>
#include <filesystem>
//==============================================================================
// Модули OpenCV
//==============================================================================
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//=============================================================================
#include "Constants.h"
#include "FishEyeCalibration.h"
#include "StandardCalibration.h"

#include <string>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <io.h>
//=============================================================================
// Подключаем пространства имен
using namespace std;
namespace fs = std::filesystem;
//=============================================================================

cv::Mat imgLeft;
cv::Mat imgRight;
cv::Mat disp;
cv::Mat disparity;

int numDisparity = 8;
int blockSize = 5;

cv::Ptr<cv::StereoSGBM> sbm = cv::StereoSGBM::create();

//=============================================================================
// Модуль теста камер
// Функция возвращает ID нерабочей камеры
// Если тест пройдет - возвращаем -1
//=============================================================================
int cameraTest()
{
    cv::Mat3b frameLeft; // Снимок с левой камеры
    cv::VideoCapture captureLeft; // Поток левой камеры
    cv::Mat3b frameRight; // Снимок с правой камеры
    cv::VideoCapture captureRight; // Поток правой камеры
    cv::Mat3b frameCombined; // Комбинация кадров левая + правая

    // Открываем поток левой камеры
    captureLeft.open(ID_WEBCAM_LEFT, cv::CAP_ANY);
    captureLeft.set(cv::CAP_PROP_FPS, 30);
    // Открываем поток правой камеры
    captureRight.open(ID_WEBCAM_RIGHT, cv::CAP_ANY);
    captureRight.set(cv::CAP_PROP_FPS, 30);

    //=========================================================================
    // Проверка камер
    //=========================================================================
    if (!captureLeft.isOpened())
    {
        cerr << "ERROR! Left camera not ready!" << endl;
        cout << "CAMERA TEST FAILED!";
        cin.get();
        return ID_WEBCAM_LEFT;
    }
    else
        cout << "LEFT camera test -- SUCCESS" << endl;

    if (!captureRight.isOpened())
    {
        cerr << "ERROR! Right camera not ready!" << endl;
        cout << "CAMERA TEST FAILED!";
        cin.get();
        return ID_WEBCAM_RIGHT;
    }
    else
        cout << "RIGHT camera test -- SUCCESS" << endl;

    cout << "CAMERA TEST SUCCESSED!" << endl << endl;

    return CAMERA_TEST_SUCCESS; // Код успешного теста
}
//=============================================================================
// Модуль сбора изображений
//=============================================================================
void collectImages()
{
    cv::Mat3b frameLeft; // Снимок с левой камеры
    cv::VideoCapture captureLeft; // Поток левой камеры
    cv::Mat3b frameRight; // Снимок с правой камеры
    cv::VideoCapture captureRight; // Поток правой камеры
    cv::Mat3b frameCombined; // Комбинация кадров левая + правая
    cv::Mat3b framePairs; // Склейка камер без счетчика

    // Открываем поток левой камеры
    captureLeft.open(ID_WEBCAM_LEFT, cv::CAP_ANY);
    captureLeft.set(cv::CAP_PROP_FPS, 30);
    // Открываем поток правой камеры
    captureRight.open(ID_WEBCAM_RIGHT, cv::CAP_ANY);
    captureRight.set(cv::CAP_PROP_FPS, 30);

    //=========================================================================
    // Тестовый запуск камер для позиционирования калибровочной доски
    //=========================================================================
    cout << "Set the calibration board inside the frame" << endl;
    cout << "Press [ESC] to start an image collection process" << endl;
    for (;;)
    {
        captureLeft >> frameLeft; // Захвата фрейма левой камеры
        captureRight >> frameRight; // Захвата фрейма правой камеры

        // Создание комбинированного фрейма
        cv::hconcat(frameLeft, frameRight, frameCombined);

        // Отображение комбинированного фрейма
        auto winName = "WebCamCombined";
        cv::imshow(winName, frameCombined);
        cv::moveWindow(winName,
                       (SCREEN_WIDTH - frameCombined.size().width) / 2,
                       (SCREEN_HEIGHT - frameCombined.size().height) / 2);

        if (cv::waitKey(5) == 27)
            break;
    }
    cv::destroyAllWindows();

    //=========================================================================
    // Работа с папкам для выходных снимков
    fs::path folderParent = fs::current_path();
    fs::path folderA = folderParent.parent_path().parent_path();

    fs::path folderL = (folderA / "output\\left");    // Левая камера
    fs::path folderR = (folderA / "output\\right");   // Правая камера
    fs::path folderP = (folderA / "output\\pairs");   // Склейка камер

    cout << "The application will save images to the folders:" << endl;
    cout << folderL << endl;
    cout << folderR << endl;
    cout << folderP << endl;

    // Очистка папок
    fs::remove_all(folderL);
    fs::remove_all(folderR);
    fs::remove_all(folderP);
    // Создание папок
    fs::create_directories(folderL);
    fs::create_directories(folderR);
    fs::create_directories(folderP);
    //=========================================================================
    // Переменные для хронометража
    auto startTime = chrono::high_resolution_clock().now();    // Начало
    auto nowTime = chrono::high_resolution_clock().now();      // Конец

    // Интервал между endTime и startTime
    auto timeInterval =
            chrono::duration_cast<chrono::milliseconds>(nowTime - startTime);

    //=========================================================================
    // Счетчики и флаги
    int timeCounter = 0; // Счетчик секунд
    int screenshotCounter = 0; // Счетчик скриншотов
    bool screenshotFlagNeed = false; // Флаг - нужно сделать скриншот
    bool screenshotFlagDone = false; // Флаг - скриншот сделан

    //=========================================================================
    // Переменные для отрисовки прямоугольника обратного счета
    int rectX = 0; // Координата X левого верхнего угла
    int rectY = 0; // Координата Y левого верхнего угла
    int rectW = 30; // Ширина прямоугольника
    int rectH = 40; // Высота прямоугольника

    rectX = frameCombined.size().width / 2 - rectW / 2;

    // Прямоугольник обратного отсчета
    cv::Rect rectCountDown(rectX, rectY, rectW, rectH);

    cout << "The image collection process started..." << endl;
    //=========================================================================
    // Создание тестовых снимков для калибровки камер
    //=========================================================================
    for (;;)
    {
        //=====================================================================
        // Захватываем левую камеру
        captureLeft.read(frameLeft);
        if (frameLeft.empty()) {
            cerr << "ERROR! blank frame for LEFT camer grabbed\n";
            break;
        }

        //=====================================================================
        // Захватываем правую камеру
        captureRight.read(frameRight);
        if (frameRight.empty()) {
            cerr << "ERROR! blank frame for RIGHT camer grabbed\n";
            break;
        }

        // Создание комбинированного фрейма
        cv::hconcat(frameLeft, frameRight, frameCombined);
        // Создаем копию для сохранения в файл
        framePairs = frameCombined.clone();

        //=====================================================================
        // Получаем текущее время и расчитываем интервал относительно старта
        nowTime = chrono::high_resolution_clock().now();
        timeInterval =
                chrono::duration_cast<chrono::milliseconds>
                (nowTime - startTime);

        //=====================================================================
        // Если прошла 1сек, то сбрасываем временную метку старта и увеличиваем
        // счетчик на 1. Если значение счетчика == SCREENSHOT_DELAY,
        // сбрасываем счетчик
        if (timeInterval.count() >= 1000)
        {
            startTime = nowTime;
            timeCounter =
                    (timeCounter == SCREENSHOT_DELAY) ? 0 : timeCounter + 1;
        }
        //=====================================================================
        // Если настало время снятия скриншота, рисуем красный прямоугольник
        if (timeCounter == SCREENSHOT_DELAY)
        {
            // Рисуем прямоугольник
            cv::rectangle(
                        frameCombined, // Фрейм, на котором рисуем
                        rectCountDown, // Прямоугольник рисования
                        cv::Scalar(0, 0, 255), // Код цвета (красный)
                        cv::FILLED); // Тип заливки (сплошная)
            // Выводим текст
            cv::putText(
                        frameCombined, // Фрейм на который выводим текст
                        to_string(timeCounter - SCREENSHOT_DELAY), // Текст
                        cv::Point(rectX + 5, 30), // Точка вывода текста
                        cv::FONT_HERSHEY_SCRIPT_SIMPLEX, // Шрифт
                        1, // Масштабирующий коэффициент для шрифта
                        cv::Scalar(0, 0, 255),
                        1, // Толщина линии
                        cv::LINE_AA // Тип линии
                        );

            // Устанавливаем флаг (Нужно сделать скриншот)
            screenshotFlagNeed = true;
        }
        else
        {
            // Рисуем белый закрашенный прямоугольник
            cv::rectangle(frameCombined,
                          rectCountDown,
                          cv::Scalar(255, 255, 255),
                          cv::FILLED);
            // Рисуем синюю границу вокруг прямоугольника
            cv::rectangle(frameCombined,
                          rectCountDown,
                          cv::Scalar(255, 0, 0),
                          1);
            // Выводим метку обратного отсчета
            // (кол-во секунд до следующего скриншота)
            cv::putText(frameCombined,
                        to_string(SCREENSHOT_DELAY - timeCounter),
                        cv::Point(rectX + 5, 30), cv::FONT_HERSHEY_SCRIPT_SIMPLEX,
                        1,
                        cv::Scalar(255, 0, 0),
                        1,
                        cv::LINE_AA);
            // Сбрасываем флаг необходимости сделать скриншот
            screenshotFlagNeed = false;
            // Сбрасываем флаг что скриншот уже сделан
            screenshotFlagDone = false;
        }

        //=====================================================================
        // Отображаем комбинированный скриншот (левая + правая)
        auto winName = "LiveCamera";
        cv::imshow(winName, frameCombined);
        cv::moveWindow(winName,
                       (SCREEN_WIDTH - frameCombined.size().width) / 2,
                       (SCREEN_HEIGHT - frameCombined.size().height) / 2);
        //=====================================================================
        // Сохраняем скриншот левой и правой камеры на диске
        if (screenshotFlagNeed && (!screenshotFlagDone))
        {
            screenshotCounter++; // Инкремент счетчика скриншотов
            fs::path fileName (
                        "frame_" + std::to_string(screenshotCounter) + ".jpg");
            fs::path fullPathL = folderL / fileName;
            fs::path fullPathR = folderR / fileName;
            fs::path fullPathP = folderP / fileName;

            // Сохраняем снимки с камер
            cv::imwrite(fullPathL.string(), frameLeft);
            cv::imwrite(fullPathR.string(), frameRight);
            cv::imwrite(fullPathP.string(), framePairs);

            //=================================================================
            // Выводим диагностическое сообщение в консоль
            auto t = std::time(nullptr);
            auto tm = *std::localtime(&t);
            cout << "Screenshot (" + std::to_string(screenshotCounter) +
                    "/" + std::to_string(SCREENSHOT_COUNT) + ") made at : "
                 << std::put_time(&tm, "%H:%M:%S") << std::endl;
            // Установка флага (Для текущего номера скриншот уже сделан)
            screenshotFlagDone = true;
        }

        //=====================================================================
        // Все скриншоты сделаны, завершаем работу
        if (screenshotCounter >= SCREENSHOT_COUNT)
            break;

        if (cv::waitKey(5) >= 0)
            break;
    }
    cv::destroyAllWindows();
}
//=============================================================================
// Модуль калибровки
//=============================================================================
void cameraCalibration()
{
    //FishEyeCalibration *feye = new FishEyeCalibration();
    //feye->cameraCalibration();
    //delete feye;

    StandardCalibration *stdcal = new StandardCalibration();
    stdcal->cameraCalibration();
    delete stdcal;
}
//=============================================================================
// Функция очистки консоли
//=============================================================================
void clearConsole()
{
#if defined _WIN32
    system("cls");
    //clrscr(); // including header file : conio.h
#elif defined (__LINUX__) || defined(__gnu_linux__) || defined(__linux__)
    system("clear");
#elif defined (__APPLE__)
    system("clear");
#endif
}
//=============================================================================
// Построение карты диспарантности
//=============================================================================
static void trackbar1(int , void* )
{
    sbm->setNumDisparities(numDisparity * 16);
    numDisparity = numDisparity * 16;
    sbm->compute(imgLeft, imgRight, disp);
    disp.convertTo(disparity, CV_8U);
    cv::applyColorMap(disparity, disparity, cv::COLORMAP_JET);
    cv::imshow("Disparity", disparity);
}

static void trackbar2(int , void* )
{
    sbm->setBlockSize(blockSize);
    // blockSize = blockSize;
    sbm->compute(imgLeft,imgRight, disp);
    disp.convertTo(disparity, CV_8U);
    cv::applyColorMap(disparity, disparity, cv::COLORMAP_JET);
    cv::imshow("Disparity", disparity);
}

void disparityMap()
{
    //imgLeft = cv::imread("D:\\TEMP\\imgLeft.png");
    //imgRight = cv::imread("D:\\TEMP\\imgRight.png");

    imgLeft = cv::imread("D:\\TEMP\\ambush_5_left.jpg");
    imgRight = cv::imread("D:\\TEMP\\ambush_5_right.jpg");

    //int numDisparity = 18;
    //int blockSize = 50;



    //sbm->setNumDisparities(numDisparity * 16);
    //sbm->setBlockSize(blockSize);

//    cv::Ptr<cv::StereoSGBM> sbm = cv::StereoSGBM::create(
//                -3,     // int minDisparity
//                96,     // int numDisparities
//                7,      // int SADWindowSize
//                60,     // int P1 = 0
//                2400,   // int P2 = 0
//                90,     // int disp12MaxDiff = 0
//                16,     // int preFilterCap = 0
//                1,      // int uniquenessRatio = 0
//                60,     // int speckleWindowSize = 0
//                20,     // int speckleRange = 0
//                true);  // bool fullDP = false

    //sbm->compute(imgLeft, imgRight, dispMap);

    //dispMap.convertTo(disparity, CV_8U);
    //cv::applyColorMap(disparity, disparity, cv::COLORMAP_JET);
    //cv::imshow("Disparity", disparity);

    cv::namedWindow("Disparity");
    cv::createTrackbar("numDisparities", "Disparity", &numDisparity, 18, trackbar1);
    cv::createTrackbar("blockSize", "Disparity", &blockSize, 50, trackbar2);

    cv::waitKey();
    cv::destroyAllWindows();



//    //cv::Mat imgLeft;
//    //cv::Mat imgRight;
//    cv::Mat greyLeft;
//    cv::Mat greyRight;

//    cv::Mat disp;
//    cv::Mat disp8;



//    cv::cvtColor(imgLeft, greyLeft, cv::COLOR_BGR2GRAY);
//    cv::cvtColor(imgRight, greyRight, cv::COLOR_BGR2GRAY);

//    cv::Mat imgDisparity16S = cv::Mat( imgLeft.rows, imgLeft.cols, CV_16S );
//    cv::Mat imgDisparity8U = cv::Mat( imgLeft.rows, imgLeft.cols, CV_8UC1 );

//    double minVal;
//    double maxVal;

//    cv::minMaxLoc( imgDisparity16S, &minVal, &maxVal );
//    printf("Min disp: %f Max value: %f \n", minVal, maxVal);

//    imgDisparity16S.convertTo( imgDisparity8U, CV_8UC1, 255/(maxVal - minVal));
//    cv::namedWindow( "Disparity", cv::WINDOW_NORMAL );
//    cv::imshow( "Disparity", imgDisparity8U );


}

void olegCalibration()
{
    int CHECKERBOARD[2] {6,9};

    // Creating vector to store vectors of 3D points for each checkerboard image
    std::vector<std::vector<cv::Point3f> > objpoints;

    // Creating vector to store vectors of 2D points for each checkerboard image
    std::vector<std::vector<cv::Point2f> > imgpointsL, imgpointsR;

    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;
    for(int i{0}; i<CHECKERBOARD[1]; i++)
    {
        for(int j{0}; j<CHECKERBOARD[0]; j++)
            objp.push_back(cv::Point3f(j,i,0));
    }

    // Extracting path of individual image stored in a given directory
    std::vector<std::string> imagesL, imagesR;
    // Path of the folder containing checkerboard images
    std::string pathL = "d:\\SourceCode\\opencv-qt\\output\\stereoL\\*.png";
    std::string pathR = "d:\\SourceCode\\opencv-qt\\output\\stereoR\\*.png";

    std::string File_Directory = "d:\\SourceCode\\opencv-qt\\output\\stereoL"; // каталог папки
    std::string File_Directory2 = "d:\\SourceCode\\opencv-qt\\output\\stereoR"; // каталог папки

    std::string FileType = ".png"; // Тип файла для поиска

    std::string result_File_Directory = "d:\\result"; // каталог папки
    std::vector<std::string> imgpaths;

    fs::path folderL = "d:\\SourceCode\\opencv-qt\\output\\left";    // Левая камера
    fs::path folderR = "d:\\SourceCode\\opencv-qt\\output\\right";;   // Правая камера

    for (int screenshotCounter = 1; screenshotCounter <= 50; screenshotCounter++)
    {
        fs::path fileName ("frame_" + std::to_string(screenshotCounter) + ".jpg");
        fs::path fullPathL = folderL / fileName;
        fs::path fullPathR = folderR / fileName;

        imagesL.push_back(fullPathL.string());
        imagesR.push_back(fullPathR.string());
    }

    cv::Mat frameL, frameR, grayL, grayR;
    // vector to store the pixel coordinates of detected checker board corners
    std::vector<cv::Point2f> corner_ptsL, corner_ptsR;
    bool successL, successR;

    // Looping over all the images in the directory
    for (int i{0}; i < (int)imagesL.size(); i++)
    {
        auto start = chrono::steady_clock::now();
        cout << "Pair No " << std::setw(2) << std::setfill('0') << i + 1 << " proceccing: ";

        frameL = cv::imread(imagesL[i]);
        cv::cvtColor(frameL, grayL, cv::COLOR_BGR2GRAY);

        frameR = cv::imread(imagesR[i]);
        cv::cvtColor(frameR, grayR, cv::COLOR_BGR2GRAY);

        // Finding checker board corners
        // If desired number of corners are found in the image then success = true

        successL = cv::findChessboardCorners(
                    grayL,
                    cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]),
                corner_ptsL);

        successR = cv::findChessboardCorners(
                    grayR,
                    cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]),
                corner_ptsR);

        if((successL) && (successR))
        {
            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
            // refining pixel coordinates for given 2d points.
            cv::cornerSubPix(grayL, corner_ptsL, cv::Size(11,11), cv::Size(-1,-1), criteria);
            cv::cornerSubPix(grayR, corner_ptsR, cv::Size(11,11), cv::Size(-1,-1), criteria);

            // Displaying the detected corner points on the checker board
            //cv::drawChessboardCorners(frameL, cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]), corner_ptsL,successL);
            //cv::drawChessboardCorners(frameR, cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]), corner_ptsR,successR);

            objpoints.push_back(objp);
            imgpointsL.push_back(corner_ptsL);
            imgpointsR.push_back(corner_ptsR);
        }

        //cv::imshow("ImageL",frameL);
        //cv::imshow("ImageR",frameR);
        //cv::waitKey(0);

        auto end = chrono::steady_clock::now();
        auto diff = end - start;
        cout << chrono::duration <double, milli> (diff).count() << " ms" << endl;
      }

      cv::destroyAllWindows();

      cv::Mat mtxL,distL,R_L,T_L;
      cv::Mat mtxR,distR,R_R,T_R;

      /*
        * Performing camera calibration by
        * passing the value of known 3D points (objpoints)
        * and corresponding pixel coordinates of the
        * detected corners (imgpoints)
      */

      cv::Mat new_mtxL, new_mtxR;

      auto startLeft = chrono::steady_clock::now();
      cout << endl << "Calibrating left camera ..." << endl;
      // Calibrating left camera
      double RLeft = cv::calibrateCamera(
                  objpoints,
                  imgpointsL,
                  grayL.size(),
                  mtxL,
                  distL,
                  R_L,
                  T_L);

      cout << "RMS re-projection error Left " << RLeft << endl;

      new_mtxL = cv::getOptimalNewCameraMatrix(
                  mtxL,
                  distL,
                  grayL.size(),
                  1,
                  grayL.size(),
                  0);

      auto endLeft = chrono::steady_clock::now();
      auto diffLeft = endLeft - startLeft;
      cout << "Left camera calibration: " << chrono::duration <double, milli> (diffLeft).count() << " ms" << endl;

      auto startRight = chrono::steady_clock::now();
      cout << endl << "Calibrating rigth camera ..." << endl;
      // Calibrating right camera
      double RRight = cv::calibrateCamera(
                  objpoints,
                  imgpointsR,
                  grayR.size(),
                  mtxR,
                  distR,
                  R_R,
                  T_R);

      cout << "RMS re-projection error Right " << RRight << endl;

      new_mtxR = cv::getOptimalNewCameraMatrix(
                  mtxR,
                  distR,
                  grayR.size(),
                  1,
                  grayR.size(),
                  0);

      auto endRight = chrono::steady_clock::now();
      auto diffRight = endRight - startRight;
      cout << "Right camera calibration: " << chrono::duration <double, milli> (diffRight).count() << " ms" << endl;

      // Here we fix the intrinsic camara matrixes so that only Rot, Trns, Emat and Fmat
      // are calculated. Hence intrinsic parameters are the same.
      cv::Mat Rot, Trns, Emat, Fmat;

      int flag = 0;
      flag |= cv::CALIB_FIX_INTRINSIC;


      // This step is performed to transformation between the two cameras and calculate Essential and
      // Fundamenatl matrix

      auto startStereo = chrono::steady_clock::now();
      cout << endl << "Calibrating stereo camera ..." << endl;
      cv::stereoCalibrate(
                  objpoints,
                  imgpointsL,
                  imgpointsR,
                  new_mtxL,
                  distL,
                  new_mtxR,
                  distR,
                  grayR.size(),
                  Rot,
                  Trns,
                  Emat,
                  Fmat,
                  flag,
                  cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 1e-6));

      cv::Mat rect_l, rect_r, proj_mat_l, proj_mat_r, Q;

      // Once we know the transformation between the two cameras we can perform
      // stereo rectification
      cv::stereoRectify(
                  new_mtxL,
                  distL,
                  new_mtxR,
                  distR,
                  grayR.size(),
                  Rot,
                  Trns,
                  rect_l,
                  rect_r,
                  proj_mat_l,
                  proj_mat_r,
                  Q,
                  1);

      // Use the rotation matrixes for stereo rectification and camera intrinsics for undistorting the image
      // Compute the rectification map (mapping between the original image pixels and
      // their transformed values after applying rectification and undistortion) for left and right camera frames
      cv::Mat Left_Stereo_Map1, Left_Stereo_Map2;
      cv::Mat Right_Stereo_Map1, Right_Stereo_Map2;

      cv::initUndistortRectifyMap(
                  new_mtxL,
                  distL,
                  rect_l,
                  proj_mat_l,
                  grayR.size(),
                  CV_16SC2,
                  Left_Stereo_Map1,
                  Left_Stereo_Map2);

      cv::initUndistortRectifyMap(
                  new_mtxR,
                  distR,
                  rect_r,
                  proj_mat_r,
                  grayR.size(),
                  CV_16SC2,
                  Right_Stereo_Map1,
                  Right_Stereo_Map2);

      auto endStereo = chrono::steady_clock::now();
      auto diffStereo = endStereo - startStereo;
      cout << "Stereo camera calibration: " << chrono::duration <double, milli> (diffStereo).count() << " ms" << endl;

      fs::path calibrationdatafolder = "d:\\SourceCode\\opencv-qt\\output\\calibration\\";

      // Результаты калибровки должны быть записаны в файл для дальнейшего использования
      std::string fileNameL = "calibration_camera_" + std::to_string(IMG_WIDTH) + "_" + std::to_string(IMG_HEIGHT) + "_left.yml";
      fs::path pathFullNameL = (calibrationdatafolder / fileNameL);
      cv::FileStorage fsL(pathFullNameL.string(), cv::FileStorage::WRITE);
      if (fsL.isOpened())
          fsL << "new_mtxL" << new_mtxL << "distL" << distL << "rect_l" << rect_l << "proj_mat_l" << proj_mat_l << "graySize" << grayR.size() << "Left_Stereo_Map1" << Left_Stereo_Map1 << "Left_Stereo_Map2" << Left_Stereo_Map2;

      std::string fileNameR = "calibration_camera_" + std::to_string(IMG_WIDTH) + "_" + std::to_string(IMG_HEIGHT) + "_right.yml";
      fs::path pathFullNameR = (calibrationdatafolder / fileNameR);
      cv::FileStorage fsR(pathFullNameR.string(), cv::FileStorage::WRITE);
      if (fsR.isOpened())
          fsR << "new_mtxR" << new_mtxR << "distR" << distR << "rect_r" << rect_r << "proj_mat_r" << proj_mat_r << "graySize" << grayR.size() << "Right_Stereo_Map1" << Right_Stereo_Map1 << "Right_Stereo_Map2" << Right_Stereo_Map2;
}
//=============================================================================
// Основная программа
//=============================================================================
int main()
{
    char modeCode = -1;
    cout << "opencv-qt-module-calibration started..." << endl;
    cout << endl;
    bool firsFlag = false;

    while (1)
    {
        if (firsFlag)
            cout << endl;
        else
            firsFlag = true;

        cout << ">>> SELECT MODE >>>" << endl;
        cout << "1:\t CAMERA TEST" << endl;
        cout << "2:\t COLLECT IMAGES" << endl;
        cout << "3:\t CAMERA CALIBRATION" << endl;
        cout << "4:\t DISPARITY MAP" << endl;
        cout << "5:\t OLEG" << endl;
        cout << "0:\t EXIT" << endl;
        cout << "YOUR CHOICE: ";

        cin.get(modeCode);
        cin.ignore(numeric_limits<streamsize>::max(), '\n');

        if (isdigit(modeCode))
        {
            switch (modeCode) {
            case 49:
                cameraTest();
                break;
            case 50:
                collectImages();
                break;
            case 51:
                cameraCalibration();
                break;
            case 52:
                disparityMap();
                break;
            case 53:
                olegCalibration();
                break;
            case 48:
                return 0;
                break;
            default:
                return 0;
                break;
            }
        }
        else
        {
            // Очистка экрана
            clearConsole();
            // cin.clear();
            return 0;
        }

    }

    return 0;
}
