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
//=============================================================================
// Подключаем пространства имен
using namespace std;
namespace fs = std::filesystem;
//=============================================================================
// Глобальные константы
#define SCREENSHOT_DELAY 3  // Интервал между скриншотам
#define SCREENSHOT_COUNT 5  // Количество скриншотов
#define ID_WEBCAM_LEFT 1   // ID левой web-камеры
#define ID_WEBCAM_RIGHT 0   // ID правой web-камеры
#define CAMERA_TEST_SUCCESS -1 // Код успешного завершения теста камер

// Размеры экрана для позиционирования главного окна
// TODO: Определить значения при инициализации программы
#define SCREEN_WIDTH    1366
#define SCREEN_HEIGHT   768

// Параметры калибровочной доски
#define CHESS_ROWS 3
#define CHESS_COLS 5
#define SQUARE_SIZE 3.0

#define DRAW_CORNERS_FLAG false

int IMG_WIDTH_FULL = 640;
int IMG_HEIGHT_FULL = 480;

int IMG_WIDTH = 320;
int IMG_HEIGHT = 240;

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

void calibrateSingleCamera(std::vector<std::vector<cv::Vec3f>> objpoints, std::vector<std::vector<cv::Vec2f>> imgpoints, std::string rightorleft, std::string calibrationdatafolder)
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
    cv::FileStorage fs(calibrationdatafolder + "calibration_camera_" + std::to_string(IMG_WIDTH) + "_" + std::to_string(IMG_HEIGHT) + "_" + rightorleft + ".yml", cv::FileStorage::WRITE);
    if (fs.isOpened())
        fs << "map1" << map1 << "map2" << map2 << "objpoints" << objpoints << "imgpoints" << imgpoints << "camera_matrix" << K << "distortion_coeff" << D;
}

bool calibrateStereoCamera(std::string calibrationdatafolder, int resx = IMG_WIDTH, int resy = IMG_HEIGHT)
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
    std::string rightOrLeft = "";

    for (int i = 0; i <= 1; i++)
    {
        if (i == 0)
            rightOrLeft = "left";
        else
            rightOrLeft = "right";

        cv::FileStorage fs(calibrationdatafolder + "calibration_camera_" + std::to_string(IMG_WIDTH) + "_" + std::to_string(IMG_HEIGHT) + "_" + rightOrLeft + ".yml", cv::FileStorage::READ);
        if (fs.isOpened())
        {
            cv::Mat map1, map2;
            fs["map1"] >> map1;
            fs["map2"] >> map2;
            fs["objpoints"] >> objectPoints;
            if (rightOrLeft == "left")
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
    cout << "Calibrating cameras together..." << endl;
    int fisheyeFlags = 0;
    fisheyeFlags |= cv::fisheye::CALIB_FIX_INTRINSIC;
    //fisheyeFlags &= cv::fisheye::CALIB_CHECK_COND;
    double rms = cv::fisheye::stereoCalibrate(objectPoints, leftImagePoints, rightImagePoints, leftCameraMatrix, leftDistortionCoefficients,
                                              rightCameraMatrix, rightDistortionCoefficients, imageSize, rotationMatrix, translationVector,
                                              fisheyeFlags, TERMINATION_CRITERIA);

    cout << "RMS: " << rms << endl;

    cv::Mat R1, R2, P1, P2, Q;
    cv::fisheye::stereoRectify(leftCameraMatrix, leftDistortionCoefficients, rightCameraMatrix, rightDistortionCoefficients,
                               imageSize, rotationMatrix, translationVector, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, cv::Size(0, 0), 0, 0);

    cout << "Saving calibration..." << endl;
    cv::Mat leftMapX, leftMapY, rightMapX, rightMapY;
    cv::fisheye::initUndistortRectifyMap(leftCameraMatrix, leftDistortionCoefficients, R1, P1, imageSize, CV_16SC2, leftMapX, leftMapY);
    cv::fisheye::initUndistortRectifyMap(rightCameraMatrix, rightDistortionCoefficients, R2, P2, imageSize, CV_16SC2, rightMapX, rightMapY);

    cv::FileStorage fsWrite(calibrationdatafolder + "stereo_camera_calibration" + std::to_string(IMG_WIDTH) + "_" + std::to_string(IMG_HEIGHT) + ".yml", cv::FileStorage::WRITE);
    if (fsWrite.isOpened())
        fsWrite << "imageSize" << imageSize << "leftMapX" << leftMapX << "leftMapY" << leftMapY << "rightMapX" << rightMapX <<
                   "rightMapY" << rightMapY << "disparityToDepthMap" << Q;
    return true;
}

void cameraCalibration()
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

        if (retL && retR)
            cout << "SUCCESS" << endl;
        else
            cout << "FAIL" << endl;

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

    std::string calibrationDataFolder = folderC.string();

    cout << "Left camera calibration..." << endl;
    calibrateSingleCamera(objPointsLeft, imgPointsLeft, "left", calibrationDataFolder);
    cout << "Right camera calibration..." << endl;
    calibrateSingleCamera(objPointsRight, imgPointsRight, "right", calibrationDataFolder);
    cout << "Stereoscopic calibration..." << endl;
    calibrateStereoCamera(calibrationDataFolder);
    cout << "Calibration complete!" << endl;

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
// Основная программа
//=============================================================================
int main()
{
    char modeCode = -1;
    cout << "opencv-qt-module-calibration started..." << endl;
    cout << endl;

    while (1)
    {
        cout << ">>> SELECT MODE >>>" << endl;
        cout << "1:\t CAMERA TEST" << endl;
        cout << "2:\t COLLECT IMAGES" << endl;
        cout << "3:\t CAMERA CALIBRATION" << endl;
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
            // clearConsole();
            // cin.clear();
        }

    }

    return 0;
}
