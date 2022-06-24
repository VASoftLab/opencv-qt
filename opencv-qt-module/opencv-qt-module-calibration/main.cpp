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
//=============================================================================
// Глобальные константы
#define SCREENSHOT_DELAY 3  // Интервал между скриншотам
#define SCREENSHOT_COUNT 5  // Количество скриншотов
#define ID_WEBCAM_LEFT 0    // ID левой web-камеры
#define ID_WEBCAM_RIGHT 0   // ID правой web-камеры
#define CAMERA_TEST_SUCCESS -1 // Код успешного завершения теста камер
//=============================================================================
// Модуль теста камер
// Функция возвращает ID нерабочей камеры
// Если тест пройдет - возвращаем -1
//=============================================================================
int performCameraTest()
{
    cv::Mat3b frameLeft; // Снимок с левой камеры
    cv::VideoCapture captureLeft; // Поток левой камеры
    cv::Mat3b frameRight; // Снимок с правой камеры
    cv::VideoCapture captureRight; // Поток правой камеры
    cv::Mat3b frameCombined; // Комбинация кадров левая + правая

    // Открываем поток левой камеры
    captureLeft.open(ID_WEBCAM_LEFT, cv::CAP_ANY);
    // Открываем поток правой камеры
    captureRight.open(ID_WEBCAM_RIGHT, cv::CAP_ANY);

    //=========================================================================
    // Проверка камер
    //=========================================================================
    if (!captureLeft.isOpened())
    {
        cerr << "ERROR! Left camera not ready!" << endl;
        cin.get();
        return ID_WEBCAM_LEFT;
    }
    else
        cout << "LEFT camera test -- SUCCESS" << endl;

    if (!captureRight.isOpened())
    {
        cerr << "ERROR! Right сamera not ready!" << endl;
        cin.get();
        return ID_WEBCAM_RIGHT;
    }
    else
        cout << "RIGHT camera test -- SUCCESS" << endl;

    return CAMERA_TEST_SUCCESS; // Код успешного теста
}
//=============================================================================
// Модуль сбора изображений
//=============================================================================
void collectImagesForCalibration()
{
    cv::Mat3b frameLeft; // Снимок с левой камеры
    cv::VideoCapture captureLeft; // Поток левой камеры
    cv::Mat3b frameRight; // Снимок с правой камеры
    cv::VideoCapture captureRight; // Поток правой камеры
    cv::Mat3b frameCombined; // Комбинация кадров левая + правая
    cv::Mat3b framePairs; // Склейка камер без счетчика

    // Открываем поток левой камеры
    captureLeft.open(ID_WEBCAM_LEFT, cv::CAP_ANY);
    // Открываем поток правой камеры
    captureRight.open(ID_WEBCAM_RIGHT, cv::CAP_ANY);

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
        cv::imshow("WebCamCombined", frameCombined);

        if (cv::waitKey(5) == 27)
            break;
    }
    cv::destroyAllWindows();

    //=========================================================================
    // Работа с папкам для выходных снимков
    std::filesystem::path folderL = ("./output/left");    // Левая камера
    std::filesystem::path folderR = ("./output/right");   // Правая камера
    std::filesystem::path folderP = ("./output/pairs");   // Склейка камер
    // Очистка папок
    std::filesystem::remove_all(folderL);
    std::filesystem::remove_all(folderR);
    std::filesystem::remove_all(folderP);
    // Создание папок
    std::filesystem::create_directory(folderL);
    std::filesystem::create_directory(folderR);
    std::filesystem::create_directory(folderP);
    //=========================================================================
    // Переменные для хронометража
    auto startTime = std::chrono::high_resolution_clock().now();    // Начало
    auto nowTime = std::chrono::high_resolution_clock().now();      // Конец

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
        nowTime = std::chrono::high_resolution_clock().now();
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
                        cv::Point(5, 30), // Точка вывода текста
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
                        cv::Point(5, 30), cv::FONT_HERSHEY_SCRIPT_SIMPLEX,
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
        cv::imshow("LiveCamera", frameCombined);

        //=====================================================================
        // Сохраняем скриншот левой и правой камеры на диске
        if (screenshotFlagNeed && (!screenshotFlagDone))
        {
            screenshotCounter++; // Инкремент счетчика скриншотов
            std::filesystem::path fileName (
                        "frame_" + std::to_string(screenshotCounter) + ".jpg");
            std::filesystem::path fullPathL = folderL / fileName;
            std::filesystem::path fullPathR = folderR / fileName;
            std::filesystem::path fullPathP = folderP / fileName;

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
// Основная программа
//=============================================================================
int main()
{
    cout << "opencv-qt-module-calibration started..." << endl;

    // Тест работоспособности камер
    int testResult = performCameraTest();

    // Если тест пройден, собираем изображения
    if (testResult == CAMERA_TEST_SUCCESS)
        collectImagesForCalibration();

    cout << "Press [ENTER] to exit..." << endl;
    cin.get();

    return 0;
}
