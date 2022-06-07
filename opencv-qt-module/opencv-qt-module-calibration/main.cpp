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
//==============================================================================
// Подключаем пространства имен
using namespace std;
//==============================================================================
// Глобальные константы
#define SCREENSHOT_DELAY 3  // Интервал между скриншотам
#define SCREENSHOT_COUNT 5  // Количество скриншотов
#define ID_WEBCAM_LEFT 1    // ID левой web-камеры
#define ID_WEBCAM_RIGHT 2   // ID правой web-камеры
//==============================================================================
// Основная программа
//==============================================================================
int main()
{
    cout << "opencv-qt-module started..." << endl;

    cv::Mat3b frameLeft; // Снимок с левой камеры
    cv::VideoCapture captureLeft; // Поток левой камеры
    cv::Mat3b frameRight; // Снимок с правой камеры
    cv::VideoCapture captureRight; // Поток правой камеры
    cv::Mat3b frameCombined; // Комбинация кадров левая + правая

    captureLeft.open(ID_WEBCAM_LEFT, cv::CAP_ANY); // Открываем поток левой камеры
    captureRight.open(ID_WEBCAM_RIGHT, cv::CAP_ANY); // Открываем поток правой камеры

    //=========================================================================
    // Проверка камер
    //=========================================================================
    if (!captureLeft.isOpened())
    {
        cerr << "ERROR! Left сamera not ready!" << endl;
        cin.get();
        return 1;
    }
    else
        cout << "LEFT camera test -- SUCCESS" << endl;

    if (!captureRight.isOpened())
    {
        cerr << "ERROR! Right сamera not ready!" << endl;
        cin.get();
        return 1;
    }
    else
        cout << "RIGHT camera test -- SUCCESS" << endl;
    //=========================================================================
    // Тестовый запуск камер для позиционирования калибровочной доски
    //=========================================================================
    cout << "Test web-camera stream started" << endl;
    cout << "Press [ESC] to start the calibration" << endl;
    for (;;)
    {
        captureLeft >> frameLeft; // Захвата фрейма левой камеры
        captureRight >> frameRight; // Захвата фрейма правой камеры
        // Создание комбинированного фрейма
        cv::hconcat(frameLeft, frameRight, frameCombined);
        // Отображение комбинированного фрейма
        cv::imshow("WebCamCombined", frameCombined);

        if (cv::waitKey(5) >= 0)
            break;
    }
    cv::destroyAllWindows();

    //=========================================================================
    // Работа с папкам для выходных снимков
    std::filesystem::path outputFolder = (".//output"); // Корневая папка
    std::filesystem::remove_all(outputFolder); // Очистка папки
    std::filesystem::create_directory(outputFolder); // Создание папки

    //=========================================================================
    // Переменные для хронометража
    auto startTime = std::chrono::high_resolution_clock().now(); // Начало
    auto nowTime = std::chrono::high_resolution_clock().now(); // Конец

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

        //=====================================================================
        // Получаем текущее время и расчитываем интервал относительно старта
        nowTime = std::chrono::high_resolution_clock().now();
        timeInterval = chrono::duration_cast<chrono::milliseconds>(nowTime - startTime);

        //=====================================================================
        // Если прошла 1 сек., то сбрасываем временную метку старта
        // и увеличиваем счетчик на 1. Если значение счетчика == SCREENSHOT_DELAY,
        // сбрасываем счетчик.
        if (timeInterval.count() >= 1000)
        {
            startTime = nowTime;
            timeCounter = (timeCounter == SCREENSHOT_DELAY) ? 0 : timeCounter + 1;
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
                        to_string(timeCounter - SCREENSHOT_DELAY), // Текст, который выводим
                        cv::Point(5, 30), // Точка вывода текста
                        cv::FONT_HERSHEY_SCRIPT_SIMPLEX, // Шрифт
                        1, // Масштабирующий коэффициент для шрифта
                        cv::Scalar(0, 0, 255),
                        1, // Толщина линии
                        cv::LINE_AA // Тип линии
                        );

            screenshotFlagNeed = true; // Устанавливаем флаг (Нужно сделать скриншот)
        }
        else
        {
            // Рисуем белый закрашенный прямоугольник
            cv::rectangle(frameCombined, rectCountDown, cv::Scalar(255, 255, 255), cv::FILLED); // Background
            // Рисуем синюю границу вокруг прямоугольника
            cv::rectangle(frameCombined, rectCountDown, cv::Scalar(255, 0, 0), 1); // Border
            // Выводим метку обратного отсчета (кол-во секунд до следующего скриншота)
            cv::putText(frameCombined,
                        to_string(SCREENSHOT_DELAY - timeCounter),
                        cv::Point(5, 30), cv::FONT_HERSHEY_SCRIPT_SIMPLEX,
                        1,
                        cv::Scalar(255, 0, 0),
                        1,
                        cv::LINE_AA);

            screenshotFlagNeed = false; // Сбрасываем флаг необходимости сделать скриншот
            screenshotFlagDone = false; // Сбрасываем флаг что скриншот уже сделан
        }

        //=====================================================================
        // Отображаем комбинированный скриншот (левая + правая)
        cv::imshow("LiveCamera", frameCombined);

        //=====================================================================
        // Сохраняем скриншот левой и правой камеры на диске
        if (screenshotFlagNeed && (!screenshotFlagDone))
        {
            screenshotCounter++; // Инкремент счетчика скриншотов
            std::filesystem::path fileLeft ("frameLeft_" + std::to_string(screenshotCounter) + ".jpg");
            std::filesystem::path fileRight ("frameRigth_" + std::to_string(screenshotCounter) + ".jpg");
            std::filesystem::path fullPathLeft = outputFolder / fileLeft; // Файл левой камеры
            std::filesystem::path fullPathRight = outputFolder / fileRight; // Файл правой камеры

            cv::imwrite(fullPathLeft.string(), frameLeft); // Сохраняем снимок левой камеры
            cv::imwrite(fullPathRight.string(), frameRight); // Сохраняем снимок правой камеры

            //=================================================================
            // Выводим диагностическое сообщение в консоль
            auto t = std::time(nullptr);
            auto tm = *std::localtime(&t);
            cout << "Screenshot (" + std::to_string(screenshotCounter) + "/" +std::to_string(SCREENSHOT_COUNT) +
                    ") made at : " << std::put_time(&tm, "%H:%M:%S") << std::endl;

            screenshotFlagDone = true; // Установка флага (Для текущего номера скриншот уже сделан)
        }

        //=====================================================================
        // Все скриншоты сделаны, завершаем работу
        if (screenshotCounter >= SCREENSHOT_COUNT)
            break;

        if (cv::waitKey(5) >= 0)
            break;
    }
    cv::destroyAllWindows();

    cout << "Press any key to exit..." << endl;
    cin.get();

    return 0;
}
