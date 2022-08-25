#ifndef CONSTANTS_H
#define CONSTANTS_H

// Глобальные константы
const int SCREENSHOT_DELAY = 3;  // Интервал между скриншотам
const int SCREENSHOT_COUNT = 50;  // Количество скриншотов
const int ID_WEBCAM_LEFT = 1;   // ID левой web-камеры
const int ID_WEBCAM_RIGHT = 0;   // ID правой web-камеры
const int CAMERA_TEST_SUCCESS = -1; // Код успешного завершения теста камер

// Размеры экрана для позиционирования главного окна
// TODO: Определить значения при инициализации программы
const int SCREEN_WIDTH = 1366;
const int SCREEN_HEIGHT = 768;

// Параметры калибровочной доски
const int CHESS_ROWS = 6; // Кол-во строк
const int CHESS_COLS = 9; // Кол-во столбцов
const double CHESS_SQUARE_SIZE = 0.024; // Размер клетки в мм

const bool DRAW_CORNERS_FLAG = false;

const int IMG_WIDTH_FULL = 640;
const int IMG_HEIGHT_FULL = 480;

const int IMG_WIDTH = 320;
const int IMG_HEIGHT = 240;

#endif // CONSTANTS_H
