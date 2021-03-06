TEMPLATE = app
CONFIG += console c++17
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        main.cpp

win32 {
    INCLUDEPATH += D:\OpenCV\build\include

    LIBS += C:\opencv-build\bin\libopencv_core455.dll
    LIBS += C:\opencv-build\bin\libopencv_highgui455.dll
    LIBS += C:\opencv-build\bin\libopencv_imgcodecs455.dll
    LIBS += C:\opencv-build\bin\libopencv_features2d455.dll
    LIBS += C:\opencv-build\bin\libopencv_calib3d455.dll
    LIBS += C:\opencv-build\bin\libopencv_videoio455.dll
    LIBS += C:\opencv-build\bin\libopencv_imgproc455.dll
}
unix {
    INCLUDEPATH += /usr/include/opencv4
    LIBS += -L/usr/local/lib -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_videoio
}
