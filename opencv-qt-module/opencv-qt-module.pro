TEMPLATE = subdirs

SUBDIRS += \
    opencv-qt-crossplatform \
    opencv-qt-module-calibration \
    opencv-qt-module-test

INCLUDEPATH += C:\OpenCV\build\include

LIBS += C:\opencv-build\bin\libopencv_core455.dll
LIBS += C:\opencv-build\bin\libopencv_highgui455.dll
LIBS += C:\opencv-build\bin\libopencv_imgcodecs455.dll
LIBS += C:\opencv-build\bin\libopencv_imgproc455.dll
LIBS += C:\opencv-build\bin\libopencv_features2d455.dll
LIBS += C:\opencv-build\bin\libopencv_calib3d455.dll
