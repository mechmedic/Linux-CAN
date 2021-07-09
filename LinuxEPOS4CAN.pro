TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        main.cpp \
    linuxsocketcan.cpp \
    EposCAN.cpp \
    windowsusb2can.cpp

HEADERS += \
    linuxsocketcan.h \
    candefines.h \
    EposCAN.h \
    windowsusb2can.h

LIBS += -lpthread

# Deploy for raspberry pi
target.path = /home/pi
INSTALLS += target
