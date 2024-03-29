TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        main.cpp \
    linuxsocketcan.cpp \
    EposCAN.cpp \
    windowsusb2can.cpp \
    canslave.cpp

HEADERS += \
    CANopenDefs.h \
    linuxsocketcan.h \
    EposCAN.h \
    windowsusb2can.h \
    canslave.h \
    object_dictionary.hpp

LIBS += -lpthread

# Deploy for raspberry pi
target.path = /home/pi
INSTALLS += target
