include(../dir.pri)

win32:TEMPLATE = vclib
unix:TEMPLATE = lib

CONFIG -= qt
CONFIG += dll 

unix {
	INCLUDEPATH += ../../libusb-unix
	INCLUDEPATH += ../../libusb-unix/libusb
	INCLUDEPATH += /usr/include/libusb-1.0/
	LIBS += -L../../bin
	LIBS += -L../../bin/lib
	
}

win32 {
	LIBS += setupapi.lib
	
	contains(QMAKE_HOST.arch, x86_64) {
		LIBS += ../../libusb-win32/MS64/static/libusb-1.0.lib 
	} else {
		LIBS += ../../libusb-win32/MS32/static/libusb-1.0.lib 
	}
	
	DEFINES += LIBUSB10
	INCLUDEPATH += ../../libusb-win32
} 

DEFINES += INSIDE_Z800

INCLUDEPATH += ../math

CONFIG(debug, debug|release) {
	LIBS += -L../math/Debug
	LIBS += -L../math -lmathD
	unix:LIBS += -lusb-1.0
	OBJECTS_DIR = Debug 
	DESTDIR = ../../bin
	TARGET = z800D
} else {
	LIBS += -L../math/Release
	LIBS += -L../math -lmath
	OBJECTS_DIR = Release
	unix:LIBS += -lusb-1.0
	DESTDIR = ../../bin
	TARGET = z800
}

SOURCES += Device.cpp \
	Calc.cpp \
	Timer.cpp \
	libz800.cpp \
	Property.cpp \
	QuatKalmanFilter.cpp

HEADERS += Device.h \
	Calc.h \
	Timer.h \
	libz800.h \
	Property.h \
	QuatKalmanFilter.h

doc.target = doc
doc.commands = rm -Rf ../../doc && doxygen && cp -f ../doxygen_style/* ../../doc/html
doc.depends = FORCE
QMAKE_EXTRA_TARGETS += doc

