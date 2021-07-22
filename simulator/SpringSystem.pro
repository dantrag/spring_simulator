QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17
CONFIG -= debug_and_release debug_and_release_target

QMAKE_CXXFLAGS_RELEASE -= -O2
QMAKE_CXXFLAGS_RELEASE *= -Ofast

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    backend/Actuator.cpp \
    backend/ElasticSimulator.cpp \
    backend/Heater.cpp \
    backend/InelasticSimulator.cpp \
    backend/Particle.cpp \
    backend/ParticleState.cpp \
    backend/Path.cpp \
    backend/Pusher.cpp \
    backend/Shape.cpp \
    backend/SimulatorSettings.cpp \
    backend/Spring.cpp \
    backend/SpringSimulator.cpp \
    backend/SpringSimulatorState.cpp \
    backend/SpringState.cpp \
    backend/WaxSimulator.cpp \
    backend/XMLIO.cpp \
    main.cpp \
    pugixml/pugixml.cpp \
    ui/mainwindow.cpp \
    ui/qactuatorwidget.cpp \
    ui/qcustomgraphicsscene.cpp \
    ui/qcustomtoolbox.cpp

HEADERS += \
    backend/Actuator.h \
    backend/ElasticSimulator.h \
    backend/Heater.h \
    backend/InelasticSimulator.h \
    backend/Particle.h \
    backend/ParticleState.h \
    backend/Path.h \
    backend/Pusher.h \
    backend/Shape.h \
    backend/SimulatorSettings.h \
    backend/Spring.h \
    backend/SpringSimulator.h \
    backend/SpringSimulatorState.h \
    backend/SpringState.h \
    backend/WaxSimulator.h \
    backend/XMLIO.h \
    mini/ini.h \
    pugixml/pugiconfig.hpp \
    pugixml/pugixml.hpp \
    ui/mainwindow.h \
    ui/qactuatorwidget.h \
    ui/qcustomgraphicsscene.h \
    ui/qcustomtoolbox.h \
    ui/qtoverloads.h

FORMS += \
    ui/mainwindow.ui \
    ui/qactuatorwidget.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
  ui/res/images.qrc

CONFIG(debug, debug|release) {
    DESTDIR = debug-x64
} else {
    DESTDIR = release-x64
}

OBJECTS_DIR = $${DESTDIR}/.obj
MOC_DIR = $${DESTDIR}/.moc
RCC_DIR = $${DESTDIR}/.rcc
UI_DIR = $${DESTDIR}/.ui
