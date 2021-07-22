#ifndef QTOVERLOADS_H
#define QTOVERLOADS_H

#include <QDoubleSpinBox>

#if QT_VERSION < QT_VERSION_CHECK(5, 14, 0)
 #define QDoubleSpinBoxChanged static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged)
#else
 #define QDoubleSpinBoxChanged decltype(&QDoubleSpinBox::valueChanged)
#endif

#endif // QTOVERLOADS_H
