#ifndef QTOVERLOADS_H
#define QTOVERLOADS_H

#include <QDoubleSpinBox>

#if QT_DEPRECATED_SINCE(5, 14)
 #define QDoubleSpinBoxChanged static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged)
#else
 #define QDoubleSpinBoxChanged decltype(&QDoubleSpinBox::valueChanged)
#endif

#endif // QTOVERLOADS_H
