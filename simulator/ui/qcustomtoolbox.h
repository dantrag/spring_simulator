#ifndef QCUSTOMTOOLBOX_H
#define QCUSTOMTOOLBOX_H

#include <QToolBox>

class QCustomToolBox : public QToolBox {
  Q_OBJECT
 public:
  explicit QCustomToolBox(QWidget *parent = nullptr) : QToolBox(parent) {}

  QSize minimumSizeHint() const override;
};

#endif // QCUSTOMTOOLBOX_H
