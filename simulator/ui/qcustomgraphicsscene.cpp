#include "ui/qcustomgraphicsscene.h"

#include <QGraphicsSceneMouseEvent>

void QCustomGraphicsScene::mousePressEvent(QGraphicsSceneMouseEvent *event) {
  selectingOn();
  if (selection)
    selection->setRect(event->scenePos().x(), event->scenePos().y(), 0, 0);
  else {
    selection = addRect(event->scenePos().x(), event->scenePos().y(), 0, 0, QPen(QBrush(Qt::darkBlue), 3));
    selection->setZValue(30);
  }
  update();
}

void QCustomGraphicsScene::mouseMoveEvent(QGraphicsSceneMouseEvent *event) {
  if (isSelectingNow()) {
    if (event->buttons() & Qt::LeftButton) {
      selection->setRect(selection->rect().x(), selection->rect().y(),
                         event->scenePos().x() - selection->rect().x(), event->scenePos().y() - selection->rect().y());
      update();
    } else {
      selectingOff();
    }
  }
}
