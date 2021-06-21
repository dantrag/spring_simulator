#include "ui/qcustomgraphicsscene.h"

#include <QGraphicsSceneMouseEvent>

void QCustomGraphicsScene::mousePressEvent(QGraphicsSceneMouseEvent* event) {
  if (event->buttons() & Qt::LeftButton) {
    drawingOn();
    switch (mode) {
      case MouseMode::kSelection : {
        if (selection)
          selection->setRect(event->scenePos().x(), event->scenePos().y(), 0, 0);
        else {
          selection = addRect(event->scenePos().x(), event->scenePos().y(), 0, 0, QPen(QBrush(Qt::darkBlue), 1));
          selection->setZValue(30);
        }
        break;
      }
      case MouseMode::kPassDrawing : {
        releaseSelection();
        if (pass)
          pass->setLine(QLineF(pass->line().p1(), QPointF(event->scenePos().x(), event->scenePos().y())));
        else {
          pass = addLine(event->scenePos().x(), event->scenePos().y(), event->scenePos().x(), event->scenePos().y(),
                         QPen(QBrush(Qt::darkMagenta), 1));
          pass->setZValue(30);
        }
        break;
      }
      default : {
        // should not happen
      }
    }
  }
  update();
}

void QCustomGraphicsScene::mouseMoveEvent(QGraphicsSceneMouseEvent* event) {
  if (isDrawingNow()) {
    if (event->buttons() & Qt::LeftButton) {
      switch (mode) {
        case MouseMode::kSelection : {
          selection->setRect(selection->rect().x(), selection->rect().y(),
                             event->scenePos().x() - selection->rect().x(),
                             event->scenePos().y() - selection->rect().y());
          break;
        }
        case MouseMode::kPassDrawing : {
          pass->setLine(QLineF(pass->line().p1(), QPointF(event->scenePos().x(), event->scenePos().y())));
          break;
        }
        default : {
          // should not happen
        }
      }
      update();
    } else {
      drawingOff();
      emit drawingFinished();
    }
  }
}

void QCustomGraphicsScene::mouseReleaseEvent(QGraphicsSceneMouseEvent*) {
  if (isDrawingNow()) {
    drawingOff();
    emit drawingFinished();
  }
}
