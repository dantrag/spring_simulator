#include "ui/qcustomgraphicsscene.h"

#include <unordered_set>

#include <QGraphicsSceneMouseEvent>
#include <QGraphicsItem>
#include <QGraphicsTextItem>

QCustomGraphicsScene::QCustomGraphicsScene(QObject* parent, bool show_axes)
    : QGraphicsScene(parent), show_axes_(show_axes) {
  createAxes();
}

void QCustomGraphicsScene::createAxes() {
  auto color = QColor::fromHsv(0, 0, 220);
  x_axis = addLine(-10000, 0, 10000, 0, QPen(QBrush(color), 1));
  y_axis = addLine(0, -10000, 0, 10000, QPen(QBrush(color), 1));
  QFont font("Helvetica", 4);
  for (int i = -99; i <= 99; ++i) {
    ticks.push_back(addLine(i * 100, -4, i * 100, 4, QPen(QBrush(color), 1)));
    ticks.push_back(addLine(-4, i * 100, 4, i * 100, QPen(QBrush(color), 1)));
    if (i != 0) {
      auto x_text = addText(QString("%1").arg(i * 100), font);
      x_text->setX(i * 100 - x_text->boundingRect().width() / 2);
      x_text->setY(6);
      x_text->setDefaultTextColor(color);
      labels.push_back(x_text);

      auto y_text = addText(QString("%1").arg(i * 100), font);
      y_text->setY(i * 100 - y_text->boundingRect().height() / 2);
      y_text->setX(-6 - y_text->boundingRect().width());
      y_text->setDefaultTextColor(color);
      labels.push_back(y_text);
    } else {
      auto text = addText("0", font);
      text->setX(-6 - text->boundingRect().width());
      text->setY(6);
      text->setDefaultTextColor(color);
      labels.push_back(text);
    }
  }
  updateAxesVisibility();
}

void QCustomGraphicsScene::updateAxesVisibility() {
  if (x_axis != nullptr) x_axis->setVisible(show_axes_);
  if (y_axis != nullptr) y_axis->setVisible(show_axes_);
  for (auto tick : ticks) if (tick != nullptr) tick->setVisible(show_axes_);
  for (auto label : labels) if (label != nullptr)
    label->setVisible(show_axes_);
}

void QCustomGraphicsScene::clearField() {
  clear();
  ticks.clear();
  labels.clear();
  createAxes();
}

QRectF QCustomGraphicsScene::fieldBoundingRect() {
  QRectF rect;
  std::unordered_set<QGraphicsItem*> field_items;
  for (auto item : items()) field_items.insert(item);
  field_items.erase(x_axis);
  field_items.erase(y_axis);
  for (auto tick : ticks) field_items.erase(tick);
  for (auto label : labels) field_items.erase(label);
  for (auto item : field_items) {
    rect = rect.united(item->sceneBoundingRect());
  }

  // extend bounding rect by 33% for a nicer view
  auto rect_width = rect.width();
  auto rect_height = rect.height();
  rect.setLeft(rect.left() - rect_width / 3);
  rect.setRight(rect.right() + rect_width / 3);
  rect.setTop(rect.top() - rect_height / 3);
  rect.setBottom(rect.bottom() + rect_height / 3);
  return rect;
}

void QCustomGraphicsScene::mousePressEvent(QGraphicsSceneMouseEvent* event) {
  QGraphicsScene::mousePressEvent(event);
  if (event->buttons() & Qt::LeftButton) {
    drawingOn();
    switch (mode) {
      case MouseMode::kSelection : {
        break;
      }
      case MouseMode::kPassDrawing : {
        clearSelection();
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
  QGraphicsScene::mouseMoveEvent(event);

  double x = event->scenePos().x();
  double y = event->scenePos().y();
  emit mouseMoved(x, y);
  if (isDrawingNow()) {
    if (event->buttons() & Qt::LeftButton) {
      switch (mode) {
        case MouseMode::kSelection : {
          break;
        }
        case MouseMode::kPassDrawing : {
          pass->setLine(QLineF(pass->line().p1(), QPointF(x, y)));
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

void QCustomGraphicsScene::mouseReleaseEvent(QGraphicsSceneMouseEvent* event) {
  QGraphicsScene::mouseReleaseEvent(event);

  if (isDrawingNow()) {
    drawingOff();
    emit drawingFinished();
  }
}
