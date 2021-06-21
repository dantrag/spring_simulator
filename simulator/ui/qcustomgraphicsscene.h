#ifndef QCUSTOMGRAPHICSSCENE_H
#define QCUSTOMGRAPHICSSCENE_H

#include <QGraphicsScene>
#include <QGraphicsRectItem>

class QCustomGraphicsScene : public QGraphicsScene {
  Q_OBJECT
public:
  QCustomGraphicsScene(QObject* parent) : QGraphicsScene(parent) {}

  enum MouseMode {
    kSelection = 0,
    kPassDrawing,
  };

  bool isDrawingNow() { return drawing_now; }
  void drawingOn() { drawing_now = true; }
  void drawingOff() { drawing_now = false; }
  void setMode(MouseMode new_mode) {
    mode = new_mode;
  }
  MouseMode currentMode() { return mode; }

  void releaseSelection() { if (selection) { removeItem(selection); delete selection; selection = nullptr; update(); } }
  void releasePass() { if (pass) { removeItem(pass); delete pass; pass = nullptr; update(); } }

  const QRectF getSelection() {
    if (selection)
      return selection->rect();
    else
      return QRectF();
  }
  const QLineF getPass() {
    if (pass)
      return pass->line();
    else
      return QLineF();
  }

  void mousePressEvent(QGraphicsSceneMouseEvent* event) override;
  void mouseMoveEvent(QGraphicsSceneMouseEvent* event) override;
  void mouseReleaseEvent(QGraphicsSceneMouseEvent*) override;

signals:
  void drawingFinished();

private:
  MouseMode mode = MouseMode::kSelection;
  bool drawing_now = false;

  QGraphicsRectItem* selection = nullptr;
  QGraphicsLineItem* pass = nullptr;
};

#endif // QCUSTOMGRAPHICSSCENE_H
