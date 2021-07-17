#ifndef QCUSTOMGRAPHICSSCENE_H
#define QCUSTOMGRAPHICSSCENE_H

#include <vector>

#include <QGraphicsScene>
#include <QGraphicsRectItem>

class QCustomGraphicsScene : public QGraphicsScene {
  Q_OBJECT
public:
  QCustomGraphicsScene(QObject* parent, bool show_axes);

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
  void setAxesVisibility(bool visible) { show_axes_ = visible; updateAxesVisibility(); }

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

  void clearField();

  QRectF fieldBoundingRect();

signals:
  void drawingFinished();
  void mouseMoved(double x, double y);

private:
  void createAxes();
  void updateAxesVisibility();

  MouseMode mode = MouseMode::kSelection;
  bool drawing_now = false;
  bool show_axes_ = true;

  QGraphicsRectItem* selection = nullptr;
  QGraphicsLineItem* pass = nullptr;
  QGraphicsLineItem* x_axis = nullptr;
  QGraphicsLineItem* y_axis = nullptr;
  std::vector<QGraphicsLineItem*> ticks = {};
  std::vector<QGraphicsTextItem*> labels = {};
};

#endif // QCUSTOMGRAPHICSSCENE_H
