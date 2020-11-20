#ifndef QCUSTOMGRAPHICSSCENE_H
#define QCUSTOMGRAPHICSSCENE_H

#include <QGraphicsScene>
#include <QGraphicsRectItem>

class QCustomGraphicsScene : public QGraphicsScene {
public:
  explicit QCustomGraphicsScene(QWidget*) {};
  bool isSelectingNow() { return selecting_now; }
  void selectingOn() { selecting_now = true; }
  void selectingOff() { selecting_now = false; }
  void releaseSelection() { if (selection) { removeItem(selection); delete selection; selection = nullptr; update();} }
  const QRectF getSelection() {
    if (selection)
      return selection->rect();
    else
      return QRectF();
  }

  void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
  void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;

private:
  bool selecting_now = false;
  QGraphicsRectItem* selection = nullptr;
};

#endif // QCUSTOMGRAPHICSSCENE_H
