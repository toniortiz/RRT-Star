#ifndef CAR_H
#define CAR_H

#include <LEDA/core/list.h>
#include <LEDA/graph/graph.h>
#include <QGraphicsPixmapItem>
#include <QKeyEvent>
#include <QObject>

using leda::list_item;

class Car : public QObject, public QGraphicsPixmapItem {
  Q_OBJECT

public:
  Car(QGraphicsItem *parent = 0);
  void animation();
  double rad2deg(double rad);
  double deg2rad(double deg);

  void keyPressEvent(QKeyEvent *event);

  QTimer *timer;
  list_item it;

public slots:
  void state(qreal x, qreal y, qreal rot);
  void move();
};

#endif // CAR_H
