#include "obstacle.h"

Obstacle::Obstacle(QGraphicsItem *parent) {}

Obstacle::Obstacle(QString path, qreal x, qreal y) {
  setPixmap(QPixmap(path));
  setPos(x, y);
}
