#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <QGraphicsItem>
#include <QGraphicsPixmapItem>
#include <QObject>

class Obstacle : public QObject, public QGraphicsPixmapItem {
  Q_OBJECT

public:
  Obstacle(QGraphicsItem *parent = 0);
  Obstacle(QString path, qreal x, qreal y);
};

#endif // OBSTACLE_H
