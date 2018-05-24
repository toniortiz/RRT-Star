#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "car.h"
#include "model.h"
#include "obstacle.h"
#include "rrt.h"
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QList>
#include <QWidget>

using leda::edge;

class Simulator : public QGraphicsView {
public:
  Simulator(QWidget *parent = 0);
  ~Simulator();
  QGraphicsScene *scene;
  Car *car;
  Model *model;

  QList<QRect> obst;
  QList<Obstacle *> buildings;

private:
  void buildCity();
  void drawStartPos();
  void drawEndPos();
  void drawRRT();
  void drawPath();
  double rad2deg(double rad);
};

#endif // SIMULATOR_H
