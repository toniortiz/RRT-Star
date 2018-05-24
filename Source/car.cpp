#include "car.h"
#include "config.h"
#include "global.h"
#include <QDebug>
#include <QTimer>

Car::Car(QGraphicsItem *parent) : QGraphicsPixmapItem(parent) {
  setPixmap(QPixmap(":/images/car.png"));

  setOffset(-13, -13);
  timer = new QTimer();
  connect(timer, SIGNAL(timeout()), this, SLOT(move()));
}

void Car::animation() {
  timer->start(50);
  it = path.first();
}

void Car::state(qreal x, qreal y, qreal rot) {
  setPos(x, y);
  setRotation(rot);
}

void Car::move() {
  leda::node n = path.contents(it);

  vector q = rrt->m_tree.inf(n);

  setPos(q[0], q[1]);
  setRotation(rad2deg(q[2]));

  it = path.succ(it);
  if (it == nullptr) {
    if (REBUILD)
      it = path.first();
    else
      timer->stop();
  }
}

double Car::rad2deg(double rad) { return rad * (180.0 / M_PI); }

double Car::deg2rad(double deg) {
  double rad = deg * (M_PI / 180.0);

  return rad;
}

void Car::keyPressEvent(QKeyEvent *event) {
  if (!GET_COORDINATES)
    return;

  const int inc = 3;

  if (event->key() == Qt::Key_Right) {
    if (pos().x() < WORLD_WIDTH) {
      setPos(x() + inc, y());
      setRotation(rotation());
    }
  }

  if (event->key() == Qt::Key_Left) {
    if (pos().x() > 0) {
      setPos(x() - inc, y());
      setRotation(rotation());
    }
  }

  if (event->key() == Qt::Key_Down) {
    if (pos().y() < WORLD_HEIGHT) {
      setPos(x(), pos().y() + inc);
      setRotation(rotation());
    }
  }

  if (event->key() == Qt::Key_Up) {
    if (pos().y() > 0) {
      setPos(x(), pos().y() - inc);
      setRotation(rotation());
    }
  }

  if (event->key() == Qt::Key_Space) {
    setPos(x(), y());
    setRotation(rotation() + inc);
  }

  qDebug() << "x: " << x() << " y: " << y() << " r: " << rotation();
}
