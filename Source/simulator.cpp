#include "simulator.h"
#include "config.h"
#include "global.h"

using leda::point;

Simulator::Simulator(QWidget *parent) {
  scene = new QGraphicsScene();
  scene->setSceneRect(0, 0, WORLD_WIDTH, WORLD_HEIGHT);

  setBackgroundBrush(QBrush(Qt::green));
  setScene(scene);
  setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setFixedSize(WORLD_WIDTH, WORLD_HEIGHT);

  buildCity();

  if (!GET_COORDINATES) {
    drawStartPos();
    drawEndPos();

    drawRRT();
    drawPath();

    car = new Car();
    car->state(this->x() + START_STATE_X, this->y() + START_STATE_Y,
               rad2deg(START_STATE_ORIENT));
    scene->addItem(car);

    car->animation();
  } else {
    car = new Car();
    car->state(20, 20, 0);
    car->setFlag(QGraphicsItem::ItemIsFocusable);
    car->setFocus();
    scene->addItem(car);
  }
  show();
}

Simulator::~Simulator() {
  delete car;
  delete model;
  qDeleteAll(buildings);
  buildings.clear();
}

void Simulator::buildCity() {
  buildings.push_back(new Obstacle(":/images/city-hall.png", 50, 50));
  buildings.push_back(
      new Obstacle(":/images/street-light.png", 50 + 90, 50 + 60));
  buildings.push_back(new Obstacle(":/images/vertical.png", 1, 1));
  buildings.push_back(new Obstacle(":/images/horizontal.png", 1, 1));
  obst.push_back(QRect(50, 50, 128, 128));

  buildings.push_back(new Obstacle(":/images/mosque.png", 228, 50));
  buildings.push_back(
      new Obstacle(":/images/street-light.png", 228 + 90, 50 + 60));
  buildings.push_back(new Obstacle(":/images/vertical.png", 178, 1));
  buildings.push_back(new Obstacle(":/images/horizontal.png", 178, 1));
  obst.push_back(QRect(228, 50, 128, 128));

  buildings.push_back(new Obstacle(":/images/building.png", 406, 50));
  buildings.push_back(
      new Obstacle(":/images/street-light.png", 406 + 90, 50 + 60));
  buildings.push_back(new Obstacle(":/images/vertical.png", 356, 1));
  buildings.push_back(new Obstacle(":/images/horizontal.png", 356, 1));
  obst.push_back(QRect(406, 50, 128, 128));

  buildings.push_back(new Obstacle(":/images/building2.png", 584, 50));
  buildings.push_back(
      new Obstacle(":/images/street-light.png", 584 + 90, 50 + 60));
  buildings.push_back(new Obstacle(":/images/vertical.png", 534, 1));
  buildings.push_back(new Obstacle(":/images/horizontal.png", 534, 1));
  obst.push_back(QRect(584, 50, 128, 128));

  ////////////////////////////////////////////////////////////////////////////

  buildings.push_back(new Obstacle(":/images/building3.png", 50, 228));
  buildings.push_back(
      new Obstacle(":/images/street-light.png", 50 + 90, 228 + 60));
  buildings.push_back(new Obstacle(":/images/vertical.png", 1, 178));
  buildings.push_back(new Obstacle(":/images/horizontal.png", 1, 178));
  obst.push_back(QRect(50, 228, 128, 128));

  buildings.push_back(new Obstacle(":/images/church.png", 228, 228));
  buildings.push_back(
      new Obstacle(":/images/street-light.png", 228 + 90, 228 + 60));
  buildings.push_back(new Obstacle(":/images/vertical.png", 178, 178));
  buildings.push_back(new Obstacle(":/images/horizontal.png", 178, 178));
  obst.push_back(QRect(228, 228, 128, 128));

  buildings.push_back(new Obstacle(":/images/hospital.png", 406, 228));
  buildings.push_back(
      new Obstacle(":/images/street-light.png", 406 + 90, 228 + 60));
  buildings.push_back(new Obstacle(":/images/vertical.png", 356, 178));
  buildings.push_back(new Obstacle(":/images/horizontal.png", 356, 178));
  obst.push_back(QRect(406, 228, 128, 128));

  buildings.push_back(new Obstacle(":/images/hotel.png", 584, 228));
  buildings.push_back(
      new Obstacle(":/images/street-light.png", 584 + 90, 228 + 60));
  buildings.push_back(new Obstacle(":/images/vertical.png", 534, 178));
  buildings.push_back(new Obstacle(":/images/horizontal.png", 534, 178));
  obst.push_back(QRect(584, 228, 128, 128));

  ////////////////////////////////////////////////////////////////////////////

  buildings.push_back(new Obstacle(":/images/shop.png", 50, 406));
  buildings.push_back(
      new Obstacle(":/images/street-light.png", 50 + 90, 406 + 60));
  buildings.push_back(new Obstacle(":/images/vertical.png", 1, 356));
  buildings.push_back(new Obstacle(":/images/horizontal.png", 1, 356));
  obst.push_back(QRect(50, 406, 128, 128));

  buildings.push_back(new Obstacle(":/images/fountain.png", 260, 430));
  buildings.push_back(new Obstacle(":/images/tree.png", 300, 470));
  buildings.push_back(new Obstacle(":/images/hydrant.png", 228, 406));
  buildings.push_back(new Obstacle(":/images/vertical.png", 178, 356));
  buildings.push_back(new Obstacle(":/images/horizontal.png", 178, 356));
  obst.push_back(QRect(228, 406, 128, 128));

  buildings.push_back(new Obstacle(":/images/skyscraper.png", 406, 406));
  buildings.push_back(
      new Obstacle(":/images/street-light.png", 406 + 90, 406 + 60));
  buildings.push_back(new Obstacle(":/images/vertical.png", 356, 356));
  buildings.push_back(new Obstacle(":/images/horizontal.png", 356, 356));
  obst.push_back(QRect(406, 406, 128, 128));

  buildings.push_back(new Obstacle(":/images/supermarket.png", 584, 406));
  buildings.push_back(new Obstacle(":/images/hydrant.png", 584, 406 + 95));
  buildings.push_back(new Obstacle(":/images/vertical.png", 534, 356));
  buildings.push_back(new Obstacle(":/images/horizontal.png", 534, 356));
  obst.push_back(QRect(584, 406, 128, 128));

  // top border
  obst.push_back(QRect(0, 0, WORLD_WIDTH, 2));

  // right border
  obst.push_back(QRect(0, 0, 2, WORLD_HEIGHT));

  // bottom border
  obst.push_back(QRect(0, WORLD_HEIGHT, WORLD_WIDTH, 2));

  // left border
  obst.push_back(QRect(WORLD_WIDTH, 0, 2, WORLD_HEIGHT));

  //  for (int i = 0; i < obst.size(); ++i)
  //    scene->addRect(obst[i]);

  for (int i = 0; i < buildings.size(); ++i)
    scene->addItem(buildings[i]);
}

void Simulator::drawStartPos() {
  scene->addEllipse(START_STATE_X - NODE_RADIUS, START_STATE_Y - NODE_RADIUS,
                    2 * NODE_RADIUS, 2 * NODE_RADIUS, QPen(Qt::white),
                    QBrush(Qt::red));
}

void Simulator::drawEndPos() {
  scene->addEllipse(GOAL_STATE_X - NODE_RADIUS, GOAL_STATE_Y - NODE_RADIUS,
                    2 * NODE_RADIUS, 2 * NODE_RADIUS, QPen(Qt::white),
                    QBrush(Qt::blue));
}

void Simulator::drawRRT() {
  model = new Model(obst);
  rrt = new RRT(model);

  rrt->plan(END_DIST_THRESHOLD, path);

  if (SHOW_RRT) {
    // draw rrt nodes
    edge e;
    vector q1, q2;

    forall_edges(e, rrt->m_tree) {
      q1 = rrt->m_tree.inf(rrt->m_tree.source(e));
      q2 = rrt->m_tree.inf(rrt->m_tree.target(e));

      QPointF p1(q1[0], q1[1]);
      QPointF p2(q2[0], q2[1]);
      QLineF line(p1, p2);
      scene->addLine(line, QPen(Qt::white));
      // scene->addEllipse(q2[0] - 1.5, q2[1] - 1.5, 4, 4, QPen(Qt::blue));
    }
  }
}

void Simulator::drawPath() {

  if (SHOW_PATH) {
    node n;
    polygon p;

    forall(n, path) {
      State st(rrt->m_tree.inf(n));

      //      list<polygon> trobot = rrt->m_model->transform_car(st);

      //      forall(p, trobot) {

      //        list<point> points = p.vertices();
      //        point p;
      //        QPolygonF q_poly;

      //        forall(p, points) { q_poly.append(QPointF(p.xcoord(),
      //        p.ycoord())); }

      //         scene->addPolygon(q_poly, QPen(Qt::magenta),
      //         QBrush(Qt::black));
      //      }
      scene->addEllipse(st.x, st.y, NODE_RADIUS, NODE_RADIUS,
                        QPen(Qt::magenta));
    }
  }
}

double Simulator::rad2deg(double rad) { return rad * (180.0 / M_PI); }
