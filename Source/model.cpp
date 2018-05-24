#include "model.h"
#include <LEDA/geo/point.h>
#include <LEDA/geo/segment.h>
#include <QDebug>

using leda::segment;
using leda::point;
using leda::list_item;

#ifndef INFINITY
#define INFINITY 1.0e40
#endif
#ifndef sqr
#define sqr(x) ((x) * (x))
#endif
#ifndef min
#define min(x, y) ((x < y) ? x : y)
#endif

Model::Model(QList<QRect> &t_obst) {

  if (!collision_free(goal_state)) {
    qDebug() << "Goal state is in collision";
    exit(EXIT_FAILURE);
  }

  if (!collision_free(initial_state)) {
    qDebug() << "Initial state is in collision";
    exit(EXIT_FAILURE);
  }

  for (int i = 0; i < t_obst.size(); i++) {
    list<segment> ls;

    double x1 = t_obst[i].x();
    double y1 = t_obst[i].y();
    double w = t_obst[i].width();
    double h = t_obst[i].height();

    point p1(x1, y1);
    point p2(x1 + w, y1);
    point p3(x1 + w, y1 + h);
    point p4(x1, y1 + h);

    ls.push_back(segment(p1, p2));
    ls.push_back(segment(p2, p3));
    ls.push_back(segment(p3, p4));
    ls.push_back(segment(p4, p1));

    m_obst.push_back(polygon(ls));
  }

  constraints();
}

Model::~Model() {}

State Model::integrate(const State &state, const Control &u, const double dt) {
  State new_state;

  new_state = Runge_Kutta_4(state, u, dt);

  if (new_state.theta > 2.0 * M_PI)
    new_state.theta -= 2.0 * M_PI;

  if (new_state.theta < 0.0)
    new_state.theta += 2.0 * M_PI;

  return new_state;
}

State Model::Runge_Kutta_4(const State &state, const Control &u,
                           const double &dt) {
  Derivative k1, k2, k3, k4;

  k1 = kinematic_model(state, u, 0.0, Derivative());
  k2 = kinematic_model(state, u, dt * 0.5, k1);
  k3 = kinematic_model(state, u, dt * 0.5, k2);
  k4 = kinematic_model(state, u, dt, k3);

  double dxdt = 1.0f / 6.0f * (k1.dx + 2.0f * (k2.dx + k3.dx) + k4.dx);
  double dydt = 1.0f / 6.0f * (k1.dy + 2.0f * (k2.dy + k3.dy) + k4.dy);
  double dthetadt =
      1.0f / 6.0f * (k1.dtheta + 2.0f * (k2.dtheta + k3.dtheta) + k4.dtheta);

  State out_state;
  out_state.x = state.x + dxdt * dt;
  out_state.y = state.y + dydt * dt;
  out_state.theta = state.theta + dthetadt * dt;

  return out_state;
}

Derivative Model::kinematic_model(const State &initial, const Control &u,
                                  const double &dt, const Derivative &d) {
  State state;

  state.x = initial.x + d.dx * dt;
  state.y = initial.y + d.dy * dt;
  state.theta = initial.theta + d.dtheta * dt;

  Derivative D;
  D.dx = CAR_SPEED * u.v * cos(state.theta);
  D.dy = CAR_SPEED * u.v * sin(state.theta);
  D.dtheta = CAR_SPEED * u.v * tan(u.steering) / CAR_LENGTH;

  return D;
}

void Model::constraints() {
  // the car can turn left, turn rigth, go forward and reverse
  double alpha;
  for (alpha = -MAX_STEERING_ANGLE; alpha <= MAX_STEERING_ANGLE;
       alpha += 2.0 * MAX_STEERING_ANGLE / 10.0) {
    Control forward = {1.0, alpha};
    Control reverse = {-1.0, alpha};
    m_inputs.push_back(forward);
    m_inputs.push_back(reverse);
  }

  list<segment> ls;
  ls.push_back(segment(point(8, 8), point(8, -8)));
  ls.push_back(segment(point(8, -8), point(-8, -8)));
  ls.push_back(segment(point(-8, -8), point(-8, 8)));
  ls.push_back(segment(point(-8, 8), point(8, 8)));
  m_car.push_back(polygon(ls));
}

State Model::random_state() {
  double r;
  State rnd_state;

  rnd_state = lower_state;

  m_random >> r;
  rnd_state.x += r * (upper_state.x - lower_state.x);

  m_random >> r;
  rnd_state.y += r * (upper_state.y - lower_state.y);

  m_random >> r;
  rnd_state.theta += r * (upper_state.theta - lower_state.theta);

  return rnd_state;
}

list<Control> Model::get_inputs() const { return m_inputs; }

bool Model::collision_free(const State &state) {
  polygon op, rp;
  segment seg, seg2;

  list<polygon> t_car = transform_car(state);

  forall(rp, t_car) {
    forall(op, m_obst) {
      forall_segments(seg, rp) {
        forall_segments(seg2, op) {
          if (seg.intersection(seg2)) {
            return false;
          }
        }
      }
    }
  }

  // Check for containment
  forall(rp, t_car) {
    forall(op, m_obst) {
      if (op.contains(rp.vertices().head()))
        return false;
    }
  }

  return true;
}

list<polygon> Model::transform_car(const State &state) {
  list<polygon> t_car;
  polygon p, tp;

  forall(p, m_car) {
    tp = p.rotate(state.theta).translate(state.x, state.y);
    t_car.push_front(tp);
  }

  return t_car;
}

double Model::distance_comp(const State &state) {
  polygon rp, op;
  double d_min, d;
  point rpt;

  d_min = INFINITY;

  list<polygon> t_car = transform_car(state);

  if (collision_free(state)) {
    forall(rp, t_car) {
      forall_vertices(rpt, rp) {
        forall(op, m_obst) {
          d = op.distance(rpt);

          if (d < d_min)
            d_min = d;
        }
      }
    }
  } else
    d_min = -1.0;

  return d_min;
}

double Model::metric(const State &s1, const State &s2) {
  double fd = fabs(s1.theta - s2.theta);
  double dtheta = min(fd, 2.0 * M_PI - fd);

  return sqrt(sqr(s1.x - s2.x) + sqr(s1.y - s2.y) + sqr(50.0 / M_PI * dtheta));
}
