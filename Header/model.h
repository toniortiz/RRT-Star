#ifndef MODEL_H
#define MODEL_H

#include "config.h"
#include <LEDA/core/list.h>
#include <LEDA/core/random.h>
#include <LEDA/core/string.h>
#include <LEDA/geo/polygon.h>
#include <LEDA/numbers/vector.h>
#include <QList>
#include <QRect>

using leda::vector;
using leda::list;
using leda::random_source;
using leda::string;
using leda::polygon;

struct State {
  double x;
  double y;
  double theta;

  State() {}

  State(const double t_x, const double t_y, const double t_theta)
      : x(t_x), y(t_y), theta(t_theta) {}

  State(const vector &q) {
    x = q[0];
    y = q[1];
    theta = q[2];
  }

  vector as_vector() const {
    vector q(3);
    q[0] = x;
    q[1] = y;
    q[2] = theta;

    return q;
  }

  void fill(const vector &q) {
    x = q[0];
    y = q[1];
    theta = q[2];
  }

  bool operator==(const State &w) const {
    return ((x == w.x) && (y == w.y) && (theta == w.theta));
  }

  bool operator!=(const State &w) const { return !(*this == w); }
};

struct Control {
  double v;
  double steering;

  vector as_vector() const {
    vector q(2);
    q[0] = v;
    q[1] = steering;

    return q;
  }
};

struct Derivative {
  double dx;
  double dy;
  double dtheta;
};

class Model {
public:
  const State initial_state = {START_STATE_X, START_STATE_Y,
                               START_STATE_ORIENT};
  const State goal_state = {GOAL_STATE_X, GOAL_STATE_Y, GOAL_STATE_ORIENT};
  const State lower_state = {0.0, 0.0, 0.0};
  const State upper_state = {WORLD_WIDTH, WORLD_HEIGHT, 2 * M_PI};
  const int state_dim = 3;
  const int input_dim = 2;
  random_source m_random;

private:
  list<Control> m_inputs;
  list<polygon> m_obst;
  list<polygon> m_car;

public:
  Model(QList<QRect> &t_obst);
  ~Model();

public:
  State random_state();
  State integrate(const State &state, const Control &u, const double dt);
  list<Control> get_inputs() const;
  bool collision_free(const State &state);
  list<polygon> transform_car(const State &state);
  double distance_comp(const State &state);
  double metric(const State &s1, const State &s2);

private:
  State Runge_Kutta_4(const State &state, const Control &u, const double &dt);
  Derivative kinematic_model(const State &initial, const Control &control,
                             const double &dt, const Derivative &d);
  void constraints();
};

#endif // MODEL_H
