#include "rrt.h"
#include <QDebug>
#include <memory>

RRT::RRT(Model *t_model) {
  m_model = t_model;
  m_goal_dist = m_model->metric(m_model->initial_state, m_model->goal_state);
  m_best_state = m_model->initial_state;

  m_node_depth.init(m_tree);
}

RRT::~RRT() { delete m_model; }

Control RRT::best_control(const State &x_near, const State &x, State &nx_best,
                          bool &success) {
  Control u, u_best;
  State x_new;
  double d, d_min;
  success = false;
  d_min = m_model->metric(x_near, x);

  list<Control> controls = m_model->get_inputs();

  forall(u, controls) {
    x_new = m_model->integrate(x_near, u, DT);
    d = m_model->metric(x_new, x);

    if ((d < d_min) && (m_model->collision_free(x_new)) && (x_near != x_new)) {
      d_min = d;
      u_best = u;
      nx_best = x_new;
      success = true;
    }
  }

  return u_best;
}

node RRT::nearest_neighbor(const State &x_rand) {
  double d, d_min;
  node n, n_best;

  n_best = m_tree.choose_node();
  d_min = INFINITY;
  d = 0.0;

  forall_nodes(n, m_tree) {

    d = m_model->metric(State(m_tree[n]), x_rand) +
        RRTSTAR_FACTOR * CAR_SPEED * DT * m_node_depth[n];

    if (d < d_min) {
      d_min = d;
      n_best = n;
    }
  }

  return n_best;
}

bool RRT::extend(const State &x_rand, node &n_new) {
  node n_near;
  Control u_new;
  State x_new;
  bool success = false;

  n_near = nearest_neighbor(x_rand);

  // x_new gets next state
  u_new = best_control(State(m_tree.inf(n_near)), x_rand, x_new, success);

  if (success) {
    n_new = m_tree.new_node(x_new.as_vector());
    m_node_depth[n_new] = m_node_depth[n_near] + 1;
    m_tree.new_edge(n_near, n_new, u_new.as_vector());
  }

  return success;
}

bool RRT::plan(double g_dist, list<node> &path) {
  int i = 0;
  double d;
  node n, n_new, n_goal;

  if (m_tree.number_of_nodes() == 0) {
    n_goal = m_tree.new_node(m_model->initial_state.as_vector());
  }

  n = nearest_neighbor(m_model->goal_state);
  m_goal_dist = m_model->metric(State(m_tree.inf(n)), m_model->goal_state);

  while (m_goal_dist > g_dist) {
    if (extend(m_model->random_state(), n_new)) {

      d = m_model->metric(State(m_tree.inf(n_new)), m_model->goal_state);

      if (d < m_goal_dist) {
        m_goal_dist = d;
        m_best_state.fill(m_tree.inf(n_new));
        n_goal = n_new;

        qDebug() << "it: " << i << "Nodes: " << m_tree.number_of_nodes()
                 << "  GoalDist: " << m_goal_dist;
      }
    }
    i++;
  }

  if (m_goal_dist < g_dist) {
    qDebug() << "Successful Path Found\n";

    node n_goal;
    n_goal = m_tree.new_node(m_model->goal_state.as_vector());
    m_node_depth[n_goal] = m_node_depth[n_goal] + 1;
    m_tree.new_edge(n_new, n_goal, vector(1, 0.4));

    path = calculate_path(n_goal);
    return true;
  } else {
    return false;
  }
}

list<node> RRT::calculate_path(const node &n) {
  list<node> path;
  node ni = n;
  int i = 0;

  path.clear();
  path.push(n);

  while ((ni != m_tree.first_node()) && (i < m_tree.number_of_nodes())) {
    path.push(m_tree.source(m_tree.first_in_edge(ni)));
    ni = m_tree.source(m_tree.first_in_edge(ni));
    i++;
  }

  return path;
}

void RRT::neighborhood(const node &q, const double &radius, list<node> &nodes) {
  node n;
  double d = 0.0;

  forall_nodes(n, m_tree) {
    d = m_model->metric(State(m_tree[n]), State(m_tree[q]));

    if (d < radius) {
      nodes.push_back(n);
    }
  }
}
