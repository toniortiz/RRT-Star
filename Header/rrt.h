#ifndef RRT_H
#define RRT_H

#include "model.h"
#include <LEDA/graph/graph.h>

using leda::GRAPH;
using leda::node_map;
using leda::node;

class RRT {
public:
  double m_goal_dist;
  State m_best_state;
  GRAPH<vector, vector> m_tree;
  Model *m_model;
  node_map<int> m_node_depth;

public:
  RRT(Model *t_model);
  ~RRT();

public:
  Control best_control(const State &s1, const State &s2, State &nx_best,
                       bool &success);
  node nearest_neighbor(const State &x_rand);
  bool extend(const State &x_rand, node &n_new);
  bool plan(double g_dist, list<node> &path);
  list<node> calculate_path(const node &n);

  void neighborhood(const node &q, const double& radius, list<node> &nodes);
};

#endif // RRT_H
