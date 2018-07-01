#ifndef TEST_H
#define TEST_H

#include <cassert>
#include "graph.h"
#include "heuristics.h"
#include "algorithms.h"
#include "stats.h"

const int NUM_TEST_PROBLEMS = 10000;

int test_path_costs() {
  Stats stats_fringe("Fringe search"),
    stats_astar_heap("A* with a heap"),
    stats_astar_basic("A* (basic)");
  Graph graph;
  graph.load_ascii_map("../maps/example.map", EDGES_OCTILE);

  for (int ii = 0; ii < NUM_TEST_PROBLEMS; ++ ii) {
    Node *ss = 0, *gg = 0;
    while (ss == gg) {
      ss = graph.random_node();
      gg = graph.random_node();
    }
    fringe_search(graph, ss, gg, stats_fringe, &octile_heuristic);
    astar_basic(graph, ss, gg, stats_astar_basic, &octile_heuristic);
    astar_heap(graph, ss, gg, stats_astar_heap, &octile_heuristic);
  }

  // Ensure optimal paths found by all algorithms have the same cost:
  size_t expected_path_cost = stats_astar_basic.path_cost;
  assert(stats_fringe.path_cost == expected_path_cost);
  assert(stats_astar_heap.path_cost == expected_path_cost);

  return 0;
}

#endif // TEST_H
