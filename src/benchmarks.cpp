#include <fstream>
using namespace std;
#include "benchmarks.h"
#include "graph.h"
#include "heuristics.h"
#include "algorithms.h"
#include "stats.h"

const int RANDOM_SEED = 10;

/// Test a random set of problems
void benchmark_all_algorithms(Graph & graph, int num_problems,
                              unsigned int (*heuristic)(Node*, Node*),
                              bool print_stats) {
  Stats stats_lrta_basic("LRTA* (suboptimal)");
  srand(RANDOM_SEED);
  for (int ii = 0; ii < num_problems; ++ ii) {
    Node *ss = 0, *gg = 0;
    while (ss == gg) {
      ss = graph.random_node();
      gg = graph.random_node();
    }
    lrta_basic(graph, ss, gg, stats_lrta_basic, heuristic);
  }
  if (print_stats)
    stats_lrta_basic.print();

  Stats stats_astar_basic("Basic A*");
  srand(RANDOM_SEED);
  for (int ii = 0; ii < num_problems; ++ ii) {
    Node *ss = 0, *gg = 0;
    while (ss == gg) {
      ss = graph.random_node();
      gg = graph.random_node();
    }
    astar_basic(graph, ss, gg, stats_astar_basic, heuristic);
  }
  if (print_stats)
    stats_astar_basic.print();

  Stats stats_fringe_search("Fringe search");
  srand(RANDOM_SEED);
  for (int ii = 0; ii < num_problems; ++ ii) {
    Node *ss = 0, *gg = 0;
    while (ss == gg) {
      ss = graph.random_node();
      gg = graph.random_node();
    }
    fringe_search(graph, ss, gg, stats_fringe_search, heuristic);
  }
  if (print_stats)
    stats_fringe_search.print();

  Stats stats_astar_heap("A* with a heap and tiebreaking on larger g");
  srand(RANDOM_SEED);
  for (int ii = 0; ii < num_problems; ++ ii) {
    Node *ss = 0, *gg = 0;
    while (ss == gg) {
      ss = graph.random_node();
      gg = graph.random_node();
    }
    astar_heap(graph, ss, gg, stats_astar_heap, heuristic);
  }
  if (print_stats)
    stats_astar_heap.print();
}

void benchmark_grid_costs() {
  size_t num_problems = 100000;

  Graph graph;
  graph.load_ascii_map("../maps/example.map", EDGES_OCTILE);
  benchmark_all_algorithms(graph, 100, &octile_heuristic); // warm the cache

  int test_costs[4][2] = {{70, 99}, {2, 3}, {50, 99}, {1, 1}};
  for (size_t ii = 0; ii < 4; ++ ii) {
    cout << endl << "Diagonal: " << test_costs[ii][0]
         << "/Cardinal: " << test_costs[ii][1] << endl;
    grid_costs(test_costs[ii][0], test_costs[ii][1]);
    benchmark_all_algorithms(graph, num_problems, &octile_heuristic, true);
  }

}
