#ifndef BENCHMARKS_H
#define BENCHMARKS_H
#include "graph.h"
#include "heuristics.h"

void benchmark_all_algorithms(Graph & g, int num_problems,
                              unsigned int (*h)(Node*, Node*), bool print_stats = false);
void benchmark_grid_costs();

#endif // BENCHMARKS_H
