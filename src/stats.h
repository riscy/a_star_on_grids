#ifndef STATS_H
#define STATS_H
#include <iostream>
using namespace std;

/// Simple structure for collecting pathfinding stats.
class Stats {
 public:
  string label;                 // an identifying label
  size_t num_problems;          // number of problems run
  size_t nodes_expanded;        // nodes expanded to make path
  size_t path_length;           // number of nodes on path
  size_t open_list_size;        // size of open list at termination
  double path_cost;             // cumulative edge cost on path
  clock_t start_time;           // for timing

  Stats(string label = "Stats") {
    this->label = label;
    renew();
  }

  void renew() {
    num_problems = 0;
    nodes_expanded = 0;
    path_length = 0;
    open_list_size = 0;
    path_cost = 0;
    start_time = clock();
  }

  inline double total_time() {
    return (clock() - start_time) / (double) CLOCKS_PER_SEC;
  }

  void print() {
    if (this->label.size())
      cout << this->label << ":" << endl;
    cout << " Total problems: " << num_problems << endl;
    cout << " Mean nodes expanded: " << nodes_expanded / num_problems << endl;
    cout << " Mean path length: " << path_length / num_problems << endl;
    cout << " Mean path cost: " << path_cost / num_problems << endl;
    cout << " Mean open list size: " << open_list_size / num_problems << endl;
    cout << " Total time (sec): " << total_time() << endl;
  }
};

#endif // STATS_H
