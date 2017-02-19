#ifndef GRIDWORLD_H
#define GRIDWORLD_H
#include <iostream>
#include <vector>
#include <list>
using namespace std;

#include <cstdlib>

enum EdgeType { EDGES_DEFAULT, EDGES_OCTILE, EDGES_QUARTILE };

class Node {
 public:
  vector<Node*> neighbors_out;
  vector<Node*> neighbors_in;
  int grid_x, grid_y;
  char glyph;                   // useful for displaying ascii map

  // pathfinding variables
  int g, f;                     // recorded g and f costs
  int heap_index;               // location in the heap
  int closed_id;
  bool open;
  Node * whence;                      // for reconstructing paths
  list<Node*>::iterator fringe_index; // used by fringe search

  Node();
  void expand(int problem_id);
  void relax(int g, int h, Node* whence);
  inline bool closed(int problem_id) { return this->closed_id == problem_id; }
  string to_str(bool verbose = false);
};

class Graph {
 public:
  Graph() { this->cost = 0; }
  ~Graph() { this->clear(); }
  void clear();

  EdgeType edge_type;
  int width, height;
  unsigned int (*cost)(Node*, Node*);

  vector<Node*> graph_view;
  vector<Node*> grid_view;       // contains nulls

  inline size_t size() { return graph_view.size(); }
  void print_stats();

  inline Node * node_at(int x, int y) { return grid_view[y * width + x]; }
  inline Node * random_node() { return graph_view[rand() % graph_view.size()]; }

  void load_ascii_map(string filename, EdgeType edge_type = EDGES_DEFAULT, bool corner_cut = false);
  void load_empty_map(int dim1, int dim2, EdgeType edge_type = EDGES_DEFAULT);

  void display_ascii_map();
  void display_ascii_path(Node*, Node*);

  size_t add_octile_edges(bool corner_cut = false);
  size_t add_quartile_edges();
  void remove_edge(Node*, Node*);
};

#endif // GRIDWORLD_H
