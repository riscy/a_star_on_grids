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

  // pathfinding variables
  int g, f;                           // recorded g and f costs
  Node * whence;                      // for reconstructing paths
  list<Node*>::iterator fringe_index; // location in the fringe (fringe search)
  int heap_index;                     // location in the heap (A* with a heap)
  short closed_id;                    // problem on which this node is closed
  bool open;                          // whether this node is on an open list
  char glyph;                         // useful for displaying an ascii map

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
  unsigned short width, height;
  unsigned int (*cost)(Node*, Node*);

  vector<Node*> graph_view;
  vector<Node*> grid_view;       // contains nulls

  inline size_t size() { return graph_view.size(); }
  void print_stats();

  inline Node * node_at(int x, int y) { return grid_view[y * width + x]; }
  inline Node * random_node() { return graph_view[rand() % graph_view.size()]; }

  void load_ascii_map(string filename, EdgeType edge_type = EDGES_DEFAULT, bool corner_cut = false, bool verbose = false);
  void load_empty_map(int dim1, int dim2, EdgeType edge_type = EDGES_DEFAULT);

  void display_ascii_map();
  void display_ascii_path(Node*, Node*);

  size_t add_octile_edges(bool corner_cut = false);
  size_t add_quartile_edges();
  void remove_edge(Node*, Node*);
};

#endif // GRIDWORLD_H
