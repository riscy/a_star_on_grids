#include <list>
#include <vector>
using namespace std;
#include <climits>
#include "algorithms.h"
#include "graph.h"
#include "heuristics.h"
#include "node_heap.h"

// These algorithms 'close' nodes by flagging them with the id of the
// current problem being solved.  This saves us unclosing every node
// after solving each path.
short problem_id = 1;

inline void init_new_problem(Graph & graph, Stats & stats) {
  ++ stats.num_problems;
  ++ problem_id;
  // check integer overflow; problem_ids are no longer unique, so reset.
  if (problem_id == 0) {
    for (auto& node: graph.graph_view)
      node->closed_id = 0;
    problem_id = 1;
  }
}

inline void reconstruct_path(Node* start, Node* current,
                             unsigned int (*cost)(Node*, Node*),
                             Stats & stats) {
  while (current != start) {
    stats.path_cost += cost(current->whence, current);
    ++ stats.path_length;
    current = current->whence;
  }
}

/// A-star with no optimizations, not even sorting the open list.
/// Additionally contains some validations on the result.
void astar_basic(Graph & graph, Node* start, Node* goal, Stats & stats,
                 unsigned int (*h)(Node* n1, Node* n2)) {
  init_new_problem(graph, stats);
  static vector<Node*> open_list;
  start->open = true;
  start->relax(0, h(start, goal), NULL);
  open_list.push_back(start);

  while (!open_list.empty()) {
    int fmin = INT_MAX;
    auto best_on_open_list = open_list.begin();
    // Pop the best node off the open_list via linear scan
    for (auto node = open_list.begin(); node != open_list.end(); ++ node) {
      if ((*node)->f < fmin) {
        fmin = (*node)->f;
        best_on_open_list = node;
      }
    }
    Node* expand_me = *best_on_open_list;
    if (expand_me == goal)
      break;
    expand_me->expand(problem_id);
    ++ stats.nodes_expanded;
    // remove it by overwriting it with the back() node
    *best_on_open_list = open_list.back();
    open_list.pop_back();

    // Add each neighbor
    for (auto add_me: expand_me->neighbors_out) {
      if (add_me->closed(problem_id))
        continue;
      const int g = expand_me->g + graph.cost(expand_me, add_me);
      if (!add_me->open) {       // If it's not open, open it
        add_me->open = true;
        add_me->relax(g, h(add_me, goal), expand_me);
        open_list.push_back(add_me);
      }
      else if (add_me->g > g) {  // If it is open, relax it
        add_me->relax(g, h(add_me, goal), expand_me);
      }
    }
  }

  // Stats collection & cleanup
  stats.open_list_size += open_list.size();
  reconstruct_path(start, goal, graph.cost, stats);
  for (auto node: open_list)
    node->open = false;
  open_list.clear();
}

/// A* with a binary heap.
void astar_heap(Graph & graph, Node* start, Node* goal, Stats & stats,
                unsigned int (*h)(Node* n1, Node* n2)) {
  init_new_problem(graph, stats);
  static vector<Node*> open_list;
  start->open = true;
  start->relax(0, h(start, goal), NULL);
  node_heap::push(open_list, start);

  while (!open_list.empty()) {
    // Pop the best node off the open_list (+ goal check)
    Node* expand_me = open_list.front();
    if (expand_me == goal)
      break;

    ++ stats.nodes_expanded;
    expand_me->expand(problem_id);
    node_heap::pop(open_list);

    // Add each neighbor
    for (auto& add_me: expand_me->neighbors_out) {
      if (add_me->closed(problem_id))
        continue;
      const int g = expand_me->g + graph.cost(expand_me, add_me);
      if (!add_me->open) {       // If it's not open, open it
        add_me->open = true;
        add_me->relax(g, h(add_me, goal), expand_me);
        node_heap::push(open_list, add_me);
      }
      else if (g < add_me->g) {  // If it is open, relax it
        add_me->relax(g, add_me->f - add_me->g, expand_me);
        node_heap::repair(open_list, add_me->heap_index);
      }
    }
  }

  // Stats collection & cleanup
  stats.open_list_size += open_list.size();
  reconstruct_path(start, goal, graph.cost, stats);
  for (auto& node: open_list)
    node->open = false;
  open_list.clear();
}

/// Fringe search (Bjornsson, Enzenberger, Holte, and Schaeffer '05).
// Like other algorithms in the A* family, fringe search expands nodes one ply
// of f values at a time.  Fringe search does this in a depth-first fashion,
// favoring the expansion of recently touched nodes, which can be accommodated
// by inserting entries into a linked list.
//
// Without aggressive compiler optimizations, Fringe Search beats A* handily.
void fringe_search(Graph & graph, Node* start, Node* goal, Stats & stats,
                   unsigned int (*h)(Node* n1, Node* n2)) {
  init_new_problem(graph, stats);
  static list<Node*> Fringe;
  Fringe.push_back(start);
  start->open = true;
  start->relax(0, h(start, goal), NULL);
  start->fringe_index = Fringe.begin();
  bool found = false;
  int f_limit = start->f;

  while (!found && !Fringe.empty()) {
    int next_f_limit = INT_MAX;
    for (auto ff = Fringe.begin(); ff != Fringe.end();) {
      Node* expand_me = *ff;
      // is this node outside the current depth?
      if (expand_me->f > f_limit) {
        if (expand_me->f < next_f_limit)
          next_f_limit = expand_me->f; // track smallest next depth
        ++ ff;
        continue; // skip this one (for now)
      }
      if (expand_me == goal) {
        found = true;
        break;
      }

      ++ stats.nodes_expanded;
      expand_me->expand(problem_id);

      // Relax the neighbors and put them on the fringe AFTER `expand_me'
      for (auto& add_me: expand_me->neighbors_out) {
        if (add_me->closed(problem_id))
          continue;
        const int g = expand_me->g + graph.cost(expand_me, add_me);

        if (!add_me->open) {
          add_me->open = true;
          add_me->relax(g, h(add_me, goal), expand_me);
          auto insertion_point = next(ff);
          add_me->fringe_index = Fringe.insert(insertion_point, add_me);
        }
        else if (g < add_me->g) {
          add_me->relax(g, add_me->f - add_me->g, expand_me);
          auto insertion_point = next(ff);
          if (*insertion_point != add_me) {
            Fringe.erase(add_me->fringe_index);
            add_me->fringe_index = Fringe.insert(insertion_point, add_me);
          }
        }
      }
      ff = Fringe.erase(ff);
    }
    // Increase the depth and scan the fringe again
    f_limit = next_f_limit;
  }

  // Stats collection & cleanup
  stats.open_list_size += Fringe.size();
  reconstruct_path(start, goal, graph.cost, stats);
  for (auto& node: Fringe)
    node->open = false;
  Fringe.clear();
}

/// Basic learning real-time search
void lrta_basic(Graph & graph, Node* start, Node* goal, Stats & stats,
                unsigned int (*h)(Node* n1, Node* n2)) {
  init_new_problem(graph, stats);
  while (start != goal) {
    Node* best_neighbor = 0;
    unsigned int best_f = INT_MAX;
    stats.nodes_expanded += 1;
    for (auto& neighb: start->neighbors_out) {
      // set default heuristic value if it isn't set
      if (!(neighb->closed(problem_id))) {
        neighb->expand(problem_id);
        neighb->f = h(neighb, goal);
      }
      unsigned int f = graph.cost(start, neighb) + neighb->f;
      if (!best_neighbor || f < best_f) {
        best_neighbor = neighb;
        best_f = f;
      }
      else if (f == best_f && stats.nodes_expanded % 2)
        best_neighbor = neighb;
    }
    start->f = best_f; // learning update
    stats.path_cost += graph.cost(start, best_neighbor);
    start = best_neighbor;
  }
  stats.path_length = stats.nodes_expanded;
}
