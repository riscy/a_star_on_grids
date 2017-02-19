#include <list>
#include <vector>
using namespace std;
#include <climits>
#include "algorithms.h"
#include "graph.h"
#include "heuristics.h"
#include "stats.h"
#include "node_heap.h"

// These algorithms 'close' nodes by flagging them with the id of the
// current problem being solved.  This saves us unclosing every node
// after solving each path.
int problem_id = 1;

inline void init_new_problem(Graph & graph, Stats & stats) {
  ++ stats.num_problems;
  ++ problem_id;
  // check integer overflow; problem_ids are no longer unique, so reset.
  if (problem_id == 0) {
    for (size_t ii = 0; ii < graph.graph_view.size(); ++ ii)
      graph.graph_view[ii]->closed_id = 0;
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
  unsigned int (*cost)(Node*, Node*) = graph.cost;
  init_new_problem(graph, stats);
  typedef vector<Node*>::iterator iter;
  static vector<Node*> open_list;
  start->open = true;
  start->relax(0, h(start, goal), NULL);
  open_list.push_back(start);

  // Pop the best node off the open_list
  while (!open_list.empty()) {
    int g = 0, fmin = INT_MAX;
    iter ii_best;
    for (iter ii = open_list.begin(); ii != open_list.end(); ++ ii) {
      Node * nd = *ii;
      if (nd->f > fmin)
        continue;
      if (nd->f <= fmin || nd->g > g) {
        fmin = nd->f;
        g = nd->g;
        ii_best = ii;
      }
    }
    Node *expand_me = *ii_best;
    expand_me->open = false;

    // remove it by replacing it with the back() node
    *ii_best = open_list.back();
    open_list.pop_back();

    // expand and close the best
    if (expand_me == gg)
      break;
    expand_me->expand(problem_id);
    ++ stats.nodes_expanded;

    // Add each neighbor
    for (vector_iter ii = expand_me->neighbors_out.begin();
         ii != expand_me->neighbors_out.end(); ++ ii) {
      Node* add_me = *ii;
      if (add_me->closed(problem_id))
        continue;
      int g = expand_me->g + cost(expand_me, add_me);
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
  for (vector_iter ii = open_list.begin(); ii != open_list.end(); ++ ii)
    (*ii)->open = false;
  open_list.clear();
}

/// A* with a binary heap.
void astar_heap(Graph & graph, Node* start, Node* goal, Stats & stats,
                unsigned int (*h)(Node* n1, Node* n2)) {
  unsigned int (*cost)(Node*, Node*) = graph.cost;
  init_new_problem(graph, stats);
  typedef vector<Node*>::iterator iter;
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
    for (iter ii = expand_me->neighbors_out.begin(), end = expand_me->neighbors_out.end();
         ii != end; ++ ii) {
      Node* add_me = *ii;
      if (add_me->closed(problem_id))
        continue;

      // NOTE: you can implement greedy best-first search by setting g = 0 here,
      // or weighted A* by scaling h up by some scalar > 1.
      const int g = expand_me->g + cost(expand_me, add_me);
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
  for (vector_iter ii = open_list.begin(); ii != open_list.end(); ++ ii)
    (*ii)->open = false;
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
  unsigned int (*cost)(Node*, Node*) = graph.cost;
  init_new_problem(graph, stats);
  static list<Node*> Fringe;
  typedef list<Node*>::iterator iter;
  Fringe.push_back(start);
  start->open = true;
  start->relax(0, h(start, goal), NULL);
  start->fringe_index = Fringe.begin();
  bool found = false;
  int flimit = start->f;

  do {
    int fnext = INT_MAX;
    iter ff = Fringe.begin();
    while (ff != Fringe.end()) {
      Node* expand_me = *ff;
      int f = expand_me->g + h(expand_me, gg);

      // Consider this node (is `expand_me' outside our depth?)
      if (f > flimit) {
        // keep track of smallest next depth
        if (f < fnext) {
          fnext = f;
        }
        ++ ff;
        continue; // skip this one (for now)
      }
      else if (expand_me == gg) {
        // is `expand_me' the goal and within our depth?
        found = true;
        break;
      }

      // Expand this node: relax its neighbors' g-values and
      // put them on the fringe directly AFTER `expand_me'
      ++ stats.nodes_expanded;
      for (vector<Node*>::iterator jj = expand_me->neighbors_out.begin();
           jj != expand_me->neighbors_out.end(); ++ jj) {
        Node* relax_me = *jj;
        int g = expand_me->g + cost(expand_me, relax_me);
        // jj is or has been on the fringe?
        if (relax_me->open || relax_me->closed == problem_id) {
          // is this a worse path?
          if (g >= relax_me->g)
            continue;
        }
        iter insertion_point = ff;
        ++ insertion_point;
        if (!relax_me->open) {
          // if not already open, insert jj after expand_me
          relax_me->fringe_index = Fringe.insert(insertion_point, relax_me);
          relax_me->open = true;
        }
        else if (*insertion_point != relax_me) {
          // if open, move it to right after expand_me
          Fringe.erase(relax_me->fringe_index);
          relax_me->fringe_index = Fringe.insert(insertion_point, relax_me);
        }
        relax_me->g = g;
        relax_me->whence = expand_me;
      }

      // Remove expand_me from Fringe
      expand_me->open = false;
      expand_me->closed = problem_id;
      expand_me->fringe_index = Fringe.end();
      ff = Fringe.erase(ff);
    }

    // Increase the depth and scan the fringe again
    flimit = fnext;
  } while (!found && !Fringe.empty());

  // Stats collection & cleanup
  stats.open_list_size += Fringe.size();
  reconstruct_path(start, goal, graph.cost, stats);
  for (list_iter ff = Fringe.begin(); ff != Fringe.end(); ++ ff)
    (*ff)->open = false;
  Fringe.clear();
}

/// Basic learning real-time search
void lrta_basic(Graph & graph, Node* start, Node* goal, Stats & stats,
                unsigned int (*h)(Node* n1, Node* n2)) {
  unsigned int (*cost)(Node*, Node*) = graph.cost;
  init_new_problem(graph, stats);
  while (start != goal) {
    Node* best_neighbor = 0;
    unsigned int best_f = INT_MAX;
    stats.nodes_expanded += 1;
    for (size_t ii = 0; ii < start->neighbors_out.size(); ++ ii) {
      Node* neighb = start->neighbors_out[ii];
      // set default heuristic value if it isn't set
      if (!(neighb->closed(problem_id))) {
        neighb->expand(problem_id);
        neighb->f = h(neighb, goal);
      }
      unsigned int f = cost(ss, neighb) + neighb->f;
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
