#ifndef ALGORITHMS_H
#define ALGORITHMS_H
#include "stats.h"
#include "graph.h"

/// A-star with no optimizations, not even sorting of the open list.
/// Additionally contains some validations on the result.
void astar_basic(Graph & graph, Node* ss, Node* gg, Stats & stats,
                 unsigned int (*h)(Node* n1, Node* n2));

/// A* with a binary heap.
void astar_heap(Graph & graph, Node* ss, Node* gg, Stats & stats,
                unsigned int (*h)(Node* n1, Node* n2));

/// Fringe search (Bjornsson, Enzenberger, Holte, and Schaeffer '05).
void fringe_search(Graph & graph, Node* ss, Node* gg, Stats & stats,
                   unsigned int (*h)(Node* n1, Node* n2));

/// Basic learning real-time search
void lrta_basic(Graph & graph, Node* ss, Node* gg, Stats & stats,
                unsigned int (*h)(Node* n1, Node* n2));

#endif // ALGORITHMS_H
