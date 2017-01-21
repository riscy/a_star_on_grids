#include <cassert>
#include "heuristics.h"
#include "graph.h"

int cardinal_cost = 2;
int diagonal_cost = 3;
int diagonal_minus_cardinal = diagonal_cost - cardinal_cost;
int two_cardinal_minus_diagonal = 2 * cardinal_cost - diagonal_cost;
int half_two_cardinal_minus_diagonal = cardinal_cost - diagonal_cost / 2;
int half_diagonal = diagonal_cost / 2;

// Costs.........................................................................

void grid_costs(int new_cardinal_cost, int new_diagonal_cost) {
  assert(new_cardinal_cost > 0 && new_diagonal_cost > 0);
  cardinal_cost = new_cardinal_cost;
  diagonal_cost = new_diagonal_cost;
  diagonal_minus_cardinal = diagonal_cost - cardinal_cost;
  two_cardinal_minus_diagonal = 2 * cardinal_cost - diagonal_cost;
  half_two_cardinal_minus_diagonal = cardinal_cost - diagonal_cost / 2;
  half_diagonal = diagonal_cost / 2;
}

unsigned int inf_norm_cost(Node*, Node*) {
  return 1;
}

unsigned int man_cost(Node* n1, Node* n2) {
  if (n1->grid_x == n2->grid_x || n1->grid_y == n2->grid_y)
    return 1;
  return 2;
}

unsigned int octile_cost(Node* n1, Node* n2) {
  if (n1->grid_x == n2->grid_x || n1->grid_y == n2->grid_y)
    return cardinal_cost;
  return diagonal_cost;
}

// Heuristics...................................................................

unsigned int zero_heuristic(Node*, Node*) {
  return 0;
}

unsigned int inf_heuristic(Node* n1, Node* n2) {
  unsigned int dx = abs(n1->grid_x - n2->grid_x);
  unsigned int dy = abs(n1->grid_y - n2->grid_y);
  if (dx > dy)
    return dx;
  return dy;
}

unsigned int man_heuristic(Node* n1, Node* n2) {
  return abs(n1->grid_x - n2->grid_x) + abs(n1->grid_y - n2->grid_y);
}

unsigned int octile_heuristic(Node* n1, Node* n2) {
  unsigned int dx = abs(n1->grid_x - n2->grid_x);
  unsigned int dy = abs(n1->grid_y - n2->grid_y);
  if (dx > dy)
    return cardinal_cost * dx + diagonal_minus_cardinal * dy;
  return cardinal_cost * dy + diagonal_minus_cardinal * dx;
}

unsigned int octile_heuristic_no_branch(Node* n1, Node* n2) {
  int dx = abs(n1->grid_x - n2->grid_x);
  int dy = abs(n1->grid_y - n2->grid_y);
  //return (half_two_cardinal_minus_diagonal * abs(dx - dy) + half_diagonal * (dx + dy));
  return (two_cardinal_minus_diagonal * abs(dx - dy) + diagonal_cost * (dx + dy)) / 2;
}
