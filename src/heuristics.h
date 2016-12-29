#ifndef HEURISTICS_H
#define HEURISTICS_H

class Node;

// Cost functions
void grid_costs(int new_cardinal_cost, int new_diagonal_cost);
unsigned int inf_cost(Node*, Node*);
unsigned int man_cost(Node*, Node*);
unsigned int octile_cost(Node*, Node*);

// Heuristic functions
unsigned int zero_heuristic(Node*, Node*);
unsigned int man_heuristic(Node*, Node*);
unsigned int inf_heuristic(Node*, Node*);
unsigned int octile_heuristic(Node*, Node*);
unsigned int octile_heuristic_no_branch(Node*, Node*);

#endif // HEURISTICS_H
