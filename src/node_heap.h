#include <vector>
using namespace std;

/// Implementation of a binary heap implemented on top of a vector.
namespace node_heap {
  /// In A*, one node is 'better' than the other when it has a lower f cost.
  bool better(Node* n1, Node* n2) {
    // tiebreak on larger g
    return (n1->f < n2->f) || (n1->f == n2->f && n1->g > n2->g);
  }

  void repair(vector<Node*> & open_list, int ii) {
    while (true) {
      int parent = (ii + 1) / 2 - 1;
      if (parent < 0)
        break;
      if (!better(open_list[ii], open_list[parent]))
        break;
      swap(open_list[ii], open_list[parent]);
      swap(open_list[ii]->heap_index, open_list[parent]->heap_index);
      ii = parent;
    }
  }

  void push(vector<Node*> & open_list, Node* add_me) {
    open_list.push_back(add_me);
    add_me->heap_index = open_list.size() - 1;
    repair(open_list, add_me->heap_index);
  }

  void pop(vector<Node*> & open_list) {
    open_list.front() = open_list.back();
    open_list.front()->heap_index = 0;
    open_list.pop_back();

    for (size_t ii = 0;;) {
      int son1 = 2 * ii + 1;
      int son2 = 2 * ii + 2;

      if (son1 >= (int) open_list.size())
        return;
      if (son2 >= (int) open_list.size())
        son2 = son1;
      if (better(open_list[ii], open_list[son1]) &&
          better(open_list[ii], open_list[son2]))
        return;

      if (!better(open_list[ii], open_list[son1]) &&
          better(open_list[son1], open_list[son2])) {
        swap(open_list[ii], open_list[son1]);
        swap(open_list[ii]->heap_index, open_list[son1]->heap_index);
        ii = son1;
        continue;
      }
      else {
        swap(open_list[ii], open_list[son2]);
        swap(open_list[ii]->heap_index, open_list[son2]->heap_index);
        ii = son2;
      }
    }
  }
}
