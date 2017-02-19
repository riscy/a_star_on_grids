#include <iostream>
#include <sstream>
#include <fstream>
using namespace std;
#include <cassert>
#include <cstdlib>
#include "graph.h"
#include "heuristics.h"

Node::Node() {
  this->f = 0;
  this->open = 0;
  this->closed_id = 0;
  this->g = 0;
  this->glyph = 0;
  this->heap_index = -1;
}

string Node::to_str(bool verbose) {
  stringstream bc;
  if (!verbose) {
    bc << grid_x << "," << grid_y;
    return bc.str();
  }
  bc << grid_x << "," << grid_y << endl << "Out: (";
  for (auto& neighbor_out: neighbors_out)
    bc << neighbor_out->grid_x << "," << neighbor_out->grid_y << ";";
  bc << ")" << endl << "In: (";
  for (auto& neighbor_in: neighbors_in)
    bc << neighbor_in->grid_x << "," << neighbor_in->grid_y << ";";
  bc << ")";
  return bc.str();
}

void Node::relax(int g, int h, Node* whence) {
  this->f = g + h;
  this->g = g;
  this->whence = whence;
}

void Node::expand(int problem_id) {
  this->closed_id = problem_id;
  this->open = false;
}

void Graph::clear() {
  for (auto& nd: graph_view)
    delete nd;
  grid_view.clear();
  graph_view.clear();
  width = 0;
  height = 0;
}

/// Load an ascii map.  Note this assumes the same file format as
/// Nathan Sturtevant's Benchmarks for Grid-Based Pathfinding (2012).
/// See: http://www.movingai.com/benchmarks/formats.html
void Graph::load_ascii_map(string filename, EdgeType edge_type, bool corner_cut) {
  clear();
  ifstream map_file(filename.c_str(), ios::in);
  assert(map_file.good());

  string prescribed_edge_type;
  while (!map_file.eof()) {
    string token;
    map_file >> token;
    if (token == "type")
      map_file >> prescribed_edge_type;
    else if (token == "width")
      map_file >> width;
    else if (token == "height")
      map_file >> height;
    else if (token == "map")
      break;
  }
  int xx = 0, yy = 0;
  string row;
  for (yy = 0; yy < height; ++ yy) {
    map_file >> row;
    for (xx = 0; xx < width; ++ xx) {
      if (row[xx] == '.') {
        Node * node = new Node();
        node->grid_x = xx;
        node->grid_y = yy;
        node->g = 0;
        node->f = 0;
        node->glyph = 0;
        node->open = 0;
        node->closed_id = 0;
        graph_view.push_back(node);
        grid_view.push_back(node);
      }
      else
        grid_view.push_back(0);
    }
  }
  map_file.close();

  // Connect up the neighbors
  size_t edges;
  if (edge_type == EDGES_DEFAULT) {
    if (prescribed_edge_type == "octile")
      edges = add_octile_edges(corner_cut);
    else
      edges = add_quartile_edges();
  }
  else if (edge_type == EDGES_OCTILE) {
    edges = add_octile_edges(corner_cut);
    this->cost = &octile_cost;
  }
  else {
    edges = add_quartile_edges();
    this->cost = &man_cost;
  }
  cout << filename << ": " << graph_view.size() << " nodes, " << edges << " edges" << endl;
}

void Graph::load_empty_map(int dim1, int dim2, EdgeType edge_type) {
  assert(dim1 > 0 && dim2 > 0);
  this->height = dim1;
  this->width = dim2;
  for (int yy = 0; yy < this->height; ++ yy) {
    for (int xx = 0; xx < this->width; ++ xx) {
      Node * node = new Node();
      node->grid_x = xx;
      node->grid_y = yy;
      graph_view.push_back(node);
      grid_view.push_back(node);
    }
  }
  size_t edges;
  if (edge_type == EDGES_DEFAULT || edge_type == EDGES_OCTILE) {
    edges = add_octile_edges();
    cost = &octile_cost;
  }
  else {
    edges = add_quartile_edges();
    cost = &man_cost;
  }
  cout << "Empty map:" << graph_view.size() << " nodes, " << edges << " edges" << endl;
}

void Graph::display_ascii_map() {
  for (int yy = 0; yy < this->height; ++ yy) {
    for (int xx = 0; xx < this->width; ++ xx) {
      Node * here = this->node_at(xx, yy);
      if (!here)
        cout << "#";
      else {
        if (!here->glyph)
          cout << ".";
        else
          cout << here->glyph;
      }
    }
    cout << endl;
  }
}

void Graph::display_ascii_path(Node * ss, Node * gg) {
  gg->glyph = '@';
  do {
    gg = gg->whence;
    gg->glyph = 'o';
  } while (gg != ss);
  display_ascii_map();
}

size_t Graph::add_octile_edges(bool corner_cut) {
  size_t edges = 0;
  this->edge_type = EDGES_OCTILE;
  for (size_t ii = 0; ii < graph_view.size(); ++ ii) {
    for (size_t jj = ii + 1; jj <= ii + width + 1 && jj < graph_view.size(); ++ jj) {
      if ((abs(graph_view[ii]->grid_x - graph_view[jj]->grid_x) <= 1) &&
          (abs(graph_view[ii]->grid_y - graph_view[jj]->grid_y) <= 1)) {
        edges += 1;
        graph_view[ii]->neighbors_out.push_back(graph_view[jj]);
        graph_view[jj]->neighbors_out.push_back(graph_view[ii]);
        graph_view[ii]->neighbors_in.push_back(graph_view[jj]);
        graph_view[jj]->neighbors_in.push_back(graph_view[ii]);
      }
    }
  }

  if (!corner_cut) {
    size_t cuts = 0;
    for (auto& nd1: graph_view) {
      // For each diagonal neighbor of nd1:
      for (size_t jj = 0; jj < nd1->neighbors_out.size(); ++ jj) {
        Node* dn = nd1->neighbors_out[jj];
        if (abs(nd1->grid_x - dn->grid_x) + abs(nd1->grid_y - dn->grid_y) != 2)
          continue;
        // Count my neighbors_out which are 1 grid away from you
        size_t neighbors_out = 0;
        for (auto& cn: nd1->neighbors_out) {
          if (abs(dn->grid_x - cn->grid_x) + abs(dn->grid_y - cn->grid_y) == 1)
            ++ neighbors_out;
        }
        // If it's not 2, then this edge cuts a corner
        if (neighbors_out != 2) {
          remove_edge(nd1, dn);
          remove_edge(dn, nd1);
          -- jj;
          ++ cuts;
        }
      }
    }
    edges -= cuts/2;
  }
  return edges;
}

size_t Graph::add_quartile_edges() {
  this->edge_type = EDGES_QUARTILE;
  size_t edges = 0;
  for (size_t ii = 0; ii < graph_view.size(); ++ ii) {
    for (size_t jj = ii + 1; jj <= ii + width && jj < graph_view.size(); ++ jj) {
      if (abs(graph_view[ii]->grid_x - graph_view[jj]->grid_x) +
          abs(graph_view[ii]->grid_y - graph_view[jj]->grid_y) <= 1) {
        edges += 1;
        graph_view[ii]->neighbors_out.push_back(graph_view[jj]);
        graph_view[jj]->neighbors_out.push_back(graph_view[ii]);
        graph_view[ii]->neighbors_in.push_back(graph_view[jj]);
        graph_view[jj]->neighbors_in.push_back(graph_view[ii]);
      }
    }
  }
  return edges;
}

void Graph::remove_edge(Node * from, Node * to) {
  int out_index = -1, in_index = -1;
  for (size_t ii = 0; ii < from->neighbors_out.size(); ++ ii) {
    if (from->neighbors_out[ii] == to) {
      out_index = ii;
      break;
    }
  }
  for (size_t ii = 0; ii < to->neighbors_in.size(); ++ ii) {
    if (to->neighbors_in[ii] == from) {
      in_index = ii;
      break;
    }
  }
  from->neighbors_out[out_index] = from->neighbors_out.back();
  from->neighbors_out.pop_back();
  to->neighbors_in[in_index] = to->neighbors_in.back();
  to->neighbors_in.pop_back();
}
