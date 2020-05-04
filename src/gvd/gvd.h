#ifndef GVD_H
#define GVD_H

#include <heap.h>
#include <iostream>

using namespace std;

typedef enum { Occupied, Unknown, Free } cell_type;

typedef vector<cell_type> row_type;
typedef vector<row_type> grid_type;

struct dist_cell {
  vector<pair<int, int>> obs;
  float distance;
  dist_cell(){};

  void add_obs(int x, int y);

  bool has_obs(int x, int y);

  bool operator>(const dist_cell& d) const;

  friend ostream& operator<<(ostream& out, const dist_cell& cell);
};

typedef vector<dist_cell> dist_col;
typedef vector<dist_col> dist_grid;

dist_grid calculate_distances(grid_type original_grid);

vector<vector<bool>> get_grid_gvd(dist_grid dg);

#endif