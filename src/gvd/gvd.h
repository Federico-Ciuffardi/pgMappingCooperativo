#ifndef GVD_H
#define GVD_H

#include <bits/stdc++.h>
#include <iostream>

using namespace std;

typedef enum { Occupied, Unknown, Free } cell_type;

typedef vector<cell_type> row_type;
typedef vector<row_type> grid_type;

typedef pair<int, int> pos;

struct dist_cell {
  vector<pos> obs;

  float distance;

  dist_cell(){};

  void add_obs(pos);

  bool has_obs(pos);

  bool operator>(const dist_cell& d) const;

  bool operator==(const dist_cell& p) const;

  friend ostream& operator<<(ostream& out, const dist_cell& cell);
};

typedef vector<dist_cell> dist_col;
typedef vector<dist_col> dist_grid;

typedef pair<float, pos> dist_pos;

pair<dist_grid, priority_queue<dist_pos>> calculate_distances(grid_type ogrid);

vector<vector<bool>> get_grid_gvd(dist_grid dg, priority_queue<dist_pos>);

#endif