#include <iostream>
#include "gvd.h"

using namespace std;

grid_type grid1 = {{Occupied, Occupied, Occupied, Occupied},
                   {Occupied, Free, Free, Occupied},
                   {Occupied, Free, Free, Occupied},
                   {Occupied, Occupied, Occupied, Occupied}};

grid_type grid2 = {{Occupied, Occupied, Occupied, Occupied, Occupied},
                   {Occupied, Free, Free, Free, Occupied},
                   {Occupied, Free, Free, Free, Occupied},
                   {Occupied, Free, Free, Free, Occupied},
                   {Occupied, Occupied, Occupied, Occupied, Occupied}};

grid_type grid3 = {{Occupied, Occupied, Occupied, Occupied},
                   {Occupied, Free, Free, Occupied},
                   {Occupied, Free, Free, Occupied},
                   {Occupied, Free, Free, Occupied},
                   {Occupied, Occupied, Occupied, Occupied}};

grid_type grid4 = {
    {Occupied, Occupied, Occupied, Occupied, Occupied, Occupied},
    {Occupied, Free, Occupied, Free, Free, Occupied},
    {Occupied, Free, Occupied, Free, Occupied, Occupied},
    {Occupied, Free, Free, Free, Free, Occupied},
    {Occupied, Free, Free, Free, Free, Occupied},
    {Occupied, Free, Free, Free, Occupied, Occupied},
    {Occupied, Occupied, Occupied, Occupied, Occupied, Occupied}};

int main(int argc, char** argv) {
  grid_type grid = grid4;
  int grid_size_x = grid.size();
  int grid_size_y = grid[0].size();
  for (int x = 0; x < grid_size_x; x++) {
    for (int y = 0; y < grid_size_y; y++) {
      cout << (int)grid[x][y] << " | ";
    }
    cout << endl;
  }
  dist_grid gd = calculate_distances(grid);
  for (int x = 0; x < grid_size_x; x++) {
    for (int y = 0; y < grid_size_y; y++) {
      cout << gd[x][y] << " | ";
    }
    cout << endl;
  }
  vector<vector<bool>> grid_gvd = get_grid_gvd(gd);
  for (int x = 0; x < grid_size_x; x++) {
    for (int y = 0; y < grid_size_y; y++) {
      cout << grid_gvd[x][y] << " | ";
    }
    cout << endl;
  }
  return 0;
}
