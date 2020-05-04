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

grid_type grid5 = {
    {Occupied, Occupied, Occupied, Occupied, Occupied, Occupied},
    {Occupied, Free, Free, Free, Free, Occupied},
    {Occupied, Free, Free, Free, Free, Occupied},
    {Occupied, Free, Free, Free, Free, Occupied},
    {Occupied, Occupied, Occupied, Occupied, Occupied, Occupied}};

grid_type grid6 = {
    {Occupied, Occupied, Occupied, Occupied, Occupied, Occupied},
    {Occupied, Free, Free, Free, Free, Occupied},
    {Occupied, Free, Free, Free, Free, Occupied},
    {Occupied, Free, Free, Free, Free, Occupied},
    {Occupied, Free, Free, Free, Free, Occupied},
    {Occupied, Free, Free, Free, Free, Occupied},
    {Occupied, Free, Free, Free, Free, Occupied},
    {Occupied, Free, Free, Free, Free, Occupied},
    {Occupied, Free, Free, Free, Free, Occupied},
    {Occupied, Free, Free, Free, Free, Occupied},
    {Occupied, Free, Free, Free, Free, Occupied},
    {Occupied, Free, Free, Free, Free, Occupied},
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
  pair<dist_grid, priority_queue<dist_pos>> ret_val = calculate_distances(grid);
  cout << "distance grid:" << endl;
  for (int x = 0; x < grid_size_x; x++) {
    for (int y = 0; y < grid_size_y; y++) {
      cout << ret_val.first[x][y] << " | ";
    }
    cout << endl;
  }
  vector<vector<bool>> grid_gvd = get_grid_gvd(ret_val.first, ret_val.second);
  cout << "gvd grid :" << endl;
  for (int x = 0; x < grid_size_x; x++) {
    for (int y = 0; y < grid_size_y; y++) {
      if (grid_gvd[x][y]) {
        cout << "*|";
      } else if (ret_val.first[x][y].distance == 0) {
        cout << "=|";
      } else {
        cout << " |";
      }
    }
    cout << endl;
  }
  return 0;
}
