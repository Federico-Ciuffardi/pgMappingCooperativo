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
  grid_type grid = grid6;
  int grid_size_x = grid.size();
  int grid_size_y = grid[0].size();
  for (int x = 0; x < grid_size_x; x++) {
    for (int y = 0; y < grid_size_y; y++) {
      cout << (int)grid[x][y] << " | ";
    }
    cout << endl;
  }
  dist_grid dgrid;
  dist_pos_queue dqueue;
  boost::tie(dgrid, dqueue) = calculate_distances(grid);
  cout << "distance grid:" << endl;
  for (int x = 0; x < grid_size_x; x++) {
    for (int y = 0; y < grid_size_y; y++) {
      cout << dgrid[x][y] << " | ";
    }
    cout << endl;
  }
  grid_gvd ggvd = get_grid_gvd(dgrid, dqueue);
  cout << "gvd grid :" << endl;
  for (int x = 0; x < grid_size_x; x++) {
    for (int y = 0; y < grid_size_y; y++) {
      if (ggvd[x][y]) {
        cout << "*|";
      } else if (dgrid[x][y].distance == 0) {
        cout << "=|";
      } else {
        cout << " |";
      }
    }
    cout << endl;
  }

  gvd GVD(ggvd);
  cout << "GVD vertices: ";
  for (auto vp = vertices(GVD.g); vp.first != vp.second; ++vp.first)
    cout << "(" << GVD.g[*vp.first].p.first << "," << GVD.g[*vp.first].p.second
         << ") ";
  cout << endl;

  cout << "GVD edges: ";
  for (auto it = edges(GVD.g); it.first != it.second; ++it.first++)
    std::cout << "|(" << GVD.g[source(*it.first, GVD.g)].p.first << ","
              << GVD.g[source(*it.first, GVD.g)].p.second << ")-("
              << GVD.g[target(*it.first, GVD.g)].p.first << ","
              << GVD.g[target(*it.first, GVD.g)].p.second << ")|";
  std::cout << std::endl;
  return 0;
}
