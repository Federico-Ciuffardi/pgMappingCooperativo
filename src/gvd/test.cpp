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

grid_type grid4 = {{Occupied, Occupied, Occupied, Occupied, Occupied, Occupied},
                   {Occupied, Free, Occupied, Free, Free, Occupied},
                   {Occupied, Free, Occupied, Free, Occupied, Occupied},
                   {Occupied, Free, Free, Free, Free, Occupied},
                   {Occupied, Free, Free, Free, Free, Occupied},
                   {Occupied, Free, Free, Free, Occupied, Occupied},
                   {Occupied, Occupied, Occupied, Occupied, Occupied, Occupied}};

grid_type grid5 = {{Occupied, Occupied, Occupied, Occupied, Occupied, Occupied},
                   {Occupied, Free, Free, Free, Free, Occupied},
                   {Occupied, Free, Free, Free, Free, Occupied},
                   {Occupied, Free, Free, Free, Free, Occupied},
                   {Occupied, Occupied, Occupied, Occupied, Occupied, Occupied}};

grid_type grid6 = {{Occupied, Occupied, Occupied, Occupied, Occupied, Occupied},
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

grid_type grid7 = {{Occupied, Occupied, Occupied, Unknown, Occupied, Occupied, Occupied},
                   {Occupied, Free, Frontier, Unknown, Frontier, Free, Occupied},
                   {Occupied, Free, Frontier, Frontier, Frontier, Free, Occupied},
                   {Occupied, Free, Free, Free, Free, Free, Occupied},
                   {Occupied, Free, Free, Free, Free, Free, Occupied},
                   {Occupied, Occupied, Free, Free, Occupied, Occupied, Occupied},
                   {Occupied, Free, Free, Free, Free, Free, Occupied},
                   {Occupied, Free, Free, Free, Free, Free, Occupied},
                   {Occupied, Free, Free, Free, Free, Free, Occupied},
                   {Occupied, Free, Free, Free, Free, Free, Occupied},
                   {Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied}};

grid_type grid8 = {{Occupied, Occupied, Occupied, Occupied, Occupied, Occupied},
                   {Occupied, Free, Free, Free, Free, Occupied},
                   {Occupied, Frontier, Frontier, Free, Free, Occupied},
                   {Occupied, Unknown, Frontier, Free, Free, Occupied},
                   {Occupied, Frontier, Frontier, Free, Free, Occupied},
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
  grid_type grid = grid7;
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
  boost::tie(dgrid, dqueue) = calculate_distances(grid, Occupied);
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
      } else if (grid[x][y] == Occupied) {
        cout << "=|";
      } else if (grid[x][y] == Unknown) {
        cout << "?|";
      } else {
        cout << " |";
      }
    }
    cout << endl;
  }

  gvd GVD(ggvd);
  cout << "GVD vertices: ";
  for (auto vp = vertices(GVD.g); vp.first != vp.second; ++vp.first)
    cout << "(" << GVD.g[*vp.first].p.first << "," << GVD.g[*vp.first].p.second << ") ";
  cout << endl;

  cout << "GVD edges: ";
  for (auto it = edges(GVD.g); it.first != it.second; ++it.first++)
    std::cout << "|(" << GVD.g[source(*it.first, GVD.g)].p.first << ","
              << GVD.g[source(*it.first, GVD.g)].p.second << ")-("
              << GVD.g[target(*it.first, GVD.g)].p.first << ","
              << GVD.g[target(*it.first, GVD.g)].p.second << ")|";
  std::cout << std::endl;

  cout << "////////////////////////////////////////////////////////////" << endl;
  /*map<pos,bool> lmins = get_local_mins(dgrid, GVD);

  cout << "gvd grid :" << endl;
  for (int x = 0; x < grid_size_x; x++) {
    for (int y = 0; y < grid_size_y; y++) {
      if (ggvd[x][y]) {
        if(lmins[pos(x,y)]){
          cout << "o|";
          continue;
        }
        cout << "*|";
      } else if (grid[x][y] == Occupied) {
        cout << "=|";
      } else if(grid[x][y] == Unknown){
        cout << "?|";
      } else{
        cout << " |";
      }
    }
    cout << endl;
  }*/

  map<pos, dist_pos> cf = get_critical_points(grid, dgrid, GVD);

  // cout<< cf.size() <<endl;

  cout << "gvd grid :" << endl;
  for (int x = 0; x < grid_size_x; x++) {
    for (int y = 0; y < grid_size_y; y++) {
      if (ggvd[x][y]) {
        if (cf.find(pos(x, y)) != cf.end()) {
          cout << "o|";
          continue;
        }
        cout << "*|";
      } else if (grid[x][y] == Occupied) {
        cout << "=|";
      } else if (grid[x][y] == Unknown) {
        cout << "?|";
      } else {
        cout << " |";
      }
    }
    cout << endl;
  }

  return 0;
}
