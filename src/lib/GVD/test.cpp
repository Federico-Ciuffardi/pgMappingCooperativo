#include <iostream>
#include "src/GVD.h"

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
  grid_type grid = grid8;
  int grid_size_x = grid.size();
  int grid_size_y = grid[0].size();
  for (int x = 0; x < grid_size_x; x++) {
    for (int y = 0; y < grid_size_y; y++) {
      cout << (int)grid[x][y] << " | ";
    }
    cout << endl;
  }
  dist_grid dgrid;
  DistPosQueue dqueue;
  boost::tie(dgrid, dqueue) = calculate_distances(grid, Occupied);
  cout << "distance grid:" << endl;
  for (int x = 0; x < grid_size_x; x++) {
    for (int y = 0; y < grid_size_y; y++) {
      cout << dgrid[x][y] << " | ";
    }
    cout << endl;
  }
  grid_gvd ggvd = get_grid_gvd(dgrid, dqueue);
  cout << "GVD grid :" << endl;
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
  /*
    GVD gvd(ggvd);
    cout << "gvd vertices: ";
    for (auto vp = vertices(gvd.g); vp.first != vp.second; ++vp.first)
      cout << "(" << gvd.g[*vp.first].p.first << "," << gvd.g[*vp.first].p.second << ") ";
    cout << endl;

    cout << "gvd edges: ";
    for (auto it = edges(gvd.g); it.first != it.second; ++it.first++)
      std::cout << "|(" << gvd.g[source(*it.first, gvd.g)].p.first << ","
                << gvd.g[source(*it.first, gvd.g)].p.second << ")-("
                << gvd.g[target(*it.first, gvd.g)].p.first << ","
                << gvd.g[target(*it.first, gvd.g)].p.second << ")|";
    std::cout << std::endl;

    cout << "////////////////////////////////////////////////////////////" << endl;
    map<pos,bool> lmins = get_local_mins(dgrid, gvd);

    cout << "GVD grid :" << endl;
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
    }
    */
  /*map<pos, dist_pos> cf = get_critical_points(grid, dgrid, gvd);

  // cout<< cf.size() <<endl;

  cout << "GVD grid :" << endl;
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
  }*/
  GVD gvd;
  criticals_info cis;
  boost::tie(cis, gvd) = get_points_of_interest(grid);
  cout << "gvd vertices: ";
  for (auto vp = vertices(gvd.g); vp.first != vp.second; ++vp.first)
    cout << "(" << gvd.g[*vp.first].p.x << "," << gvd.g[*vp.first].p.x << ") ";
  cout << endl;

  cout << "gvd edges: ";
  for (auto it = edges(gvd.g); it.first != it.second; ++it.first++)
    std::cout << "|(" << gvd.g[source(*it.first, gvd.g)].p.x << ","
              << gvd.g[source(*it.first, gvd.g)].p.y << ")-("
              << gvd.g[target(*it.first, gvd.g)].p.x << ","
              << gvd.g[target(*it.first, gvd.g)].p.y << ")|";
  std::cout << std::endl;

  // get_path(gvd,pos(1,1),pos(12,4));

  return 0;
}
