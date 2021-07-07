#include <iostream>
#include "src/GVD.h"

using namespace std;

StateGrid grid1;
StateGrid grid2;
StateGrid grid3;
StateGrid grid4; 
StateGrid grid5;
StateGrid grid6;
StateGrid grid7;
StateGrid grid8;



int main(int argc, char** argv) {
  grid1.grid = {{Occupied, Occupied, Occupied, Occupied},
                {Occupied, Free, Free, Occupied},
                {Occupied, Free, Free, Occupied},
                {Occupied, Occupied, Occupied, Occupied}};

  grid2.grid = {{Occupied, Occupied, Occupied, Occupied, Occupied},
                {Occupied, Free, Free, Free, Occupied},
                {Occupied, Free, Free, Free, Occupied},
                {Occupied, Free, Free, Free, Occupied},
                {Occupied, Occupied, Occupied, Occupied, Occupied}};

  grid3.grid = {{Occupied, Occupied, Occupied, Occupied},
                {Occupied, Free, Free, Occupied},
                {Occupied, Free, Free, Occupied},
                {Occupied, Free, Free, Occupied},
                {Occupied, Occupied, Occupied, Occupied}};


  grid4.grid = {{Occupied, Occupied, Occupied, Occupied, Occupied, Occupied},
                {Occupied, Free, Occupied, Free, Free, Occupied},
                {Occupied, Free, Occupied, Free, Occupied, Occupied},
                {Occupied, Free, Free, Free, Free, Occupied},
                {Occupied, Free, Free, Free, Free, Occupied},
                {Occupied, Free, Free, Free, Occupied, Occupied},
                {Occupied, Occupied, Occupied, Occupied, Occupied, Occupied}};
  grid5.grid = {{Occupied, Occupied, Occupied, Occupied, Occupied, Occupied},
                {Occupied, Free, Free, Free, Free, Occupied},
                {Occupied, Free, Free, Free, Free, Occupied},
                {Occupied, Free, Free, Free, Free, Occupied},
                {Occupied, Occupied, Occupied, Occupied, Occupied, Occupied}};

  grid6.grid = {{Occupied, Occupied, Occupied, Occupied, Occupied, Occupied},
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

  grid7.grid = {{Occupied, Occupied, Occupied, Unknown, Occupied, Occupied, Occupied},
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

  grid8.grid = {{Occupied, Occupied, Occupied, Occupied, Occupied, Occupied},
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




  StateGrid grid = grid8;
  pair<Int,Int> size = grid.size();
  for (int x = 0; x < size.first; x++) {
    for (int y = 0; y < size.second; y++) {
      cout << (int)grid.cell(x,y) << " | ";
    }
    cout << endl;
  }
  DistGrid dgrid;
  DistPosQueue dqueue;
  boost::tie(dgrid, dqueue) = calculate_distances(grid, Occupied);
  cout << "Obstacles at min dist:" << endl;
  for (int x = 0; x < size.first; x++) {
    for (int y = 0; y < size.second; y++) {
      cout << dgrid.cell(x,y) << " | ";
    }
    cout << endl;
  }
  GridGvd ggvd = get_grid_gvd(dgrid, dqueue);
  cout << "GVD grid (* GVD, = obstacle, ? unknown,  free:" << endl;
  for (int x = 0; x < size.first; x++) {
    for (int y = 0; y < size.second; y++) {
      if (ggvd.cell(x,y)) {
        cout << "*|";
      } else if (grid.cell(x,y) == Occupied) {
        cout << "=|";
      } else if (grid.cell(x,y) == Unknown) {
        cout << "?|";
      } else {
        cout << " |";
      }
    }
    cout << endl;
  }
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
