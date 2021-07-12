#include <iostream>
#include <vector>
#include "src/Gvd.h"
#include "src/Map.h"
#include "src/TopoMap.h"
#include "src/data/Pos.h"

using namespace std;

void testGrid(StateGrid stateGrid){
  // StateGrid
  pair<Int,Int> size = stateGrid.size();
  
  cout<<"StateGrid:"<<endl;
  cout<<stateGrid<<endl;

  
  // Init topo map and update it with the stateGrid
  // meaning: update DistMap, update Gvd, update TopoMap 
  TopoMap topoMap(stateGrid.size());
  topoMap.update(stateGrid);

  // DistGrid
  cout << "DistGrid:" << endl;
  cout<<*topoMap.distMap<<endl;

  // GVD
  GvdGraph gvd = topoMap.gvd->graphGvd;

  cout << "Gvd:" << endl;

  /// Print GVD vertices in the stateGrid
  cout << "GVD in stateGrid:" << endl;
  for (int x = 0; x < size.first; x++) {
    for (int y = 0; y < size.second; y++) {
      Pos p(x,y);
      cout<<"|";
      /* if (gvd.has(p)) { */
      if (is_elem(gvd.vertices,p)) {
        cout << " *";
      }else{
        cout << stateGrid[p];
      }
    }
    cout <<"|"<<endl;
  }

  cout << "GVD vertices:" << endl;
  for (auto vp = vertices(gvd.g); vp.first != vp.second; ++vp.first){
    cout << "(" << gvd.g[*vp.first].p.x << "," << gvd.g[*vp.first].p.x << ") ";
  }
  cout << endl;

  cout << "GVD edges:" << endl;
  for (auto it = edges(gvd.g); it.first != it.second; ++it.first++){
    std::cout << "|(" << gvd.g[source(*it.first, gvd.g)].p.x << ","
              << gvd.g[source(*it.first, gvd.g)].p.y << ")-("
              << gvd.g[target(*it.first, gvd.g)].p.x << ","
              << gvd.g[target(*it.first, gvd.g)].p.y << ")| ";
  }
  cout << endl;

  // TopoMap
  criticals_info cis = topoMap.cis;
  cout << "GVD and Crits in stateGrid:" << endl;
  for (int x = 0; x < size.first; x++) {
    for (int y = 0; y < size.second; y++) {
      Pos p = Pos(x,y);
      cout<<"|";
      if(is_elem(p,cis)){
        cout << "!*";
      /* } else if(gvd.has(p)) { */
      } else if (is_elem(gvd.vertices,p)) {
        cout << " *";
      }else{
        cout << stateGrid[p];
      }
    }
    cout <<"|"<<endl;
  }

}

int main(int argc, char** argv) {
  vector<StateGrid> tests;
  StateGrid grid;
  //0
  grid.grid = {{Occupied, Occupied, Occupied, Occupied},
               {Occupied, Free, Free, Occupied},
               {Occupied, Free, Free, Occupied},
               {Occupied, Occupied, Occupied, Occupied}};
  tests.push_back(grid);

  //1
  grid.grid = {{Occupied, Occupied, Occupied, Occupied, Occupied},
               {Occupied, Free, Free, Free, Occupied},
               {Occupied, Free, Free, Free, Occupied},
               {Occupied, Free, Free, Free, Occupied},
               {Occupied, Occupied, Occupied, Occupied, Occupied}};
  tests.push_back(grid);

  //2
  grid.grid = {{Occupied, Occupied, Occupied, Occupied},
               {Occupied, Free, Free, Occupied},
               {Occupied, Free, Free, Occupied},
               {Occupied, Free, Free, Occupied},
               {Occupied, Occupied, Occupied, Occupied}};
  tests.push_back(grid);

  //3
  grid.grid = {{Occupied, Occupied, Occupied, Occupied, Occupied, Occupied},
               {Occupied, Free, Occupied, Free, Free, Occupied},
               {Occupied, Free, Occupied, Free, Occupied, Occupied},
               {Occupied, Free, Free, Free, Free, Occupied},
               {Occupied, Free, Free, Free, Free, Occupied},
               {Occupied, Free, Free, Free, Occupied, Occupied},
               {Occupied, Occupied, Occupied, Occupied, Occupied, Occupied}};
  tests.push_back(grid);

  //4
  grid.grid = {{Occupied, Occupied, Occupied, Occupied, Occupied, Occupied},
               {Occupied, Free, Free, Free, Free, Occupied},
               {Occupied, Free, Free, Free, Free, Occupied},
               {Occupied, Free, Free, Free, Free, Occupied},
               {Occupied, Occupied, Occupied, Occupied, Occupied, Occupied}};
  tests.push_back(grid);

  //5
  grid.grid = {{Occupied, Occupied, Occupied, Occupied, Occupied, Occupied},
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
  tests.push_back(grid);

  //6
  grid.grid = {{Occupied, Occupied, Occupied, Unknown, Occupied, Occupied, Occupied},
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
  tests.push_back(grid);

  //7
  grid.grid = {{Occupied, Occupied, Occupied, Occupied, Occupied, Occupied},
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
  tests.push_back(grid);

  if (argc == 2) {
    char* a = argv[1];
    int num = atoi(a);
    if (num >= tests.size() || num < 0) {
      cout << "TEST " << num << " NO EXISTE!" << endl;
      exit(1);
    }
    cout<<"Test "<<num<<endl;
    testGrid(tests[num]);
  }else{
    int i = 0;
    for(int i = 0; i < tests.size(); i++){
      cout<<"--------------------------------------------------------------------"<<endl;
      cout<<"Test "<<i<<endl;
      testGrid(tests[i]);
      cout<<endl;
    }
  }
}
