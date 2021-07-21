#include <iostream>
#include <vector>
#include "src/ConnectedComponents.h"
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
  TopoMap topoMap(stateGrid);
  topoMap.update();

  // DistGrid
  cout << "DistGrid:" << endl;
  cout<<topoMap.distMap->distMap<<endl;

  // GVD
  GvdGraph& gvd = *topoMap.gvd->graphGvd;

  cout << "Gvd:" << endl;

  /// Print GVD vertices in the stateGrid
  cout << "GVD in stateGrid:" << endl;
  for (int x = 0; x < size.first; x++) {
    for (int y = 0; y < size.second; y++) {
      Pos p(x,y);
      cout<<"|";
      if (gvd.has(p)) {
        cout << " *";
      }else{
        cout << stateGrid[p];
      }
    }
    cout <<"|"<<endl;
  }

  /* cout << "GVD vertices:" << endl; */
  /* for (GvdGraph::Vertex v : gvd){ */
  /*   cout<<"|"<< gvd.g[v].p<<"|";//<<"->"<<v<<" : gr "; */
  /*   /1* cout<<gvd.degree(v)<<endl; *1/ */
  /* } */
  /* cout << endl; */

  /* cout << "GVD edges:" << endl; */
  /* for (auto it = edges(gvd.g); it.first != it.second; ++it.first++){ */
  /*   std::cout << "|" << gvd.g[source(*it.first, gvd.g)].p << "-" << gvd.g[target(*it.first, gvd.g)].p << "| "; */
  /* } */
  /* cout << endl; */

  // TopoMap
  CriticalInfos cis = topoMap.cis;
  cout << "GVD and Crits in stateGrid:" << endl;
  for (int x = 0; x < size.first; x++) {
    for (int y = 0; y < size.second; y++) {
      Pos p = Pos(x,y);
      cout<<"|";
      if(is_elem(p,cis)){
        cout << "!*";
      } else if(gvd.has(p)) {
        cout << " *";
      }else{
        cout << stateGrid[p];
      }
    }
    cout <<"|"<<endl;
  }

  // Connected Componnents
  cout<<"Segments: "<<endl;
  for (int x = 0; x < size.first; x++) {
    for (int y = 0; y < size.second; y++) {
      Pos p = Pos(x,y);
      cout<<"|";
      if(topoMap.segmenter->idGrid[p] != NULL_ID){
        cout << " "<< topoMap.segmenter->idGrid[p];
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

  //8
  grid.grid = {{Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied},
               {Occupied, Free,     Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied},
               {Occupied, Free,     Free,     Occupied, Occupied, Occupied, Occupied, Occupied, Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Occupied, Occupied, Occupied, Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Occupied, Occupied, Occupied, Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Occupied, Occupied, Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied, Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied}};
  tests.push_back(grid);
  //9 esquina inferior izquierda no pertenece al GVD por culpa del cleanup (esto es correcto)
  grid.grid = {{Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied},
               {Occupied, Free,     Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied},
               {Occupied, Free,     Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied},
               {Occupied, Free,     Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied}};
  tests.push_back(grid);

  //10 componentes conexas
  grid.grid = {{Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied},
               {Occupied, Free,     Occupied, Free,     Free,     Occupied, Free,     Occupied, Free,     Occupied, Occupied},
               {Occupied, Free,     Occupied, Free,     Free,     Occupied, Occupied, Occupied, Occupied, Occupied, Occupied},
               {Occupied, Free,     Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied},
               {Occupied, Free,     Free,     Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Occupied, Occupied, Occupied},
               {Occupied, Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied},
               {Occupied, Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Occupied, Free,     Occupied},
               {Occupied, Free,     Free,     Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied}};
  tests.push_back(grid);

  //10 componentes conexas + lineas criticas
  grid.grid = {{Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Occupied, Free,     Free,     Free,     Occupied, Occupied, Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Occupied, Free,     Free,     Free,     Occupied, Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Occupied, Occupied, Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Occupied, Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Occupied, Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Occupied, Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Occupied, Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Occupied, Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Occupied, Occupied, Free,     Occupied, Occupied, Occupied, Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Occupied, Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Frontier, Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Unknown,  Unknown,  Unknown,  Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Unknown,  Unknown,  Unknown,  Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied}};
  tests.push_back(grid);

  //11 componentes conexas + lineas criticas
  grid.grid = {{Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Occupied, Free,     Free,     Free,     Occupied, Occupied, Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Occupied, Occupied, Free,     Occupied, Occupied, Occupied, Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Free,     Free,     Free,     Occupied, Free,     Free,     Free,     Free,     Free,     Free,     Occupied},
               {Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied, Occupied}};
  tests.push_back(grid);

  if (argc == 2) {
    char* a = argv[1];
    int num = atoi(a);
    if (num >= tests.size() || num < 0) {
      cout << "TEST " << num << " NO EXISTE!" << endl;
      exit(1);
    }
    cout<<"Test "<<num<<endl;
    for(int i = 0; i < tests[num].size().first; i++){
      for(int j = 0; j < tests[num].size().second; j++){
        cout<<"|"<<Pos(i,j)<<"|";
      }
      cout<<endl;
    }
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
