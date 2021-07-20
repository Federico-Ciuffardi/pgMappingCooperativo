#include "ConnectedComponents.h"
#include "utils.h"

ConnectedComponents::ConnectedComponents(MapType& map, vector<CellType> nonTraversables) : map(map){
  this->idGrid = Grid<IdType>(map.size(), NULL_ID);
  this->nonTraversables = nonTraversables;
}



void ConnectedComponents::add(Pos p, IdType id){
  idGrid[p] = id;
  connectedComponents[id].members.insert(p);
  connectedComponents[id].typeMembers[map[p]].insert(p);
} 

void ConnectedComponents::fill(Pos p, IdType id){
  add(p, id);
  for (Pos np : map.adj(p,nonTraversables)) {
    if(idGrid[np] == NULL_ID){
      fill(np, id);
    }
  }
}

ConnectedComponents::IdType ConnectedComponents::genId(){
  IdType i = firstId;
  for(auto it : connectedComponents){
    if (it.first != i) break;
    i++;
  }
  return i;
}

void ConnectedComponents::update(){
  // Clean old result
  connectedComponents.clear();
  idGrid = Grid<IdType>(map.size(), NULL_ID);

  // Get new result
  for(Pos p : map){
    if(!is_elem(map[p],nonTraversables) && idGrid[p] == NULL_ID){
      fill(p, genId());
    }
  }
}
