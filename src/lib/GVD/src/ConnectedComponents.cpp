#include "ConnectedComponents.h"
#include <utility>
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
  stack<pair<Pos,IdType>> s;

  add(p, id);
  s.push(make_pair(p,id));

  while(!s.empty()){
    p  = s.top().first;
    id = s.top().second;
    s.pop();

    for (Pos pN : map.adj(p,nonTraversables)) {
      if(idGrid[pN] == NULL_ID){
        add(pN,id);
        s.push(make_pair(pN, id));
      }
    }
  }
}

ConnectedComponents::IdType ConnectedComponents::genId(){
  return lastId++;
}

void ConnectedComponents::update(){
  // Clean old result
  lastId = firstId;
  connectedComponents.clear();
  idGrid = Grid<IdType>(map.size(), NULL_ID);

  // Get new result
  for(Pos p : map){
    if(!is_elem(map[p],nonTraversables) && idGrid[p] == NULL_ID){
      fill(p, genId());
    }
  }
}
