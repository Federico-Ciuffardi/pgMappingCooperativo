#pragma once

#include <bits/stdc++.h>
#include <iostream>

#include <math.h>
#include <cfloat>
#include <iterator>

#include "Pos.h"

using namespace std;

template<typename cell_type>
struct Grid{
  typedef cell_type CellType;
  typedef vector<CellType> ColType;
  typedef vector<ColType>  GridType;
  typedef typename ColType::reference reference;
  GridType grid;

  // diplacement to possible neighbors (defaults to 8-connected)
  // must be ordered clockwise
  vector<Pos> neighborDisplacement = {Pos(-1, -1), Pos(-1, 0), Pos(-1, 1), Pos(0, 1),
                                      Pos(1, 1),   Pos(1, 0),  Pos(1, -1), Pos(0, -1)};
  /* vector<Pos> neighborDisplacement = {Pos(-1, 0), Pos(0, 1), Pos(1, 0), Pos(0, -1)}; */

  Grid(){}

  Grid(pair<Int,Int> size){
    for (Int x = 0; x < size.first; x++) {
      grid.push_back(ColType());
      for (Int y = 0; y < size.second; y++) {
        grid.at(x).push_back(CellType());
      }
    }
  }
  Grid(pair<Int,Int> size, CellType def){
    for (Int x = 0; x < size.first; x++) {
      grid.push_back(ColType());
      for (Int y = 0; y < size.second; y++) {
        grid.at(x).push_back(def);
      }
    }
  }

  // returns grid size
  pair<Int, Int> size() {
    return pair<Int, Int>(grid.size(), grid.at(0).size());
  }

  // true if Pos is inside the grid bounds
  bool inside(Int x, Int y) {
    pair<Int, Int> size = this->size();
    return ((x >= 0 && x < size.first) && (y >= 0 && y < size.second));
  }

  // true if Pos is inside the grid bounds
  bool inside(Pos p) {
    return inside(p.x,p.y);
  }

  // returns the cell corresponding to x, y
  reference cell(Int x, Int y) {
    return grid.at(x).at(y);
  }
  // returns the cell corresponding to Pos p
  reference cell(Pos p) {
    return grid.at(p.x).at(p.y);
  }
  // returns the cell corresponding to Pos p
  reference operator[](Pos p){
    return grid.at(p.x).at(p.y);
  }
  // returns the Col corresponding to x
  ColType operator[](Int x){
    return grid.at(x);
  }

  // returns all the neighbors Pos of a given Pos p in the grid
  vector<Pos> adj(Pos p) {
    vector<Pos> adj;
    for (Pos displacement : neighborDisplacement) {
      Pos n = p + displacement;
      if (!inside(n)) continue; 
      adj.push_back(n);
    }
    return adj;
  }
  // returns all *valid* neighbors Pos of a given Pos p in the grid
  vector<Pos> adj(Pos p, vector<CellType> invalids) {
    vector<Pos> adj;
    for (Pos displacement : neighborDisplacement) {
      Pos n = p + displacement;
      if (!inside(n)) continue; 

      bool valid = true;
      for(CellType invalid : invalids){
        valid = cell(n) != invalid;
        if(!valid) break;
      }
      if (valid) {
        adj.push_back(n);
      }
    }
    return adj;
  }
  class Iterator : public std::iterator<output_iterator_tag, Pos>{
    Pos p;
    pair<Int,Int> size;
  public:
    Iterator(pair<Int,Int> size, Pos p = Pos() ) { 
      this-> p = p;
      this->size = size;
    }
    Iterator& operator++() {
      if ( ++p.x >= size.first ){
        p.x = 0;
        p.y++;
      }
      return (*this);
    }
    Iterator operator++(int) {
      Iterator tmp(*this); 
      operator++(); 
      return tmp;
    }
    bool operator==(const Iterator& it) const {
      return p==p;
    }
    bool operator!=(const Iterator& it) const {
      return p!=it.p;
    }
    Pos operator*() {
      return p;
    }
  };

  Iterator begin(){
    return Iterator(size());
  }
  Iterator end(){
    pair<Int,Int> size = this->size(); 
    return Iterator(size,Pos(0,size.second));
  }
  template<typename CellType>
  friend ostream& operator<<(ostream& out, Grid<CellType>&);
};


template<typename CellType>
ostream& operator<<(ostream& out, Grid<CellType>& grid){
  pair<Int,Int> size = grid.size(); 
  for(int x = 0; x < size.first ; x++){
    for(int y = 0; y < size.second; y++){
      out<<"|"<<grid[x][y];
    }
    out<<"|"<<endl;
  }
  return out;
}

