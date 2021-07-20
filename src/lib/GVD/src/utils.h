#pragma once

#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include "data/Pos.h"
#include "data/Grid.h"

using namespace std;

// productive

/// discretize line from p1 to p2 and return the cells of the discrtized line
/// asumes that the slope of the line p1 to p2 is less or equal to 1  and that p1.x < p2.x

/// * source:
///     Foley, Van Dam, Feiner, Hughes, Phillips, "Introducción a la Graficación
///     por Computador", Addisson-Wesley Iberoamericana, S.A., 1996, ISBN
///     0-201-6599-7 
inline vector<Pos> discretizeLineAux(int x0, int y0, int x1, int y1){
  vector<Pos> line;

  int dx = x1 - x0;
  int dy = y1 - y0;
  int d  = 2 * dy - dx;
  int increE = 2 * dy;
  int incrNE = 2 * (dy - dx);
  int x = x0;
  int y = y0;
  line.push_back(Pos(x,y));

  while(x<x1){
    if (d <= 0) {
      d+=increE;
      x++;
    } else {
      d += incrNE;
      x++;
      y++;
    }
    line.push_back(Pos(x,y));
  }

  return line;
}

/// discretizeLineAux wrapper to modify p1 and p2 to comply with the
/// discretizeLineAux assumptions and then correct the discretizeLineAux result
/// to be valid with the original p1 and p2
inline vector<Pos> discretizeLine(Pos p1, Pos p2){
  Pos deltaP = p1 - p2;
  bool swapXY = abs(deltaP.x) < abs(deltaP.y);
  if(swapXY){
    swap(p1.x,p1.y);
    swap(p2.x,p2.y);
  }

  bool negX = p1.x > p2.x; 
  if(negX){
    swap(p1,p2);
  }

  vector<Pos> line = discretizeLineAux(p1.x,p1.y,p2.x,p2.y);

  if(swapXY){
    vector<Pos> auxLine;
    for(Pos linePos : line){
      swap(linePos.x,linePos.y);
      auxLine.push_back(linePos);
    }
    line = auxLine;
  }

  /* if(negX){ */
    // the line is backwards could change that here
  /* } */

  return line;
}

/// is_elem, returns true if elem belongs to collection
//// unordered_set
template <typename T>
inline bool is_elem(boost::unordered_set<T>& s, T e) {
  return s.find(e) != s.end();
}
template <typename T>
inline bool is_elem(T e, boost::unordered_set<T>& s) {
  return s.find(e) != s.end();
}

//// unordered_map
template <typename K, typename T>
inline bool is_elem(boost::unordered_map<K, T>& m, K e) {
  return m.find(e) != m.end();
}
template <typename K, typename T>
inline bool is_elem(K e, boost::unordered_map<K, T>& m) {
  return m.find(e) != m.end();
}

//// map
template <typename K, typename T>
inline bool is_elem(map<K, T>& m, K e) {
  return m.find(e) != m.end();
}

template <typename K, typename T>
inline bool is_elem(K e, map<K, T>& m) {
  return m.find(e) != m.end();
}

//// vector
template <typename T>
inline bool is_elem(vector<T> v, T e) {
  for(int i = 0; i < v.size() ; i++){
    if(v[i] == e){
      return true;
    }
  }
  return false;
}
template <typename T>
inline bool is_elem(T e,vector<T> v) {
  for(int i = 0; i < v.size() ; i++){
    if(v[i] == e){
      return true;
    }
  }
  return false;
}

/// Transform from unordered_set to vector
template<typename T>
inline vector<T> toVec(boost::unordered_set<T> set){
  vector<T> res;
  for(T t : set){
    res.push_back(t);
  }
  return res;
}

/// get the closests positions from pSet to p and the distance of those
/// closest positions to p
inline boost::tuple<Float, PosSet> closests(Pos p, PosSet pSet) {
  Float minD = INF;
  PosSet minDPs;
  for (Pos p1 : pSet) {
    Float d = p.distanceTo(p1);
    if (d < minD) {
      minD = d;
      minDPs.clear();
      minDPs.insert(p1);
    } else if (d == minD) {
      minDPs.insert(p1);
    }
  }
  return boost::make_tuple(minD, minDPs);
}


// Test/Debug Aux

/// Print sets
template <typename T>
ostream& operator<<(ostream& out, const boost::unordered_set<T>& v) {
  if (!v.empty()) {
    out << '[';
    for (auto it = v.begin(); it != v.end(); it++) {
      out << *it << ",";
    }
    out << "]";
  }
  return out;
}

/// Print vectors
template <typename T>
ostream& operator<<(ostream& out, const vector<T>& v) {
  if (!v.empty()) {
    out << '[';
    std::copy(v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
    out << "\b\b]";
  }
  return out;
}

/// Print maps */
template <typename K, typename T>
ostream& operator<<(ostream& out, const map<K, T>& m) {
  out << "[";
  for (auto it = m.begin(); it != m.end(); it++) {
    out << "(" << it->first << ":" << it->second << ")";
  }
  out << "]";
  return out;
}

/// Print maps
template <typename K, typename T>
ostream& operator<<(ostream& out, const boost::unordered_map<K, T>& m) {
  out << "[";
  for (auto it = m.begin(); it != m.end(); it++) {
    out << "(" << it->first << ":" << it->second << ")";
  }
  out << "]";
  return out;
}

/// Print grid Property (the cells of gt present set)
template <typename T>
inline void print_property(PosSet& set, Grid<T>& gt) {
  int mapWidth = gt.size();
  int mapHeight = gt[0].size();
  for (int x = 0; x < mapWidth; x++) {
    for (int y = 0; y < mapHeight; y++) {
      cout << "|" << is_elem(set, Pos(x, y));
    }
    cout << "|" << endl;
  }
}

/// Print grid Property (the cells of grid present in the map)
template <typename T>
inline void print_property(boost::unordered_map<Pos, int>& map, Grid<T>& gt) {
  int mapWidth = gt.size();
  int mapHeight = gt[0].size();
  for (int x = 0; x < mapWidth; x++) {
    for (int y = 0; y < mapHeight; y++) {
      cout << "|" << map[Pos(x, y)];
    }
    cout << "|" << endl;
  }
}
