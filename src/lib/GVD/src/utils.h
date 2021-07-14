#pragma once

#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include "data/Pos.h"
#include "data/Grid.h"

using namespace std;

////// productive
template <typename T>
inline bool is_elem(boost::unordered_set<T> s, T e) {
  return s.find(e) != s.end();
}

template <typename K, typename T>
inline bool is_elem(boost::unordered_map<K, T> m, K e) {
  return m.find(e) != m.end();
}

template <typename K, typename T>
inline bool is_elem(map<K, T> m, K e) {
  return m.find(e) != m.end();
}

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
inline bool is_elem(T e, boost::unordered_set<T> s) {
  return s.find(e) != s.end();
}

template <typename K, typename T>
inline bool is_elem(K e, boost::unordered_map<K, T> m) {
  return m.find(e) != m.end();
}

template <typename K, typename T>
inline bool is_elem(K e, map<K, T> m) {
  return m.find(e) != m.end();
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

///// Test/Debug Aux

/* Print sets */
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

/* Print vectors */
template <typename T>
ostream& operator<<(ostream& out, const vector<T>& v) {
  if (!v.empty()) {
    out << '[';
    std::copy(v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
    out << "\b\b]";
  }
  return out;
}

/* Print maps */
template <typename K, typename T>
ostream& operator<<(ostream& out, const map<K, T>& m) {
  out << "[";
  for (auto it = m.begin(); it != m.end(); it++) {
    out << "(" << it->first << ":" << it->second << ")";
  }
  out << "]";
  return out;
}

/* Print maps */
template <typename K, typename T>
ostream& operator<<(ostream& out, const boost::unordered_map<K, T>& m) {
  out << "[";
  for (auto it = m.begin(); it != m.end(); it++) {
    out << "(" << it->first << ":" << it->second << ")";
  }
  out << "]";
  return out;
}

/* Print grid Property (the cells of gt present set) */
template <typename T>
inline void print_property(PosSet set, Grid<T>& gt) {
  int mapWidth = gt.size();
  int mapHeight = gt[0].size();
  for (int x = 0; x < mapWidth; x++) {
    for (int y = 0; y < mapHeight; y++) {
      cout << "|" << is_elem(set, Pos(x, y));
    }
    cout << "|" << endl;
  }
}

/* Print grid Property (the cells of grid present in the map) */
template <typename T>
inline void print_property(boost::unordered_map<Pos, int> map, Grid<T>& gt) {
  int mapWidth = gt.size();
  int mapHeight = gt[0].size();
  for (int x = 0; x < mapWidth; x++) {
    for (int y = 0; y < mapHeight; y++) {
      cout << "|" << map[Pos(x, y)];
    }
    cout << "|" << endl;
  }
}
