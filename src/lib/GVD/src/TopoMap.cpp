#include "TopoMap.h"
#include <iostream>
#include <vector>
#include "Gvd.h"
#include "Map.h"
#include "utils.h"

// Returns all the positions of distance local mins vertices
// Local min here means:
//  * there is no neighbor with less distance
//  * there is at least one neighbor with greater distance
PosSet getLocalMins(DistMap& dg, GvdGraph& gvd) {
  PosSet localMins;
  PosSet notLocalMins;

  for (auto it : gvd.idVertexMap) {
    Pos p = it.first;

    if (is_elem(p,localMins) || is_elem(p,notLocalMins)) continue; // already processed

    bool isMin = true;
    bool hasGreater = false;

    for (Pos pN : gvd.adj(p)) {

      // p must have at least one neighbor with greater distance
      if (dg[p].distance < dg[pN].distance) { 
        hasGreater = true; // p has a neighbor with greater distance
        notLocalMins.insert(pN); // pN is not a min
      }

      // neighbor of p must be grater or equal
      isMin = isMin && dg[p].distance <= dg[pN].distance; 

      if (!isMin) break;
    }
    if (isMin && hasGreater){
      localMins.insert(p);
    }
  }
  return localMins;
}

// increase sparseness by removing redundant edges:
//
//   * in this case redundant edges are the ones that from v lead to a neighbor
//     vN1 that can be reached from another neighbor of v vN2, meaning there is a
//     path v - vN1 and a path v - vN2 - vN1, so v - vN1 can be safely removed
//
//   * v is only considered for cleanUp if it has a neighbor with a greater or
//     equal degree one of those neighbors of greater or equal degree (then v and
//     v's neighbors) will keep the connection with v
void cleanUp(GvdGraph& gvd) {
  for (GvdGraph::VertexIterator vIt = gvd.begin(); vIt != gvd.end();) {
    GvdGraph::Vertex v = *(vIt++);

    // get the vertex with max degree from the v and its neighbors
    GvdGraph::Vertex maxDegV = v;
    int maxDeg = gvd.degree(v);
    for (GvdGraph::Vertex vN : gvd.adj(v)) {
      int vNdegree = gvd.degree(vN);
      if (vNdegree >= maxDeg) {
        maxDegV = vN;
        maxDeg = vNdegree;
      }
    }

    if (maxDegV == v) continue;
                                
    GvdGraph::AdjacencyIterator avIt, avItEnd;
    for (boost::tie(avIt, avItEnd) = adjacent_vertices(v, gvd.g); avIt != avItEnd;) {
      GvdGraph::Vertex vN = *(avIt++);

      if (maxDegV == vN) continue; //skip max deg vertex

      if (edge(vN, maxDegV, gvd.g).second) {
        gvd.removeE(vN, v);
        gvd.removeE(v,vN);

        if(gvd.degree(vN)==1){
          gvd.removeE(vN,maxDegV);
          gvd.removeE(maxDegV,vN);
        }

        if(gvd.degree(v)==1){
          gvd.removeE(v,maxDegV);
          gvd.removeE(maxDegV,v);
        }

        boost::tie(avIt, avItEnd) = adjacent_vertices(v, gvd.g);
      }
    }
  }
  for (GvdGraph::VertexIterator vp = gvd.begin(); vp != gvd.end();) {
    GvdGraph::VertexIterator vpAux = vp++;
    if (out_degree(*vpAux, gvd.g) == 0) {
      gvd.removeV(*vpAux);
    }
  }
}


bool sameDirecction(Pos p1, Pos p2) {
  return p1.normalize() == -p2.normalize();
}

// Simplify the graph without removing information.
//   Remove all the non critical candidates of degree2 that lie within a
//   straight line of vertices and are not the ends. The removed vertices are
//   represented with an edge connecting the preserved ends of the line.
void collapseVertices(GvdGraph& gvd, PosSet lmins) {
  for (GvdGraph::VertexIterator vp = gvd.begin(); vp != gvd.end();) {
    GvdGraph::VertexIterator vpAux = vp++;

    Pos current_Pos = gvd[*vpAux].p;

    bool is_min = is_elem(current_Pos,lmins);
    if (!is_min && gvd.degree(*vpAux) == 2) {
      vector<GvdGraph::Vertex> adj = gvd.adj(*vpAux);

      auto adj1 = gvd[adj[0]];
      Pos pToAdj1 = adj1.p - current_Pos;

      auto adj2 = gvd[adj[1]];
      Pos pToAdj2 = (adj2.p - current_Pos);

      if (sameDirecction(pToAdj1, pToAdj2)) {
        gvd.addE(adj[0], adj[1]);
        gvd.addE(adj[1], adj[0]);
        gvd.removeV(*vpAux);
      }
    }
  }
}

// Returns two maps : criticals -> frontiers, criticals-> min dis to frontier, segments gvd
CriticalInfos unknownDistConstraint(StateGrid& ogrid, GvdGraph& gvd) {
  CriticalInfos res;

  /* DistPosQueue dqueue; */
  DistMap distMap(ogrid.size(),{Critical},{Unknown,Occupied}, {Frontier});
  cout << "unknownDistConstraint" << endl;
  distMap.update(ogrid);
  /* cout<<distMap.distMap<<endl; */

  cout << "set each frontier to a critical" << endl;
  while (!distMap.fullDQueue.empty()) {
    DistPos frontier_dp = distMap.fullDQueue.top();
    distMap.fullDQueue.pop();
    Pos frontier = frontier_dp.second;

    /* cout<<"Frontier: "<<frontier_dp.second<<endl; */
    /* cout<<"Critical: "<<distMap[frontier].obs<<endl; */
    vector<Pos>& frontier_crits = distMap[frontier].obs;

    for (int i = 0; i < frontier_crits.size(); i++) {
      Pos critical_Pos = frontier_crits[i];
      GvdVertexProperty& c = gvd[critical_Pos];
      if (!c.is_critical) {
        c.is_critical = true;
        // c.segment = critical_Pos;
        res[critical_Pos].mindToF = frontier_dp.first;
        res[critical_Pos].frontiers.push_back(frontier);
        // frontier_crits.clear();
        frontier_crits[0] = critical_Pos;
        break;
      }
      if (i == (frontier_crits.size() - 1)) {  // i is the last one
        res[critical_Pos].frontiers.push_back(frontier);
        // frontier_crits.clear();
        frontier_crits[0] = critical_Pos;
      }
    }
  }
  cout << "set the segment for every vertex" << endl;
  GvdGraph::VertexIterator v_it, v_it_end;
  for (tie(v_it, v_it_end) = vertices(gvd.g); v_it != v_it_end; v_it++) {
    GvdVertexProperty& v = gvd[*v_it];
    if (distMap[v.p].obs.size() > 0) {
      v.segment = distMap[v.p].obs[0];
    } else {
      v.segment = Pos(INT_MIN, INT_MIN);
      cout << "warning vertex without segment" << endl;
    }
  }
  cout << "Done" << endl;

  return res;
}

int degree_constraint(StateGrid& ogrid, GvdGraph& gvd, PosSet local_mins) {
  int criticals_count = 0;
  Pos current_Pos;
  for (auto vp = vertices(gvd.g); vp.first != vp.second; ++vp.first) {
    current_Pos = gvd[*vp.first].p;
    if (out_degree(*vp.first, gvd.g) == 2 && is_elem(current_Pos,local_mins)) {
      for (auto ad = adjacent_vertices(*vp.first, gvd.g); ad.first != ad.second; ++ad.first) {
        if (out_degree(*ad.first, gvd.g) >= 3) {
          ogrid.cell(current_Pos.x,current_Pos.y) = Critical;
          criticals_count++;
          break;
        }
      }
    }
  }
  // No real critical point found, create an artificial one representing the hole space as one
  // segment
  if (criticals_count == 0) {
    int max = -1;
    Pos max_Pos;

    for (auto vp = vertices(gvd.g); vp.first != vp.second; ++vp.first) {
      current_Pos = gvd[*vp.first].p;
      int current_deg = out_degree(*vp.first, gvd.g);
      if (current_deg > max || max < 0) {
        max = current_deg;
        max_Pos = current_Pos;
      }
    }
    ogrid.cell(max_Pos.x,max_Pos.y) = Critical;
    criticals_count++;
  }
  return criticals_count;
}

CriticalInfos get_critical_points(StateGrid& stateGrid, DistMap& dg, GvdGraph& gvd) {
  cout << "debug :: get local mins" << endl;
  PosSet local_mins = getLocalMins(dg, gvd);

  cout << "debug :: clean_up" << endl;
  // TODO clean_up and collapse_vertices can be merged into one function
  cleanUp(gvd);

  cout << "debug :: copy graph" << endl;
  GvdGraph graphGvdCopy(gvd);
  /* cout << "debug :: after copy" << endl; */

  cout << "debug :: collapse_vertices" << endl;
  collapseVertices(graphGvdCopy, local_mins);

  /* cout << "debug :: copycopy" << endl; */
  /* GvdGraph graphGvdCopyCopy(graphGvdCopy); */
  /* cout << "debug :: aftercopy" << endl; */
  // TODO borrar el criticals count no se usa
  // Quick Fix, pass the local_mins to the degree constraint, this should be mark on the gvd so
  // there is no need to passed the map of local mins
  cout << "debug :: degree_constraint" << endl;
  degree_constraint(stateGrid, graphGvdCopy, local_mins);
  cout << "debug :: unknown_dist_constraint2" << endl;
  CriticalInfos cis = unknownDistConstraint(stateGrid, gvd);
  // return critical_with_frontier;
  return cis;
}

TopoMap::TopoMap(Gvd* gvd){
  this->gvd = gvd;
  this->distMap = gvd->distMap;
}

TopoMap::TopoMap(pair<Int, Int> size){
  this->distMap = new DistMap(size,{Occupied},{Occupied,Unknown});
  this->gvd = new Gvd(distMap);
}

/* boost::tuple<criticals_info, GvdGraph> get_points_of_interest(StateGrid ogrid) { */
void TopoMap::update(StateGrid& sg){
  this->gvd->update(sg); // also updates the shared distMap
  cout << "debug :: Calculate ciritical points" << endl;
  this->cis = get_critical_points(sg, *distMap, *gvd->graphGvd);
}

TopoMap::~TopoMap(){
  delete distMap;
  if(gvd)
    gvd->distMap = NULL;
  delete gvd;
}
