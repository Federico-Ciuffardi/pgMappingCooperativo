#include "TopoMap.h"
#include <vector>
#include "Gvd.h"
#include "utils.h"

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

bool sameDirecction(Pos p1, Pos p2) {
  return p1.normalize() == -p2.normalize();
}

void collapseVertices(GvdGraph& gvd, PosSet lmins) {
  for (GvdGraph::VertexIterator vp = gvd.begin(); vp != gvd.end();) {
    GvdGraph::VertexIterator vpAux = vp++;

    Pos current_Pos = gvd[*vpAux].p;

    bool is_min = is_elem(current_Pos,lmins);
    if (!is_min && gvd.degree(*vpAux) == 2) {
      vector<GvdGraph::Vertex> adj = gvd.adj(*vpAux);//adjacent_vertices(*vpAux, gvd.g);

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
  DistMap distMap(ogrid.size(),{Critical},{Unknown,Occupied});

  /* DistPosQueue dqueue; */
  CriticalInfos res;
  cout << "calculate_distances" << endl;

  distMap.update(ogrid);
  /* boost::tie(dgrid, dqueue) = calculate_distances(ogrid, Critical);  // dqueue = frontier queue */
  cout << "set each frontier to a critical" << endl;
  while (!distMap.fullDQueue.empty()) {
    DistPos frontier_dp = distMap.fullDQueue.top();
    distMap.fullDQueue.pop();
    Pos frontier = frontier_dp.second;

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


void clean_up(GvdGraph& gvd, DistMap& dgrid, int min_deg) {
  GvdGraph::VertexIterator v_it, v_it_end;
  for (tie(v_it, v_it_end) = vertices(gvd.g); v_it != v_it_end;) {
    GvdGraph::VertexIterator v_it_aux = v_it;
    v_it++;

    GvdGraph::Vertex max_deg_v = *v_it_aux;
    int max_deg = out_degree(*v_it_aux, gvd.g);

    if (max_deg >= min_deg) {  // just for not recalculating out_degree(*v_it_aux, gvd.g) it does
                               // not have anything to do with the max deg itself
      GvdGraph::AdjacencyIterator av_it, av_it_end;
      for (boost::tie(av_it, av_it_end) = boost::adjacent_vertices(*v_it_aux, gvd.g);
           av_it != av_it_end; ++av_it) {
        int current_degree = out_degree(*av_it, gvd.g);
        if (current_degree >=
            max_deg) {  // >= max clean up, > just when there is just one max_degree
          max_deg_v = *av_it;
          max_deg = current_degree;
        }
      }

      if (max_deg_v == *v_it_aux)
        continue;

      for (tie(av_it, av_it_end) = adjacent_vertices(*v_it_aux, gvd.g); av_it != av_it_end;) {
        GvdGraph::AdjacencyIterator av_it_aux = av_it;
        ++av_it;
        if (max_deg_v == *av_it_aux)
          continue;
        if (edge(*av_it_aux, max_deg_v, gvd.g).second) {
          remove_edge(*av_it_aux, *v_it_aux, gvd.g);

          if (out_degree(*av_it_aux, gvd.g) == 1) {
            clear_vertex(*av_it_aux, gvd.g);
            break;
          } else {
            remove_edge(*v_it_aux, *av_it_aux, gvd.g);
            tie(av_it, av_it_end) = adjacent_vertices(*v_it_aux, gvd.g);
          }
        }
      }
    }
  }
  for (tie(v_it, v_it_end) = vertices(gvd.g); v_it != v_it_end;) {
    GvdGraph::VertexIterator v_it_aux = v_it;
    v_it++;
    if (out_degree(*v_it_aux, gvd.g) == 0) {
      remove_vertex(*v_it_aux, gvd.g);
    }
  }
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
  clean_up(gvd, dg, 3);
  clean_up(gvd, dg, 2);

  cout << "debug :: collapse_vertices" << endl;
  GvdGraph graphGvdCopy = gvd;
  collapseVertices(graphGvdCopy, local_mins);

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
