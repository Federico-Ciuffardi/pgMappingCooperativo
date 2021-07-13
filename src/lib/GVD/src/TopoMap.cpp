#include "TopoMap.h"
#include "Gvd.h"

boost::unordered_map<Pos, bool> get_local_mins(DistMap& dg, GvdGraph& gvd) {
  boost::unordered_map<Pos, bool> lmins;

  for (auto vp = vertices(gvd.g); vp.first != vp.second; ++vp.first) {
    bool is_min_e = true;
    bool has_greater = false;
    Pos current_Pos = gvd[*vp.first].p;

    if (is_elem(current_Pos,lmins)) continue; // already processed

    for (Pos adj_Pos : gvd.adj(current_Pos)) {
      bool auxmin = dg[current_Pos].distance <= dg[adj_Pos].distance;
      bool adj_greater = dg[current_Pos].distance < dg[adj_Pos].distance;

      if (adj_greater) {
        has_greater = true;
        lmins[adj_Pos] = false;
      }
      is_min_e = is_min_e && auxmin;
      if (!is_min_e)
        break;
    }
    lmins[current_Pos] = is_min_e && has_greater;
  }
  return lmins;
}

bool same_direcction(Pos p1, Pos p2) {
  return p1.normalize() == -p2.normalize();
}

void collapse_vertices(GvdGraph& gvd, boost::unordered_map<Pos, bool> lmins) {
  for (auto vp = vertices(gvd.g); vp.first != vp.second;) {
    auto vp_aux = vp.first;
    ++vp.first;
    Pos current_Pos = gvd[*vp_aux].p;

    bool is_min = lmins[current_Pos];
    if (!is_min && out_degree(*vp_aux, gvd.g) == 2) {
      auto adj = adjacent_vertices(*vp_aux, gvd.g);

      auto adj1 = gvd[*adj.first];
      Pos adj1_aux = adj1.p - current_Pos;
      auto adj_aux = adj;

      ++adj.first;
      auto adj2 = gvd[*adj.first];
      Pos adj2_aux = (adj2.p - current_Pos);

      if (same_direcction(adj1_aux, adj2_aux)) {
        gvd.addE(*adj_aux.first, *adj.first);
        gvd.addE(*adj.first, *adj_aux.first);
        clear_vertex(*vp_aux, gvd.g);
        remove_vertex(*vp_aux, gvd.g);
      }
    }
  }
}

// Returns two maps : criticals -> frontiers, criticals-> min dis to frontier, segments gvd
criticals_info unknown_dist_constraint2(StateGrid& ogrid, GvdGraph& gvd) {
  DistMap distMap(ogrid.size(),{Critical},{Unknown,Occupied});
  /* DistPosQueue dqueue; */
  criticals_info res;
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
        res[critical_Pos].mind_f = frontier_dp.first;
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


int degree_constraint(StateGrid& ogrid, GvdGraph& gvd, boost::unordered_map<Pos, bool> local_mins) {
  int criticals_count = 0;
  Pos current_Pos;
  for (auto vp = vertices(gvd.g); vp.first != vp.second; ++vp.first) {
    current_Pos = gvd[*vp.first].p;
    if (out_degree(*vp.first, gvd.g) == 2 && local_mins[current_Pos]) {
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

criticals_info get_critical_points(StateGrid& stateGrid, DistMap& dg, GvdGraph& gvd) {
  boost::unordered_map<Pos, bool> local_mins = get_local_mins(dg, gvd);
  cout << "debug :: clean_up" << endl;
  // TODO clean_up and collapse_vertices can be merged into one function
  clean_up(gvd, dg, 3);
  clean_up(gvd, dg, 2);
  GvdGraph graphGvdCopy = gvd;
  cout << "debug :: collapse_vertices" << endl;
  collapse_vertices(graphGvdCopy, local_mins);

  // TODO borrar el criticals count no se usa
  // Quick Fix, pass the local_mins to the degree constraint, this should be mark on the gvd so
  // there is no need to passed the map of local mins
  cout << "debug :: degree_constraint" << endl;
  degree_constraint(stateGrid, graphGvdCopy, local_mins);
  cout << "debug :: unknown_dist_constraint2" << endl;
  criticals_info cis = unknown_dist_constraint2(stateGrid, gvd);
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
