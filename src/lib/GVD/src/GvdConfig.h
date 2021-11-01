#pragma once
////////////
// Config //
////////////

// singleton that stores the Gvd lib config
class GvdConfig {
public:
  // connectivityMethod: how connectivity is guaranteed
  // 0: connectivity is not guaranteed
  // 1: unknown cell generate waves (map borders generate waves)
  // 2: unknown cells are considered unobstructed (map borders generate waves)
  // 3: unknown cells are considered unobstructed (map borders DO NOT generate waves)
  int connectivityMethod = 1;

  // GVD vertex simplification: how hard are the GVD vercices eroded/deleted to
  // simplify or thin the GVD:
  // 0: no simplification (only for debugging)
  // 1: A vertex is deleted if it is not needed for connectivity and it does not have
  //    two or more non adjacent sources
  int vertexSimplificationMethod = 1;


  // GVD edge simplification: how hard are the GVD edges deleted to simplify the GVD:
  // 0: no simplification
  // 1: edges to vertices that can be accessed through a neighbor of grater degree are deleted
  // 2: edges to vertices that can be accessed through a neighbor of grater or equal degree are deleted
  int edgeSimplificationMethod = 2;

  // Allow GVD edge simplification to remove vertex when the edge simplification leaves the vertex with only onew edge
  // 0: Do not allow vertex removal
  // 1: Remove the vetex if the vertex also has 3 or more adjacent cells that belong to the GVD 
  // 2: Remove the vertex if the vertex also has 2 or more adjacent cells that belong to the GVD 
  // 2: Always remove the vertex
  int edgeSimplificationAllowVertexRemoval = 1;

  // Set what is the min codition 
  // 0: - All neighbors has greater distance
  // 1: - There is no neighbor with less distance 
  //    - And there is at least one neighbor with greater distance
  int criticalConditionMin = 1;

private:
  GvdConfig();

public:
    // get instance
    static GvdConfig* get();
};
