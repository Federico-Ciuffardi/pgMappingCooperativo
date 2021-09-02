#pragma once
////////////
// Config //
////////////

// singleton that stores the Gvd lib config
class GvdConfig {
public:
  // connectivityMethod: how connectivity is guaranteed
  // 0: unknown cell generate waves (suitable for indoor)
  // 1: unknown cells are considered unobstructed and map is surrounded with obstacles (suitable for indoor and outdoor)
  // 2: unknown cells are considered unobstructed (suitable for indoor)
  int connectivityMethod = 0;

  // GVD vertex simplification: how hard are the GVD vercices eroded/deleted to
  // simplify or thin the GVD:
  // 0: no simplification (only for debugging)
  // 1: A vertex is deleted if it is not needed for connectivity and it does not have
  //    two or more sources (basis points at minumun distance)
  // 2: A vertex is deleted if it is not needed for connectivity and it does not have
  //    two or more sources (basis points at minumun distance). 
  //    Some vertices are added just to keep the connectivity in the regions that are
  //    beeing expored [*], those vertices are deleted if they are not  needed for connectivity
  // 3: All vertices will be discarded if they can be safely discarded without
  //    disconnecting the GVD.
  // [*] Depends on the connectivityMethod used:
  //       0:  that have less than 2 obstacles as basis points  are auxiliar
  //       (are a products of unknown vertices)
  //       1 and 2: vertices that are unknown or are close to unknown
  int vertexSimplificationMethod = 1;


  // GVD edge simplification: how hard are the GVD edges deleted to simplify the GVD:
  // 0: no simplification
  // 1: edges to vertices that can be accessed through a neighbor of grater degree are deleted
  // 2: edges to vertices that can be accessed through a neighbor of grater or equal degree are deleted
  int edgeSimplificationMethod = 2;

  // Allow GVD edge simplification to remove vertex: if when removing edges the verte
  // 0: Do not allow vertex removal
  // 1: Allow vertex removal
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
