#pragma once

#include "utils.h"
#include "Map.h"
#include "data/Num.h"

struct DistMap{

  /////////////
  // Typedef //
  /////////////

  typedef CellState      CellType; // could be set as template if needed
  typedef Grid<CellType> MapType; // could be set as template if needed

  struct DistCell {
    PosSet sources;
    PosSet pseudoSources;
    Float distance = INF;

    bool toRaise = false;
    bool isCleared = true;

    bool operator>(const DistCell& d) const;
    bool operator==(const DistCell& p) const;

    void clear();

    friend ostream& operator<<(ostream& out, const DistCell& cell);
  };

  typedef Grid<DistCell> DistMapType;

  ///////////////
  // Variables //
  ///////////////

  // Results
  DistMapType distMap;
  DistPosQueue open; 

  PosSet waveCrashes;
  PosSet modified;

  // Info
  MapType& map;

  // Configuration
  vector<CellType> nonTraversables;
  vector<CellType> sources;

  ///////////////
  // functions //
  ///////////////

  pair<Int,Int> size();
  DistMapType::reference operator[](Pos);

  void update();
  void update(MapUpdatedCells &mapUpdatedCells);

  PosSet basisPoints(Pos p);
  bool isWaveCrash(Pos p);

  // Constructor
  DistMap(MapType&, vector<CellType> sources, vector<CellType> nonTraversables, vector<CellType> objectives = {});

  friend ostream& operator<<(ostream& out, const DistMap&);

 private:
  ///////////////
  // functions //
  ///////////////

  void setSource(Pos p);
  void removeSource(Pos p);
  bool hasBasisPoint(Pos p);
  void processLower(Pos s);
  void processRaise(Pos p);
  void setConsistentBorders(Pos p);

  void setPseudoSourcesFromWave(Pos p, Pos waveP);
  void processWaveCrash(Pos p, Pos waveP);

  void updateBase();
};

