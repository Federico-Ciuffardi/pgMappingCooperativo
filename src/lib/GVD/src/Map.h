#pragma once

#include "data/Grid.h"

/* template<typename T> */
/* struct Map{ */
/*   typedef T CellType; */

/* }; */

enum CellState { Occupied, Unknown, Free, Critical, Frontier, CriticalLine };
typedef Grid<CellState> Map;
typedef boost::unordered_map<Pos,Map::CellType> MapUpdatedCells;

/* typedef Map<CellState> StateMap; */
CellState toOccupancyState(CellState cs);

ostream& operator<<(ostream& out, const CellState cs);

void updateMap(boost::unordered_map<Pos,CellState> &updatedCells,Map &map, Pos p, CellState newCellState);
