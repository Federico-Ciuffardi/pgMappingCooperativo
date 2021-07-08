#pragma once

#include <bits/stdc++.h>
#include <iostream>
#include <limits>

#include <boost/unordered_set.hpp>

#include "Vector2.h"

using namespace std;

#define NULL_POS Pos(numeric_limits<Pos::InNum>::max(), numeric_limits<Pos::InNum>::max())

/*
 *  position
 */
typedef Vector2 Pos;
typedef pair<float, Pos> DistPos;
typedef priority_queue<DistPos, vector<DistPos>, greater<DistPos>> DistPosQueue;

typedef boost::unordered_set<Pos> PosSet;

/*
 *  Pos implementation
 */
bool same_direction(Pos p1, Pos p2);
