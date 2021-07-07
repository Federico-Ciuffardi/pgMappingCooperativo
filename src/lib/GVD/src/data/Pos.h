#pragma once

#include <bits/stdc++.h>
#include <iostream>

#include <boost/unordered_set.hpp>

#include "Vector2.h"

using namespace std;

#define NULL_POS Pos(MAXFLOAT, MAXFLOAT)

/*
 *  position
 */
typedef Vector2 Pos;
typedef pair<float, Pos> dist_pos;
typedef priority_queue<dist_pos, vector<dist_pos>, greater<dist_pos>> dist_pos_queue;

/*
 *  Pos implementation
 */
bool same_direction(Pos p1, Pos p2);

float angle(Pos p);
