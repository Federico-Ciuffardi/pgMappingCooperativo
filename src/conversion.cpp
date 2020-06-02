#include "GVD/GVD.h"
#include <tscf_exploration/Point2D.h>
#include <tscf_exploration/SegmentAuction.h>
#include <tscf_exploration/SegmentBid.h>

static tscf_exploration::Point2D p_to_p2d(pos p){
  tscf_exploration::Point2D p2d;
  p2d.x = p.first;
  p2d.y = p.second; 
  return p2d;
}

static pos p2d_to_p(tscf_exploration::Point2D p2d){
  pos p;
  p.first = p2d.x; 
  p.second = p2d.y; 
  return p;
}