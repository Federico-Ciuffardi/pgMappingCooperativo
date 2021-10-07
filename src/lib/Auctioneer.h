#pragma once

#include <bits/stdc++.h>
#include <boost/unordered/unordered_map_fwd.hpp>
#include <boost/unordered/unordered_set_fwd.hpp>
#include "../lib/GVD/src/Gvd.h"

using namespace std;

template<typename bidder_id, typename item_id, typename sub_item_id>
class Auctioneer{ 
 public:
  typedef bidder_id BidderId;
  typedef sub_item_id SubItemId;
  typedef item_id ItemId;

  struct Bid{
    BidderId bidderId;
    SubItemId subItemId;
    ItemId itemId;
    Float value;
    
    bool operator> (const Bid& b) const {
        return (value > b.value) || (value == b.value && bidderId > b.bidderId);
    }

    Bid(BidderId bidderId, ItemId itemId, SubItemId subItemId, Float value){
      this->bidderId  = bidderId;
      this->subItemId = subItemId;
      this->itemId    = itemId;
      this->value     = value;
    }
  };

 private:
  typedef priority_queue<Bid, vector<Bid>, greater<Bid>> BidPriorityQueue;
  typedef priority_queue<Int, vector<Int>, greater<Int>> IntPriorityQueue;

  BidPriorityQueue bidPriorityQueue;
  boost::unordered_set<string> remainingBidders;
  boost::unordered_map<ItemId,boost::unordered_set<SubItemId>> remainingItemSubItems;

 public:
  void addBid(BidderId bidderId, ItemId itemId, SubItemId subItemId, Float value){
    remainingBidders.insert(bidderId);
    bidPriorityQueue.push(Bid(bidderId, itemId, subItemId, value));
    remainingItemSubItems[itemId].insert(subItemId);
  }

  // Returns the assignments (bidders -> subItems) corresponding to the closest distribution to the uniform distribution that satisfies the restriction:
  //   (I) The number of bidders in a item must not exceed the number of subItems available in the item 
  //
  // Being n the number of bidders assigned to each item in the uniform distribution (without the constrain (I)) there might be items with less subItems 
  // than n, so the solution assumes that all those subItems will be assigned and then n is recalculated considering that. When all the items have a 
  // number of subItems grater than n, that n is used as a limit for the items that have a number of subItems greater than n.
  //
  // So the solution to the original problem is a distribution that complies with the following restrictions:
  //   (I)  The previous one
  //   (II) The number of bidders in a item must not exceed the number n.
  //
  // Resets all the information regarding the action to resolve including the parameter remainingItemSubItems

  boost::unordered_map<BidderId, SubItemId> resolveAuction(){
    // PART 1: calculate n

    /// Construct itemCapacityPQ (PQ of the subItem number of each item in ascending order)
    IntPriorityQueue itemCapacityPQ;

    for (auto it : remainingItemSubItems){
      boost::unordered_set<SubItemId> subItems = it.second;

      itemCapacityPQ.push(subItems.size());
    }
      
    /// Calculate the number n not to exceed on the restriction (II). This number actually depends on two two numbers, distributionQuotient and
    /// distribiutionRemainder, therefore these will be the ones to calculate. 
    int distributionQuotient   = -1; 
    int distribiutionRemainder = -1;
    
    int numItems   = remainingItemSubItems.size();
    int numBidders = remainingBidders.size();
    
    while(!itemCapacityPQ.empty()){
      distributionQuotient   = numBidders / numItems;
      distribiutionRemainder = numBidders % numItems; 
      
      int itemCapacity = itemCapacityPQ.top();
      itemCapacityPQ.pop(); 

      // Check if the current itemCapacity (and the rest because they are in a priority queue) have sufficient borders to be (II) the constrain that limits
      // the number of bidders on the item and not (I)
      //
      // If this is the case then it can be said that distributionQuotient and distribiutionRemainder converge to their final values and we can move
      // on to part 2  
      //
      // if(itemCapacity > distributionQuotient) || (distribiutionRemainder == 0 && itemCapacity == distributionQuotient){
      if ((itemCapacity + (distribiutionRemainder == 0) > distributionQuotient)){
        break;
      }
      
      // If it did not break, it is because the segment does not have enough subItems to be (II) the limiting constrain, therefore it can be
      // assumed that (I) will be the limiting constrain so all the subItems from the current item will be assigned. Therefore, numBidders and
      // numItems are recalculated taking that into account (the item is no longer counted in numItems and as many bidders as subItems are
      // subtracted from numBidders).
      numBidders -= itemCapacity; 
      numItems--;

      // distributionQuotient always grows becouse itemCapacity <= distributionQuotient
    }
      

    // PART 2: Assign robots taking into account constraints (1) and (2) 
    
    boost::unordered_map<ItemId,int> biddersOnItem; // Stores the number of bidders assigned to a subItem of the item

    boost::unordered_map<BidderId, SubItemId> assignments; // Stores the assigned robots to its assigned subItem

    /// while(bids remaining && bidders to assigned && items with subItems to assign){
    ///
    /// `remainingItemSubItems.empty()` corresponds to: are there items with remaining subItem to be assigned?.
    /// Meaning in remainingItemSubItems there are no idItems mapped to a empty subItemIds set.
    while (!remainingBidders.empty() && !remainingItemSubItems.empty() && !bidPriorityQueue.empty() ){ 
      Bid bid = bidPriorityQueue.top();
      bidPriorityQueue.pop();

      // DEBUG
      /* cout<<"----------------------"<<endl; */
      /* cout<<"bidderId:  "<<bid.bidderId<<endl; */
      /* cout<<"itemId:    "<<bid.itemId<<endl; */
      /* cout<<"subItemId: "<<bid.subItemId<<endl; */
      /* cout<<"value:     "<<bid.value<<endl; */
      /* cout<<"----------------------"<<endl; */
      /* cout<<"distributionQuotient:   "<<distributionQuotient<<endl; */
      /* cout<<"distribiutionRemainder: "<<distribiutionRemainder<<endl; */
      /* cout<<"remainingBidders        "<<remainingBidders<<endl; */
      /* cout<<"remainingItemSubItems:  "<<endl; */
      /* for( auto it : remainingItemSubItems){ */
      /*   cout<<it.first<<": "<<it.second<<endl; */
      /* } */
      
      // Ignore the bid if the bidder was already been assigned || the subItem has already been assigned 
      //
      // (I) is satisfied because the following if, according to it is not be possible to assign a robot to a segment if all its borders have
      // already been assigned
      if (!is_elem(bid.bidderId, remainingBidders) || !is_elem(bid.subItemId, remainingItemSubItems, bid.itemId )){
        continue;
      }
      
      // The following if checks whether the robot be assigned to the segment respecting (2)
      //
      // If there is a remainder (`distribiutionRemainder > 0`) then this consists of allowing to assign at most  `distributionQuotient + 1`
      // bidders, otherwise the maximum is `distributionQuotient`
      if( biddersOnItem[bid.itemId] < distributionQuotient + (distribiutionRemainder>0) ){
        // The assignment respects (2) and it already respected (1), then it can be made effective
        assignments[bid.bidderId] = bid.subItemId;
        biddersOnItem[bid.itemId]++;

        // Alter the data to account for the latest assignation
        /// Decrement the remainder if quotient + 1 bidders has been assinged to the item so it only happens remainder-1 times more
        if( biddersOnItem[bid.itemId] == distributionQuotient + 1 ){
          distribiutionRemainder--; 
        }
        /// the bidder is no longer available
        remainingBidders.erase(bid.bidderId); 
        /// the subItem is no longer available
        remainingItemSubItems[bid.itemId].erase(bid.subItemId);
        /// Do not let empty sets on the values of remainingItemSubItems
        if (remainingItemSubItems[bid.itemId].empty()){ 
          remainingItemSubItems.erase(bid.itemId); 
        }
      }
    }

    // DEBUG
    /* cout<<"Assignments: "<<endl; */
    /* cout<<assignments<<endl; */

    // Reset auction data
    bidPriorityQueue = BidPriorityQueue();
    remainingBidders.clear();
    remainingItemSubItems.clear(); 

    return assignments;
  }
};
