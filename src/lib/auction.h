#ifndef AUCTION_H
#define AUCTION_H

#include <bits/stdc++.h>
#include "../lib/GVD/GVD.h"

using namespace std;

typedef pair<string,pos> assignment;
typedef pair<float, assignment> bid;
typedef priority_queue<bid, vector<bid>, greater<bid>> bids_priority_queue;

static void add_bid(bids_priority_queue &bids_pq,string bidder,pos item,float bidding){
  bid b(bidding,assignment(bidder,item));
  bids_pq.push(b);
}

static void clear_bids(bids_priority_queue &bids_pq){
  bids_pq = bids_priority_queue();
}

static map<string,pos> resolve_auction(bids_priority_queue bids_pq,int total_bidders,int total_items,std::map<pos,int>* item_capacity = NULL){

  std::map<string,pos> bidder_item;//std::unordered_map?
  std::map<pos,int> item_bidders_num;//std::unordered_map?

  int assigned_bidders = 0;
  int item_with_extra_bidder = 0;

  int bidders_per_item = total_bidders / total_items;
  int reminder_bidders = total_bidders % total_items;

  int max_bidder_per_item;
  if(reminder_bidders == 0){
    max_bidder_per_item = bidders_per_item;
  }else{
    max_bidder_per_item = bidders_per_item + 1;
  }

  while(assigned_bidders < total_bidders && !bids_pq.empty()){
    bid b = bids_pq.top(); bids_pq.pop();
    float value = b.first;
    string r_name = b.second.first; 
    pos item = b.second.second;
    
    //bidder no asignado, itemo no supera el maximo admitido de bidders y quedan fronteras disponibles
    if(bidder_item.find(r_name) == bidder_item.end() && item_bidders_num[item] < max_bidder_per_item && ( !item_capacity || (*item_capacity)[item] > 0)){
      bidder_item[r_name] =item;
      assigned_bidders++;
      item_bidders_num[item]++;
      if(item_capacity) (*item_capacity)[item]--;

      if(item_bidders_num[item]==bidders_per_item+1){
        item_with_extra_bidder++;
        if(item_with_extra_bidder == reminder_bidders){
          max_bidder_per_item = bidders_per_item;
        }
      }
    }else{
      continue;
    }
  }

  return bidder_item;
}

#endif