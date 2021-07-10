#pragma once

#include <bits/stdc++.h>
#include "../lib/GVD/src/Gvd.h"

using namespace std;

typedef pair<string,Pos> assignment;
typedef pair<float, assignment> bid;
typedef priority_queue<bid, vector<bid>, greater<bid>> bids_priority_queue;

static void add_bid(bids_priority_queue &bids_pq,string bidder,Pos item,float bidding){
  bid b(bidding,assignment(bidder,item));
  bids_pq.push(b);
}

static void clear_bids(bids_priority_queue &bids_pq){
  bids_pq = bids_priority_queue();
}

static boost::unordered_map<string,Pos> resolve_auction(bids_priority_queue bids_pq,int total_bidders,int total_items,boost::unordered_map<Pos,int>* item_capacity = NULL){

  bids_priority_queue original_bids_pq = bids_pq; 

  boost::unordered_map<string,Pos> bidder_item;//std::unordered_map?
  boost::unordered_map<Pos,int> item_bidders_num;//std::unordered_map?

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
    Pos item = b.second.second;
    
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

  if(item_capacity && assigned_bidders < total_bidders){
    int total_item_capacity = 0;
    int depleated_items = 0; 
    for(auto it = (*item_capacity).begin(); it!= (*item_capacity).end(); it++){
      if(it->second > 0){
        total_item_capacity += it->second;
      }else{
        depleated_items++;
      }
    }
    if(total_item_capacity>0){
      bids_pq = bids_priority_queue();
      while(!original_bids_pq.empty()){
        bid b = original_bids_pq.top(); original_bids_pq.pop();
        string r_name = b.second.first; 
        if(bidder_item.find(r_name) == bidder_item.end()){
          bids_pq.push(b);
        }
      }
      boost::unordered_map<string,Pos> sub_bidder_item = resolve_auction(bids_pq,total_bidders-assigned_bidders,total_items-depleated_items,item_capacity);
      for(auto it = sub_bidder_item.begin(); it != sub_bidder_item.end(); it++){
        bidder_item[it->first] = it->second;
      }
    }

  }

  return bidder_item;
}
