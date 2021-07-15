#pragma once
#include <boost/unordered/unordered_set.hpp>
#include <queue>

using namespace std;

template<typename priority, typename elem>
struct Queue{
  typedef priority Priority;
  typedef elem Elem;
  typedef pair<Priority,Elem> PrioElem;

  typedef set<PrioElem> QueueType;
  typedef typename QueueType::iterator Iterator;

  QueueType queue;

  void push(PrioElem e){
    queue.insert(e);
  }
  PrioElem pop(){
    Iterator it = queue.begin();
    PrioElem e = *it;
    queue.erase(it);
    return e;
  }
  bool empty(){
    return queue.empty();
  }
}
