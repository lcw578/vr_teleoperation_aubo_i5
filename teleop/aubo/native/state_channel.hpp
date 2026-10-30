#pragma once
#include <cmath>
#include <mutex>
#include <stdexcept>

// Shared observation only. Never hold this mutex across vendor I/O.
struct StateObservation {
 int state=-1,return_code=-1;
 double started=0,received=0;
 unsigned long long generation=0,sequence=0;
 bool valid=false;
};
class StateObservationCache {
 mutable std::mutex mutex_;
 StateObservation value_;
 public:
 unsigned long long invalidate(){
  std::lock_guard<std::mutex>lock(mutex_);
  ++value_.generation;value_.valid=false;return value_.generation;
 }
 StateObservation read()const{
  std::lock_guard<std::mutex>lock(mutex_);return value_;
 }
 // A result from an invalidated connection may never revive this cache.
 bool publish(int state,int rc,double started,double received,unsigned long long generation){
  std::lock_guard<std::mutex>lock(mutex_);
  if(generation!=value_.generation)return false;
  value_.state=state;value_.return_code=rc;value_.started=started;value_.received=received;
  ++value_.sequence;
  // Known observation does not mean permission to move (send checks 0/1).
  value_.valid=rc==0&&state>=0&&state<=3&&std::isfinite(started)&&
      std::isfinite(received)&&started<=received&&received-started<=.5;
  return true;
 }
};

inline bool state_observation_recent(const StateObservation&v,double now){
 return v.valid&&std::isfinite(now)&&now>=v.received&&now-v.received<=.2;
}
