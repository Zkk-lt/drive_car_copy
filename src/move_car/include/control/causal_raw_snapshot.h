#pragma once
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace research_control {
// External shadow only: receipt-causal selection for a historical control stamp.
// Neither receipt nor message acquisition stamp is changed. Source-clock
// verification and freshness remain the state observer's responsibility.
template<class T,std::size_t Capacity=512> class CausalRawSnapshot {
 static_assert(Capacity>0,"raw snapshot capacity must be positive");
 struct Entry {T value{};double receipt=0.;std::uint64_t id=0;};
 std::array<Entry,Capacity> entries_{};
 std::size_t next_=0,count_=0;
 std::uint64_t serial_=0,last_fed_=0,dropped_=0,rejected_=0;
 double latest_receipt_=0.,fed_receipt_=0.;
public:
 bool push(const T& value,double receipt) {
   // A receipt-clock reversal cannot establish a new causal history. Retain
   // the old evidence until normal control-time/reset handling recovers.
   if(!std::isfinite(receipt)||receipt<=0.||receipt<latest_receipt_){++rejected_;return false;}
   if(count_==Capacity)++dropped_;else ++count_;
   auto& entry=entries_[next_];entry.value=value;entry.receipt=receipt;entry.id=++serial_;
   next_=(next_+1)%Capacity;latest_receipt_=receipt;return true;
 }
 template<class Feed> bool feed(double control_stamp,Feed callback) {
   if(!std::isfinite(control_stamp)||control_stamp<=0.)return false;
   const Entry* selected=nullptr;
   for(std::size_t i=0;i<count_;++i) {
     const auto& entry=entries_[i];
     if(entry.receipt<=control_stamp&&(!selected||entry.receipt>selected->receipt||
        (entry.receipt==selected->receipt&&entry.id>selected->id)))selected=&entry;
   }
   if(!selected||selected->id==last_fed_)return false;
   callback(selected->value,selected->receipt);
   last_fed_=selected->id;fed_receipt_=selected->receipt;return true;
 }
 // Observer resets must allow the same still-causal sample to be fed again.
 // Keep original records so a reset cannot invent a current receipt timestamp.
 void resetFed(){last_fed_=0;fed_receipt_=0.;}
 std::size_t size()const{return count_;}
 std::uint64_t overwritten()const{return dropped_;}
 std::uint64_t rejected()const{return rejected_;}
 double lastFedReceipt()const{return fed_receipt_;}
};
}
