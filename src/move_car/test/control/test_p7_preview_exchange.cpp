#include "control/p7_preview_exchange.h"

#include <cassert>
#include <future>
#include <iostream>

int main() {
    using unified_control::PreviewContext;
    using namespace std::chrono;
    const auto now=PreviewContext::Clock::now();
    PreviewContext candidate{now-milliseconds(32),1,3,7,10,false};
    const PreviewContext live{now,1,3,8,11,false};
    assert(unified_control::previewCanRoll(candidate,live,1./30.));
    auto reject=[&]{assert(!unified_control::previewCanRoll(candidate,live,1./30.));};
    candidate.history=8; reject(); candidate.history=6; reject(); candidate.history=7;
    candidate.config=4; reject(); candidate.config=3;
    candidate.epoch=2; reject(); candidate.epoch=1;
    candidate.use_raw=true; reject(); candidate.use_raw=false;
    candidate.reference=12; reject(); candidate.reference=10;
    candidate.measured=now+milliseconds(1); reject();
    candidate.measured=now-milliseconds(68); reject();

    // Keep the worker occupied while the callback submits another request.
    // The callback returns without waiting and the newest pending job wins.
    unified_control::PreviewExchange<int,int> exchange;
    std::promise<void> entered, release;
    auto entered_future=entered.get_future();
    auto permission=release.get_future();
    exchange.start([&](int job){
        if(job==1){entered.set_value(); permission.wait();}
        return job;
    });
    int result=0;
    exchange.tryExchange(1,result);
    assert(entered_future.wait_for(seconds(1))==std::future_status::ready);
    exchange.tryExchange(2,result);
    exchange.tryExchange(3,result);
    assert(result==0);
    release.set_value();
    const auto until=PreviewContext::Clock::now()+seconds(1);
    while(result!=3 && PreviewContext::Clock::now()<until) {
        std::this_thread::sleep_for(milliseconds(1));
        exchange.tryExchange(3,result);
        assert(result!=2);
    }
    assert(result==3);
    std::cout<<"preview expiry, causal lineage and nonblocking exchange passed\n";
}
