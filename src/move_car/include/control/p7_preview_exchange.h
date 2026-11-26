#pragma once

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>
#include <utility>

namespace unified_control {

// A preview is a warm sequence only. It has no command or publication handle.
struct PreviewContext {
    using Clock = std::chrono::steady_clock;
    Clock::time_point measured{};
    std::uint64_t epoch=0, config=0, history=0, reference=0;
    bool use_raw=false;
};

inline bool previewCanRoll(const PreviewContext& prepared, const PreviewContext& current,
                           double period) {
    return prepared.epoch==current.epoch && prepared.config==current.config &&
        current.history>0 && prepared.history==current.history-1 &&
        prepared.reference>0 && prepared.reference<=current.reference &&
        prepared.use_raw==current.use_raw && prepared.measured<=current.measured &&
        period>0. && std::chrono::duration<double>(current.measured-prepared.measured).count()<=2.*period;
}

template<class Job, class Result>
class PreviewExchange {
    std::mutex mutex_;
    std::condition_variable available_;
    Job pending_{};
    Result latest_{};
    bool job_ready_=false, result_ready_=false, shutdown_=false;
    std::thread worker_;
public:
    ~PreviewExchange() {
        if (!worker_.joinable()) return;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            shutdown_=true;
        }
        available_.notify_one();
        worker_.join();
    }

    template<class Solve>
    void start(Solve solve) {
        worker_=std::thread([this, solve=std::move(solve)]() mutable {
            for (;;) {
                Job job;
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    available_.wait(lock,[this]{return shutdown_||job_ready_;});
                    if (shutdown_) return;
                    job=pending_;
                    job_ready_=false;
                }
                Result result=solve(job);
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    latest_=result;
                    result_ready_=true;
                }
            }
        });
    }

    void tryExchange(const Job& job, Result& result) {
        std::unique_lock<std::mutex> lock(mutex_,std::try_to_lock);
        if (!lock.owns_lock()) return;
        if (result_ready_) {
            result=latest_;
            result_ready_=false;
        }
        pending_=job;
        job_ready_=true;
        lock.unlock();
        available_.notify_one();
    }
};
}
