#pragma once

#include <condition_variable>
#include <chrono>
#include <deque>
#include <exception>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

namespace nuwax_lio {

// 单个有界写出队列，算法锁内只排队；扫描点云在末态写出后再发布。
template<class Message>
class OrderedOdometryOutput {
public:
    explicit OrderedOdometryOutput(std::function<void(const Message&)> publish,
                                   std::size_t capacity = 8192)
        : publish_(std::move(publish)), capacity_(capacity), worker_([this] { run(); }) {}
    ~OrderedOdometryOutput() { close(); }
    OrderedOdometryOutput(const OrderedOdometryOutput&) = delete;
    OrderedOdometryOutput& operator=(const OrderedOdometryOutput&) = delete;

    void submit(Message message) {
        std::lock_guard<std::mutex> lock(mutex_);
        check_error();
        if (closing_ || queue_.size() >= capacity_)
            throw std::runtime_error("逐点里程计输出队列关闭或积压超限");
        queue_.push_back(std::move(message));
        changed_.notify_all();
    }

    void flush() {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!changed_.wait_for(lock, std::chrono::seconds(2), [this] {
                return error_ || (queue_.empty() && !publishing_); }))
            throw std::runtime_error("逐点里程计输出等待超时");
        check_error();
    }

    void close() noexcept {
        { std::lock_guard<std::mutex> lock(mutex_); closing_ = true; changed_.notify_all(); }
        if (worker_.joinable()) worker_.join();
    }

private:
    void check_error() const { if (error_) std::rethrow_exception(error_); }
    void run() noexcept {
        for (;;) {
            std::unique_lock<std::mutex> lock(mutex_);
            changed_.wait(lock, [this] { return closing_ || !queue_.empty(); });
            if (queue_.empty()) return;
            Message message = std::move(queue_.front()); queue_.pop_front(); publishing_ = true;
            lock.unlock();
            try { publish_(message); }
            catch (...) {
                lock.lock(); error_ = std::current_exception(); publishing_ = false;
                closing_ = true; queue_.clear(); changed_.notify_all(); return;
            }
            lock.lock(); publishing_ = false; changed_.notify_all();
        }
    }
    std::function<void(const Message&)> publish_;
    std::size_t capacity_;
    std::mutex mutex_;
    std::condition_variable changed_;
    std::deque<Message> queue_;
    bool closing_ = false;
    bool publishing_ = false;
    std::exception_ptr error_;
    std::thread worker_;
};

}  // 命名空间nuwax_lio
