#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>
#include <mutex>
#include <unordered_map>
#include <deque>
#include <condition_variable>

namespace rvpoint {

enum class IngressPolicy {
    Block,
    DropOldest
};

/**
 * @brief Thread-safe re-order buffer ensuring monotonic frame egress.
 *
 * In FrameWorkerPool execution, variable-duration frames may complete out of order.
 * StreamResequencer buffers completions and emits them in strictly monotonic sequence_id order.
 */
template <typename T>
class StreamResequencer {
public:
    using Callback = std::function<void(uint64_t seq_id, T item)>;

    explicit StreamResequencer(Callback on_emit)
        : on_emit_(std::move(on_emit)), next_expected_seq_(0) {}

    void push(uint64_t seq_id, T item) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (seq_id == next_expected_seq_) {
            on_emit_(seq_id, std::move(item));
            next_expected_seq_++;

            while (true) {
                auto it = buffer_.find(next_expected_seq_);
                if (it == buffer_.end()) break;
                on_emit_(it->first, std::move(it->second));
                buffer_.erase(it);
                next_expected_seq_++;
            }
        } else if (seq_id > next_expected_seq_) {
            buffer_.emplace(seq_id, std::move(item));
        }
        // If seq_id < next_expected_seq_, it is stale/dropped; ignore
    }

    uint64_t next_expected_seq() const noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        return next_expected_seq_;
    }

    size_t pending_count() const noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        return buffer_.size();
    }

    void reset(uint64_t start_seq = 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_.clear();
        next_expected_seq_ = start_seq;
    }

private:
    mutable std::mutex mutex_;
    Callback on_emit_;
    uint64_t next_expected_seq_ = 0;
    std::unordered_map<uint64_t, T> buffer_;
};

/**
 * @brief Thread-safe frame queue with Block and DropOldest ingress policies.
 */
template <typename T>
class FrameQueue {
public:
    explicit FrameQueue(size_t max_capacity = 8, IngressPolicy policy = IngressPolicy::Block)
        : max_capacity_(max_capacity), policy_(policy), stopped_(false), dropped_count_(0) {}

    bool push(T item) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (stopped_) return false;

        if (queue_.size() >= max_capacity_) {
            if (policy_ == IngressPolicy::DropOldest) {
                queue_.pop_front();
                dropped_count_++;
            } else {
                cv_produce_.wait(lock, [this] { return queue_.size() < max_capacity_ || stopped_; });
                if (stopped_) return false;
            }
        }

        queue_.push_back(std::move(item));
        cv_consume_.notify_one();
        return true;
    }

    bool pop(T& out_item) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_consume_.wait(lock, [this] { return !queue_.empty() || stopped_; });
        if (queue_.empty() && stopped_) return false;

        out_item = std::move(queue_.front());
        queue_.pop_front();
        cv_produce_.notify_one();
        return true;
    }

    void stop() {
        std::lock_guard<std::mutex> lock(mutex_);
        stopped_ = true;
        cv_produce_.notify_all();
        cv_consume_.notify_all();
    }

    size_t dropped_count() const noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        return dropped_count_;
    }

    size_t size() const noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

private:
    size_t max_capacity_;
    IngressPolicy policy_;
    bool stopped_;
    size_t dropped_count_;
    mutable std::mutex mutex_;
    std::condition_variable cv_produce_;
    std::condition_variable cv_consume_;
    std::deque<T> queue_;
};

} // namespace rvpoint

