#pragma once

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <utility>

namespace jetson_rgbd_udp_sender
{

template<typename T>
class LatestSlot
{
public:
  struct PutResult
  {
    bool accepted{false};
    bool overwrote{false};
    std::uint64_t total_overwrites{0};
  };

  PutResult put(T value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {
      return PutResult{false, false, overwrite_count_};
    }

    const bool overwrote = value_.has_value();
    if (overwrote) {
      ++overwrite_count_;
    }
    value_ = std::move(value);
    condition_.notify_one();
    return PutResult{true, overwrote, overwrite_count_};
  }

  std::optional<T> take()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!value_) {
      return std::nullopt;
    }
    std::optional<T> result(std::move(value_));
    value_.reset();
    return result;
  }

  std::optional<T> wait_take()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this]() {return closed_ || value_.has_value();});
    if (!value_) {
      return std::nullopt;
    }
    std::optional<T> result(std::move(value_));
    value_.reset();
    return result;
  }

  void close()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = true;
    condition_.notify_all();
  }

  bool closed() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return closed_;
  }

  std::uint64_t overwrite_count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return overwrite_count_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::optional<T> value_;
  bool closed_{false};
  std::uint64_t overwrite_count_{0};
};

}  // namespace jetson_rgbd_udp_sender
