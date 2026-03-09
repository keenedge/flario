#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

template <size_t HistoryCap = 64>
class FrequencyReporter
{
public:
    struct Snapshot
    {
        float measured_hz = 0.0f;
        float expected_hz = 0.0f;
        float min_hz = 0.0f;
        bool below_threshold = false;
    };

    FrequencyReporter(const char *name,
                      std::atomic<uint32_t> &counter,
                      float expected_hz,
                      float min_hz,
                      int64_t window_us = 5LL * 1000LL * 1000LL,
                      int64_t sample_period_us = 100000LL)
        : name_(name),
          counter_(&counter),
          expected_hz_(expected_hz),
          min_hz_(min_hz),
          window_us_(window_us),
          sample_period_us_(sample_period_us)
    {
    }

    void reset(int64_t now_us)
    {
        head_ = 0U;
        len_ = 0U;
        last_history_sample_us_ = now_us;
        push_sample(now_us, counter_->load(std::memory_order_relaxed));
    }

    Snapshot update(int64_t now_us)
    {
        if ((len_ == 0U) || ((now_us - last_history_sample_us_) >= sample_period_us_))
        {
            push_sample(now_us, counter_->load(std::memory_order_relaxed));
            last_history_sample_us_ = now_us;
        }

        Snapshot out{};
        out.expected_hz = expected_hz_;
        out.min_hz = min_hz_;

        if (len_ >= 2U)
        {
            const size_t oldest_idx = head_;
            const size_t newest_idx = (head_ + len_ - 1U) % HistoryCap;
            const Sample &oldest = history_[oldest_idx];
            const Sample &newest = history_[newest_idx];
            const int64_t dt_us = newest.t_us - oldest.t_us;
            const uint32_t delta = newest.count - oldest.count;

            if (dt_us > 0)
            {
                out.measured_hz = (static_cast<float>(delta) * 1000000.0f) / static_cast<float>(dt_us);
            }
        }

        out.below_threshold = (out.measured_hz < min_hz_);
        return out;
    }

    const char *name() const { return name_; }
    float expected_hz() const { return expected_hz_; }
    float min_hz() const { return min_hz_; }

private:
    struct Sample
    {
        int64_t t_us = 0;
        uint32_t count = 0;
    };

    void push_sample(int64_t t_us, uint32_t count)
    {
        size_t write_idx = 0U;
        if (len_ < HistoryCap)
        {
            write_idx = (head_ + len_) % HistoryCap;
            len_++;
        }
        else
        {
            write_idx = head_;
            head_ = (head_ + 1U) % HistoryCap;
        }

        history_[write_idx] = {t_us, count};

        while (len_ > 1U)
        {
            const size_t oldest_idx = head_;
            if ((t_us - history_[oldest_idx].t_us) <= window_us_)
                break;

            head_ = (head_ + 1U) % HistoryCap;
            len_--;
        }
    }

    const char *name_;
    std::atomic<uint32_t> *counter_;
    float expected_hz_;
    float min_hz_;
    int64_t window_us_;
    int64_t sample_period_us_;
    int64_t last_history_sample_us_ = 0;
    std::array<Sample, HistoryCap> history_{};
    size_t head_ = 0U;
    size_t len_ = 0U;
};
