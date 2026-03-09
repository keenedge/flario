#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <utility>
#include "esp_timer.h"
#include "frequency_reporter.hpp"

template <size_t N, size_t HistoryCap = 64>
class ReportFrequencyDiag
{
public:
    struct Config
    {
        const char *name = "";
        std::atomic<uint32_t> *counter = nullptr;
        float expected_hz = 0.0f;
        float min_hz = 0.0f;
    };

    struct Result
    {
        std::array<float, N> measured_hz{};
        std::array<bool, N> below_threshold{};
        bool any_below = false;
    };

    ReportFrequencyDiag(const std::array<Config, N> &configs,
                        int64_t window_us = 5LL * 1000LL * 1000LL,
                        int64_t sample_period_us = 100000LL)
        : reporters_(make_reporters(configs, window_us, sample_period_us))
    {
    }

    void reset()
    {
        const int64_t now_us = esp_timer_get_time();
        for (auto &reporter : reporters_)
        {
            reporter.reset(now_us);
        }
    }

    Result update()
    {
        const int64_t now_us = esp_timer_get_time();
        Result result{};

        for (size_t i = 0; i < N; ++i)
        {
            const typename FrequencyReporter<HistoryCap>::Snapshot snap = reporters_[i].update(now_us);
            result.measured_hz[i] = snap.measured_hz;
            result.below_threshold[i] = snap.below_threshold;
            result.any_below |= snap.below_threshold;
        }

        return result;
    }

    float expected_hz(size_t idx) const
    {
        return reporters_[idx].expected_hz();
    }

    const char *name(size_t idx) const
    {
        return reporters_[idx].name();
    }

    static constexpr size_t size()
    {
        return N;
    }

private:
    template <size_t... I>
    static std::array<FrequencyReporter<HistoryCap>, N> make_reporters_impl(const std::array<Config, N> &configs,
                                                                             int64_t window_us,
                                                                             int64_t sample_period_us,
                                                                             std::index_sequence<I...>)
    {
        return {FrequencyReporter<HistoryCap>(
                configs[I].name,
                *configs[I].counter,
                configs[I].expected_hz,
                configs[I].min_hz,
                window_us,
                sample_period_us)...};
    }

    static std::array<FrequencyReporter<HistoryCap>, N> make_reporters(const std::array<Config, N> &configs,
                                                                        int64_t window_us,
                                                                        int64_t sample_period_us)
    {
        return make_reporters_impl(configs, window_us, sample_period_us, std::make_index_sequence<N>{});
    }

    std::array<FrequencyReporter<HistoryCap>, N> reporters_;
};

