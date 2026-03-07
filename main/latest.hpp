// latest.hpp
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

template<typename T>
class Latest
{
public:
    void write(const T& v)
    {
        portENTER_CRITICAL(&mux_);
        value_ = v;
        portEXIT_CRITICAL(&mux_);
    }

    T read() const
    {
        portENTER_CRITICAL(&mux_);
        T v = value_;
        portEXIT_CRITICAL(&mux_);
        return v;
    }

private:
    T value_{};
    mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
};
