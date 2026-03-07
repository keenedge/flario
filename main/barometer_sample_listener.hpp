#pragma once

class BarometerSampleListener {
public:
    virtual ~BarometerSampleListener() = default;
    virtual void onBarometerSampleReady() = 0;
};