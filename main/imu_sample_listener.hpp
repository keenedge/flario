#pragma once

class ImuSampleListener {
public:
    virtual ~ImuSampleListener() = default;
    virtual void onImuSampleReady() = 0;
};