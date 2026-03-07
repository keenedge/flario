#pragma once

class FusionSampleListener {
public:
    virtual ~FusionSampleListener() = default;
    virtual void onFusionSampleReady() = 0;
};