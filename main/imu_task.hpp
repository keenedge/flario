#pragma once

#include <stdint.h>

#include "task_base.hpp"
#include "latest.hpp"

#include "BNO08x.hpp"
#include "samples.hpp"

#include "imu_sample_listener.hpp"

class ImuTask : public TaskBase
{
public:
    bool start();

    void registerListener(ImuSampleListener* listener)
    {
        imuSampleListener_ = listener;
    }

    ImuSample latestSample() const
    {
        return latest_.read();
    }

protected:
    void run() override;

private:
    Latest<ImuSample> latest_;
    ImuSampleListener* imuSampleListener_ = nullptr;
};
