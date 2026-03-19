#pragma once

<<<<<<< HEAD
#include <stdint.h>

#include "task_base.hpp"
#include "samples.hpp"
#include "latest.hpp"

#include "barometer_sample_listener.hpp"

class BarometerTask : public TaskBase
{
public:
    bool start();
    
    void registerListener(BarometerSampleListener* listener)
    {
        barometerSampleListener_ = listener;
    }
    
    BarometerSample latestSample() const
    {
        return latest_.read();
    }

protected:
    void run() override;

private:
    void init_sensor();
    BarometerSample read_sensor();

private:
    BarometerSampleListener* barometerSampleListener_ = nullptr;
    Latest<BarometerSample> latest_;
};
=======
void barometer_start(void);
void barometer_task(void *arg);
>>>>>>> testf-branch
