// task_base.hpp
#pragma once

#include <atomic>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class TaskBase
{
public:
    virtual ~TaskBase()
    {
        stop();
    }

    bool isStarted() const
    {
        return taskHandle_ != nullptr;
    }

    void stop()
    {
        if (!taskHandle_) {
            return;
        }

        stopRequested_ = true;
        onStopRequest();
        xTaskNotifyGive(taskHandle_);

        while (taskHandle_ != nullptr) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        stopRequested_ = false;
    }

protected:
    bool startTask(const char* name,
                   uint32_t stackWords,
                   UBaseType_t priority,
                   BaseType_t core = tskNO_AFFINITY)
    {
        configASSERT(taskHandle_ == nullptr);

        BaseType_t rc = xTaskCreatePinnedToCore(
            &TaskBase::taskEntry,
            name,
            stackWords,
            this,
            priority,
            &taskHandle_,
            core);

        return rc == pdPASS;
    }

    bool stopRequested() const
    {
        return stopRequested_;
    }

    TaskHandle_t taskHandle() const
    {
        return taskHandle_;
    }

    virtual void run() = 0;

    virtual void onStopRequest()
    {
    }

private:
    static void taskEntry(void* arg)
    {
        auto* self = static_cast<TaskBase*>(arg);
        self->run();
        self->taskHandle_ = nullptr;
        vTaskDelete(nullptr);
    }

protected:
    TaskHandle_t taskHandle_ = nullptr;
    std::atomic<bool> stopRequested_{false};
};
