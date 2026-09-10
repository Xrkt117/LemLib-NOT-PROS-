#pragma once

#include <cstdint>
#include <functional>

#include "vex.h"

namespace lemlib {
namespace platform {

uint32_t millis();
void delay(uint32_t milliseconds);
uint32_t competitionStatus();
void drive(vex::motor_group* motors, float power);

class Mutex {
    public:
        Mutex();
        void lock();
        void unlock();

    private:
        volatile int locked;
};

class AsyncTask {
    public:
        AsyncTask();
        ~AsyncTask();

        AsyncTask(const AsyncTask&) = delete;
        AsyncTask& operator=(const AsyncTask&) = delete;

        void start(const std::function<void()>& callback);
        void stop();

    private:
#if LEMLIB_FULL_VEX_SDK
        static int run(void* argument);
        vex::task* task;
#else
        vex::thread* task;
#endif
        std::function<void()> callback;
};

} // namespace platform
} // namespace lemlib
