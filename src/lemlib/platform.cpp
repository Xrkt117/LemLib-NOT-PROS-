#include "lemlib/platform.hpp"

#if LEMLIB_FULL_VEX_SDK
#include "v5_api.h"
#endif

namespace lemlib {
namespace platform {

uint32_t millis() { return vex::timer::system(); }

void delay(uint32_t milliseconds) { vex::this_thread::sleep_for(milliseconds); }

uint32_t competitionStatus() {
#if LEMLIB_FULL_VEX_SDK
    return vexCompetitionStatus();
#else
    static vex::competition competition;
    uint32_t status = competition.isEnabled() ? 1 : 0;
    if (competition.isAutonomous()) status |= 2;
    if (competition.isDriverControl()) status |= 4;
    return status;
#endif
}

void drive(vex::motor_group* motors, float power) {
    if (motors == nullptr) return;
    if (power > 127) power = 127;
    if (power < -127) power = -127;
    motors->spin(vex::directionType::fwd, power * 100.0 / 127.0, vex::percentUnits::pct);
}

Mutex::Mutex()
    : locked(0) {}

void Mutex::lock() {
    while (__sync_lock_test_and_set(&locked, 1)) delay(1);
}

void Mutex::unlock() { __sync_lock_release(&locked); }

AsyncTask::AsyncTask()
    : task(nullptr) {}

AsyncTask::~AsyncTask() { stop(); }

#if LEMLIB_FULL_VEX_SDK
int AsyncTask::run(void* argument) {
    AsyncTask* self = static_cast<AsyncTask*>(argument);
    self->callback();
    return 0;
}
#endif

void AsyncTask::start(const std::function<void()>& callback) {
    stop();
    this->callback = callback;
#if LEMLIB_FULL_VEX_SDK
    task = new vex::task(run, this);
#else
    task = new vex::thread([this]() { this->callback(); });
#endif
}

void AsyncTask::stop() {
    if (task != nullptr) {
#if LEMLIB_FULL_VEX_SDK
        task->stop();
#else
        task->interrupt();
#endif
        delete task;
        task = nullptr;
    }
}

} // namespace platform
} // namespace lemlib
