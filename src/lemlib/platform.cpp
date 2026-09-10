#include "lemlib/platform.hpp"

#include "v5_api.h"

namespace lemlib {
namespace platform {

uint32_t millis() { return vex::timer::system(); }

void delay(uint32_t milliseconds) { vex::this_thread::sleep_for(milliseconds); }

uint32_t competitionStatus() { return vexCompetitionStatus(); }

void drive(vex::motor_group* motors, float power) {
    if (motors == nullptr) return;
    if (power > 127) power = 127;
    if (power < -127) power = -127;
    motors->spin(vex::forward, power * 100.0 / 127.0, vex::pct);
}

AsyncTask::AsyncTask()
    : task(nullptr) {}

AsyncTask::~AsyncTask() { stop(); }

int AsyncTask::run(void* argument) {
    AsyncTask* self = static_cast<AsyncTask*>(argument);
    self->callback();
    return 0;
}

void AsyncTask::start(const std::function<void()>& callback) {
    stop();
    this->callback = callback;
    task = new vex::task(run, this);
}

void AsyncTask::stop() {
    if (task != nullptr) {
        task->stop();
        delete task;
        task = nullptr;
    }
}

} // namespace platform
} // namespace lemlib
