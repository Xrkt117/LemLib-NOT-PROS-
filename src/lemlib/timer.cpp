#include "lemlib/timer.hpp"
#include "lemlib/platform.hpp"

using namespace lemlib;

Timer::Timer(uint32_t time)
    : period(time) {
    lastTime = platform::millis();
}

uint32_t Timer::getTimeSet() {
    const uint32_t time = platform::millis();
    if (!paused) timeWaited += time - lastTime; // don't update if paused
    lastTime = time; // update last time
    return period;
}

uint32_t Timer::getTimeLeft() {
    const uint32_t time = platform::millis();
    if (!paused) timeWaited += time - lastTime; // don't update if paused
    lastTime = time; // update last time
    const int delta = period - timeWaited; // calculate how much time is left
    return (delta > 0) ? delta : 0; // return 0 if timer is done
}

uint32_t Timer::getTimePassed() {
    const uint32_t time = platform::millis();
    if (!paused) timeWaited += time - lastTime; // don't update if paused
    lastTime = time; // update last time;
    return timeWaited;
}

bool Timer::isDone() {
    const uint32_t time = platform::millis();
    if (!paused) timeWaited += time - lastTime; // don't update if paused
    lastTime = time; // update last time
    const int delta = period - timeWaited; // calculate how much time is left
    return delta <= 0;
}

bool Timer::isPaused() {
    const uint32_t time = platform::millis();
    if (!paused) timeWaited += time - lastTime; // don't update if paused
    return paused;
}

void Timer::set(uint32_t time) {
    period = time; // set how long to wait
    reset();
}

void Timer::reset() {
    timeWaited = 0;
    lastTime = platform::millis();
}

void Timer::pause() {
    if (!paused) lastTime = platform::millis();
    paused = true;
}

void Timer::resume() {
    if (paused) lastTime = platform::millis();
    paused = false;
}

void Timer::waitUntilDone() {
    do platform::delay(5);
    while (!this->isDone());
}
