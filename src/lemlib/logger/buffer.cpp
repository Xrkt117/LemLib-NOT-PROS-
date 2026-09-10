#define FMT_HEADER_ONLY
#include "fmt/core.h"
#include "lemlib/logger/buffer.hpp"
#include "lemlib/logger/message.hpp"
#include "lemlib/platform.hpp"

namespace lemlib {
Buffer::Buffer(std::function<void(const std::string&)> bufferFunc)
    : bufferFunc(bufferFunc),
      task(nullptr),
      rate(50) {
    task = new vex::task(taskEntry, this);
}

int Buffer::taskEntry(void* argument) {
    static_cast<Buffer*>(argument)->taskLoop();
    return 0;
}

bool Buffer::buffersEmpty() {
    mutex.lock();
    bool status = buffer.size() == 0;
    mutex.unlock();
    return status;
}

Buffer::~Buffer() {
    // make sure when the destructor is called so all
    // the messages are logged
    while (!buffersEmpty()) { platform::delay(10); }
    if (task != nullptr) {
        task->stop();
        delete task;
        task = nullptr;
    }
}

void Buffer::pushToBuffer(const std::string& bufferData) {
    mutex.lock();
    buffer.push_back(bufferData);
    mutex.unlock();
}

void Buffer::setRate(uint32_t rate) { this->rate = rate; }

void Buffer::taskLoop() {
    while (true) {
        mutex.lock();
        if (buffer.size() > 0) {
            bufferFunc(buffer.at(0));
            buffer.pop_front();
        }
        mutex.unlock();
        platform::delay(rate);
    }
}
} // namespace lemlib
