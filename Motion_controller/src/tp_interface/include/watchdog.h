#pragma once

#include <thread>
#include <atomic>

class Watchdog
{
public:
    Watchdog();
    Watchdog(unsigned int milliseconds, std::function<void(void*)> callback, void* params);
    ~Watchdog();

    void Start(unsigned int milliseconds, std::function<void(void*)> callback, void* params);
    void Stop();
    void Pet();

private:
    unsigned int _interval;
    std::atomic<unsigned int> _timer;
    std::atomic<bool> _running;
    std::thread _thread;
    std::function<void(void*)> _callback;

    void Loop(void* params);
};
