#include "watchdog.h"
#include <iostream>

Watchdog::Watchdog() :
    _interval(0),
    _timer(0),
    _running(false) 
{
}

Watchdog::Watchdog(unsigned int milliseconds, std::function<void(void*)> callback, void* params)
{
    Start(milliseconds, callback, params);
}

Watchdog::~Watchdog()
{
}

void Watchdog::Start(unsigned int milliseconds, std::function<void(void*)> callback, void* params)
{
    _interval = milliseconds;
    _timer = 0;
    _callback = callback;
    _running = true;
    _thread = std::thread(&Watchdog::Loop, this,params);
	_thread.detach();
}

void Watchdog::Stop()
{
    _running = false;
    _thread.join();
}

void Watchdog::Pet()
{
    _timer = 0;
}

void Watchdog::Loop(void* params)
{
    while (_running)
    {
        _timer++;
        if (_timer >= _interval)
        {
            _running = false;
            _callback(params);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
