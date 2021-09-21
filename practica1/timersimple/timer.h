#ifndef TIMER_H
#define TIMER_H

#include <thread>
#include <chrono>
#include <functional>
#include <future>
#include <cstdio>
#include <iostream>

class Timer {
public:

    Timer() {};

    template<class callable>
    void connect(callable &&f) {
        std::thread([=]() {
            while (true) {
                if (go.load())
                    std::invoke(f);
                std::this_thread::sleep_for(std::chrono::milliseconds(period.load()));
            }
        }).detach();
    };

    //Start the stopwatch
    void start(int p) {
        period.store(p);
        go.store(true);
    };

    //Stops the stopwatch
    void stop() { go.store(!go); };

    //Sets values for the period
    void setPeriod(int p) { period.store(p); };

    //Gets the preiod
    int getPeriod() {
        return period.load();
    }

    //Sets the given integer to the init variable
    void setInit(int p) { init.store(p); };

    //Returns the value stored in init
    int getInit() {
        return init.load();
    }

private:
    std::atomic_bool go = false;
    std::atomic_int period = 0;
    std::atomic_int init = 0;

};

#endif // TIMER_H
