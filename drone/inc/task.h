#ifndef TASK_H
#define TASK_H

#include <cstdint>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include <logger.h>
#include <wall_time.h>

inline const uint32_t TASK_WARN_EXCEEDED_EXEC_PERIOD_PERC = 20; // [%]

class Task
{
public:
    Task(uint32_t exec_period_ms,
         std::string name = "N.N.");

    void launch();
    void teardown();
protected:
    virtual void _setup() {};
    virtual void _execute() {};
    virtual void _finish() {};
private:
    uint32_t _exec_period_ms;
    std::string _name;

    std::atomic<bool> _run_thread;
    std::thread _thread;

    void _execute_thread();
};

#endif /* TASK_H */
