#ifndef TASK_H
#define TASK_H

#include <cstdint>
#include <thread>
#include <atomic>
#include <chrono>
#include <wall_time.h>

class Task
{
public:
    Task(uint32_t execution_time_ms, void (*exec_time_exceeded_cb)());

    void launch();
    void teardown();
protected:
    virtual void _setup() {};
    virtual void _execute() {};
    virtual void _finish() {};
private:
    uint32_t _exp_exec_time_ms;
    void (*_exec_time_exceeded_cb)();

    std::atomic<bool> _run_thread;
    std::thread _thread;

    void _execute_thread();
};

#endif /* TASK_H */
