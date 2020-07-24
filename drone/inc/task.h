#ifndef TASK_H
#define TASK_H

#include <cstdint>
#include <thread>
#include <atomic>
#include <chrono>

class Task
{
public:
    Task(uint32_t execution_time_ms);

    void launch();
    void teardown();
protected:
    virtual void _setup() {};
    virtual void _execute() {};
    virtual void _finish() {};
private:
    uint32_t _execution_time_ms;
    std::atomic<bool> _run_thread;
    std::thread _thread;

    void _execute_thread();
};

#endif /* TASK_H */
