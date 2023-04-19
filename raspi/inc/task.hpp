#ifndef TASK_HPP
#define TASK_HPP

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

#include <logger.hpp>
#include <wall_time.hpp>

class Task
{
  public:
    Task(uint32_t exec_period_ms, std::string name = "N.N.");

    void launch();
    void teardown();

  protected:
    virtual void _setup(){};
    virtual void _execute(){};
    virtual void _finish(){};

  private:
    uint32_t _exec_period_ms;
    std::string _name;

    std::atomic<bool> _run_thread;
    std::thread _thread;

    void _execute_thread();
};

#endif /* TASK_HPP */
