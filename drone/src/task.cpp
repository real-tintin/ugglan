#include <task.h>

Task::Task(uint32_t execution_time_ms, void (*exec_time_exceeded_cb)()) :
    _exp_exec_time_ms(execution_time_ms),
    _exec_time_exceeded_cb(exec_time_exceeded_cb)
{
}

void Task::launch()
{
    _setup();
    _run_thread = true;
    _thread = std::thread(&Task::_execute_thread, this);
}

void Task::teardown()
{
    _run_thread = false;
    _thread.join();
    _finish();
}

void Task::_execute_thread()
{
    uint32_t _start_exec_time_ms, act_exec_time_ms, sleep_ms;

    while (_run_thread)
    {
        _start_exec_time_ms = wall_time.millis();
        _execute();
        act_exec_time_ms = wall_time.millis() - _start_exec_time_ms;

        if (act_exec_time_ms > _exp_exec_time_ms)
        {
            _exec_time_exceeded_cb();
            sleep_ms = 0;
        }
        else
        {
            sleep_ms = _exp_exec_time_ms - act_exec_time_ms;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }
}
