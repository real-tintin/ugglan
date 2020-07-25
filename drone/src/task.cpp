#include <task.h>

Task::Task(uint32_t exec_period_ms, void (*exec_period_exceeded_cb)()) :
    _exec_period_ms(exec_period_ms),
    _exec_period_exceeded_cb(exec_period_exceeded_cb)
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
    uint32_t start_time_ms, exec_time_ms, sleep_ms;

    while (_run_thread)
    {
        start_time_ms = wall_time.millis();
        _execute();
        exec_time_ms = wall_time.millis() - start_time_ms;

        if (exec_time_ms > _exec_period_ms)
        {
            _exec_period_exceeded_cb();
            sleep_ms = 0;
        }
        else
        {
            sleep_ms = _exec_period_ms - exec_time_ms;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }
}
