#include <task.h>

Task::Task(uint32_t execution_time_ms) :
    _execution_time_ms(execution_time_ms)
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
    while (_run_thread)
    {
        _execute();
        std::this_thread::sleep_for(std::chrono::milliseconds(_execution_time_ms));
    }
}
