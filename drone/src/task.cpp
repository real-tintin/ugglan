#include <task.h>

Task::Task(uint32_t exec_period_ms,
           std::string name) :
    _exec_period_ms(exec_period_ms),
    _name(name)
{
}

void Task::launch()
{
    _setup();
    _run_thread = true;
    _thread = std::thread(&Task::_execute_thread, this);

    logger.debug("Launching task " + _name + ".");
}

void Task::teardown()
{
    _run_thread = false;
    _thread.join();
    _finish();

    logger.debug("Tearing down task " + _name + ".");
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
            sleep_ms = 0;
            logger.warn("Execution period exceeded for task " + _name + " by " + \
                        std::to_string(exec_time_ms - _exec_period_ms)  + " ms.");
        }
        else
        {
            sleep_ms = _exec_period_ms - exec_time_ms;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }
}
