#include <task.hpp>

static const uint32_t WARN_EXCEEDED_EXEC_PERIOD_PERC = 20; // [%]

Task::Task(uint32_t exec_period_ms, std::string name) : _exec_period_ms(exec_period_ms), _name(name)
{
}

void Task::launch()
{
    _setup();
    _run_thread = true;
    _thread = std::thread(&Task::_execute_thread, this);

    logger.info("Launching task " + _name + ".");
}

void Task::teardown()
{
    _run_thread = false;
    _thread.join();
    _finish();

    logger.info("Tearing down task " + _name + ".");
}

void Task::_execute_thread()
{
    uint32_t start_time_ms, exec_time_ms, sleep_ms;

    while (_run_thread)
    {
        start_time_ms = WallTime::millis();
        _execute();
        exec_time_ms = WallTime::millis() - start_time_ms;

        if (exec_time_ms > _exec_period_ms)
        {
            sleep_ms = 0;
            uint32_t exec_exceed_perc = (exec_time_ms - _exec_period_ms) * 100 / _exec_period_ms;

            if (exec_exceed_perc > WARN_EXCEEDED_EXEC_PERIOD_PERC)
            {
                logger.warn("Execution period exceeded for task " + _name + " by " +
                            std::to_string(exec_time_ms - _exec_period_ms) + " ms (" +
                            std::to_string(exec_exceed_perc) + " %).");
            }
        }
        else
        {
            sleep_ms = _exec_period_ms - exec_time_ms;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }
}
