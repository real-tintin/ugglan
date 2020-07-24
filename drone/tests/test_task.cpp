#include <catch.h>

#include <task.h>

static const uint32_t EXECUTION_TIME_MS = 100;
static const uint32_t SLEEP_MS = 1000;

static uint8_t n_calls_setup = 0;
static uint8_t n_calls_execute = 0;
static uint8_t n_calls_finish = 0;

class TestTask : public Task
{
public:
    TestTask(uint32_t execution_time_ms) : Task(execution_time_ms)
    {
    }
protected:
    void _setup() { n_calls_setup++; }
    void _execute() { n_calls_execute++; }
    void _finish() { n_calls_finish++; }
};

TEST_CASE("launch and teardown")
{
    TestTask test_task(EXECUTION_TIME_MS);

    test_task.launch();
    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MS));
    test_task.teardown();

    REQUIRE(n_calls_setup == 1);
    REQUIRE(n_calls_execute == (SLEEP_MS / EXECUTION_TIME_MS));
    REQUIRE(n_calls_finish == 1);
}
