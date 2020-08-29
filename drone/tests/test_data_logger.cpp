#include <catch.h>
#include <catch_utils.h>

#include <thread>
#include <task.h>
#include <data_log_queue.h>
#include <data_logger.h>

catch_utils::TmpDir tmpdir;

static const uint32_t EXEC_PERIOD_IMU_MS = 20; // 50 Hz
static const uint32_t EXEC_PERIOD_ESC_MS = 10; // 100 Hz
static const uint32_t EXEC_PERIOD_LOGGER_MS = 5; // 200 Hz

static const uint32_t SLEEP_MAIN_MS = 200;

DataLogQueue queue;
DataLogger logger(queue, tmpdir.get_path());

class TestTaskImu : public Task
{
public:
    TestTaskImu(uint32_t exec_period_ms, void (*exec_period_exceeded_cb)()) :
        Task(exec_period_ms, exec_period_exceeded_cb) {}
protected:
    void _execute() { queue.push(double(0.3), DataLogSignal::ImuAccelerationX); }
};

class TestTaskEsc : public Task
{
public:
    TestTaskEsc(uint32_t exec_period_ms, void (*exec_period_exceeded_cb)()) :
        Task(exec_period_ms, exec_period_exceeded_cb) {}
protected:
    void _execute() { queue.push(uint8_t(0x02U), DataLogSignal::EscStatus0); }
};

class TestTaskLogger : public Task
{
public:
    TestTaskLogger(uint32_t exec_period_ms, void (*exec_period_exceeded_cb)()) :
        Task(exec_period_ms, exec_period_exceeded_cb) {}
protected:
    void _setup() {  logger.start(); }
    void _execute() {  logger.pack(); }
    void _finish() {  logger.stop(); }
};

TEST_CASE("data_logger")
{
    TestTaskImu task_imu(EXEC_PERIOD_IMU_MS, nullptr);
    TestTaskEsc task_esc(EXEC_PERIOD_ESC_MS, nullptr);
    TestTaskLogger task_logger(EXEC_PERIOD_LOGGER_MS, nullptr);

    task_imu.launch();
    task_esc.launch();
    task_logger.launch();

    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MAIN_MS));

    task_imu.teardown();
    task_esc.teardown();
    task_logger.teardown();

    std::string data = catch_utils::read_file(logger.get_file_path());
    REQUIRE(data.size() > 0);
    // TODO: Compare file size to exp estimated size.
}
