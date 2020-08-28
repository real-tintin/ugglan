#include <catch.h>
#include <catch_utils.h>

#include <thread>
#include <task.h>
#include <data_log_queue.h>
#include <data_logger.h>

static const std::string DATA_LOG_PATH = "./tests/test.dat";

static const uint8_t N_PUSH_IMU = 10;
static const uint8_t N_PUSH_ESC = 20;

static uint8_t i_push_imu = 0;
static uint8_t i_push_esc = 0;

DataLogQueue queue;
DataLogger logger(queue, DATA_LOG_PATH);

class TestTaskImu : public Task
{
public:
    TestTaskImu(uint32_t exec_period_ms, void (*exec_period_exceeded_cb)()) :
        Task(exec_period_ms, exec_period_exceeded_cb) {}
protected:
    void _execute() { if (i_push_imu < N_PUSH_IMU) { queue.push(0.3, DataLogSignal::ImuAccelerationX); i_push_imu++; } }
};

class TestTaskEsc : public Task
{
public:
    TestTaskEsc(uint32_t exec_period_ms, void (*exec_period_exceeded_cb)()) :
        Task(exec_period_ms, exec_period_exceeded_cb) {}
protected:
    void _execute() { if (i_push_esc < N_PUSH_ESC) { queue.push(0x02U, DataLogSignal::EscStatus0); i_push_esc++; } }
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
    TestTaskImu task_imu(10, nullptr);
    TestTaskEsc task_esc(10, nullptr);
    TestTaskLogger task_logger(10, nullptr);

    task_imu.launch();
    task_esc.launch();
    task_logger.launch();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    task_imu.teardown();
    task_esc.teardown();
    task_logger.teardown();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::string data = read_file(DATA_LOG_PATH);
    REQUIRE(data.size() > 0);
    // TODO: Compare file size to exp estimated size.
}
