#include <task.h>
#include <thread>

#include <catch.hpp>
#include <catch_utils.hpp>

#include <data_log_queue.hpp>
#include <data_logger.hpp>

catch_utils::TmpDir tmpdir;

static const uint32_t EXEC_PERIOD_IMU_MS = 20;   // 50 Hz
static const uint32_t EXEC_PERIOD_ESC_MS = 10;   // 100 Hz
static const uint32_t EXEC_PERIOD_LOGGER_MS = 5; // 200 Hz

static const uint32_t SLEEP_MAIN_MS = 200; // Should be large enough to make all pushes to queue.

static const uint8_t IMU_N_PUSH_TO_QUEUE = 5;
static const uint8_t ESC_N_PUSH_TO_QUEUE = 10;

static const uint8_t IMU_PAYLOAD_SIZE = 2 + 8 + 1;
static const uint8_t ESC_PAYLOAD_SIZE = 2 + 1 + 1;

static const uint32_t EXP_DATA_SIZE = IMU_N_PUSH_TO_QUEUE * IMU_PAYLOAD_SIZE + ESC_N_PUSH_TO_QUEUE * ESC_PAYLOAD_SIZE;

static uint8_t i_push_imu = 0;
static uint8_t i_push_esc = 0;

DataLogQueue queue;
DataLogger data_logger(queue, tmpdir.get_path());

class TestTaskImu : public Task
{
  public:
    using Task::Task;

  protected:
    void _execute() override
    {
        if (i_push_imu < IMU_N_PUSH_TO_QUEUE)
        {
            queue.push(double(0.3), DataLogSignal::ImuAccelerationX);
            i_push_imu++;
        }
    }
};

class TestTaskEsc : public Task
{
  public:
    using Task::Task;

  protected:
    void _execute() override
    {
        if (i_push_esc < ESC_N_PUSH_TO_QUEUE)
        {
            queue.push(uint8_t(0x02U), DataLogSignal::EscStatus0);
            i_push_esc++;
        }
    }
};

class TestTaskLogger : public Task
{
  public:
    using Task::Task;

  protected:
    void _setup() override
    {
        data_logger.start();
    }
    void _execute() override
    {
        data_logger.pack();
    }
    void _finish() override
    {
        data_logger.stop();
    }
};

void split_raw_into_header_and_data(std::string &raw, std::string &header, std::string &data)
{
    size_t first_endl = raw.find(DATA_LOG_ENDL);

    header = raw.substr(0, first_endl);
    data = raw.substr(first_endl + 1);
}

TEST_CASE("data_logger")
{
    TestTaskImu task_imu(EXEC_PERIOD_IMU_MS);
    TestTaskEsc task_esc(EXEC_PERIOD_ESC_MS);
    TestTaskLogger task_logger(EXEC_PERIOD_LOGGER_MS);

    task_imu.launch();
    task_esc.launch();
    task_logger.launch();

    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MAIN_MS));

    task_imu.teardown();
    task_esc.teardown();
    task_logger.teardown();

    std::string header, data;
    std::string raw = catch_utils::read_file(data_logger.get_file_path());
    split_raw_into_header_and_data(raw, header, data);

    REQUIRE(header.size() > 0);
    REQUIRE(data.size() == EXP_DATA_SIZE);
}
