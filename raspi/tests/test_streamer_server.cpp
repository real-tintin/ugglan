#include <cstring>

#include <catch/catch.hpp>
#include <nlohmann/json.hpp>
#include <zmq/zmq.hpp>

#include <data_log_queue.hpp>
#include <data_log_utils.hpp>
#include <streamer_server.hpp>
#include <streamer_server_msg.hpp>
#include <zmq_conn_stub.hpp>

using json = nlohmann::ordered_json;
using namespace streamer;
using namespace streamer::msg;

const json JSON_EMPTY_DICT = json::parse("{}");
const json JSON_EMPTY_LIST = json::parse("[]");

class ServerTestFixture
{
public:
    ServerTestFixture() : server(req_conn, stream_conn, queue)
    {
    }

protected:
    ZmqRepStub req_conn{};
    ZmqPushStub stream_conn{};

    DataLogQueue queue{};

    Server server;

    void send_request_and_expect_response(Request &req, Response &exp_res)
    {
        _send_request(req);

        server.execute();

        Response act_res = _get_last_response();

        REQUIRE(exp_res.code() == act_res.code());
        REQUIRE(exp_res.data() == act_res.data());
    }

private:
    void _send_request(Request &req)
    {
        zmq::message_t msg{};

        req.to_msg(msg);

        req_conn.set_exp_recv_msg(msg);
    }

    Response _get_last_response()
    {
        Response res{};
        const zmq::message_t &msg = req_conn.get_last_send_msg();

        res.from_msg(msg);

        return res;
    }
};

TEST_CASE_METHOD(ServerTestFixture, "simple life cycle")
{
    server.connect();
    server.execute();
    server.disconnect();

    REQUIRE(req_conn.get_n_calls_recv() == 1);
    REQUIRE(req_conn.get_n_calls_send() == 0);
    REQUIRE(stream_conn.get_n_calls_recv() == 0);
    REQUIRE(stream_conn.get_n_calls_send() == 0);
}

TEST_CASE_METHOD(ServerTestFixture, "request methods return as expected")
{
    Method req_method{};
    json req_data{};

    Code exp_res_code = Code::Ok;
    json exp_res_data = JSON_EMPTY_DICT;

    SECTION("get")
    {
        SECTION("data log metadata")
        {
            req_method = Method::Get_DataLogMetadata;
            data_log::utils::add_data_log_metadata_to_json(exp_res_data);

            Request req{req_method, req_data};
            Response exp_res{exp_res_code, exp_res_data};

            send_request_and_expect_response(req, exp_res);
        }
        SECTION("selected data log signals")
        {
            req_method = Method::Get_SelectedDataLogSignals;
            exp_res_data = JSON_EMPTY_LIST;

            Request req{req_method, req_data};
            Response exp_res{exp_res_code, exp_res_data};

            send_request_and_expect_response(req, exp_res);
        }
    }
    SECTION("set")
    {
        SECTION("start stream")
        {
            req_method = Method::Set_StartStream;
            req_data = JSON_EMPTY_DICT;

            Request req{req_method, req_data};
            Response exp_res{exp_res_code, exp_res_data};

            send_request_and_expect_response(req, exp_res);
        }
        SECTION("stop stream")
        {
            req_method = Method::Set_StopStream;
            req_data = JSON_EMPTY_DICT;

            Request req{req_method, req_data};
            Response exp_res{exp_res_code, exp_res_data};

            send_request_and_expect_response(req, exp_res);
        }
        SECTION("selected data log signals")
        {
            req_method = Method::Set_SelectedDataLogSignals;
            req_data = JSON_EMPTY_LIST;

            Request req{req_method, req_data};
            Response exp_res{exp_res_code, exp_res_data};

            send_request_and_expect_response(req, exp_res);
        }
    }
}

TEST_CASE_METHOD(ServerTestFixture, "update with two valid data log signals")
{
    WHEN("two valid signals are set")
    {
        json signals = json(std::vector<DataLogSignal>{DataLogSignal::ImuAccelerationX, DataLogSignal::EscStatus0});

        Request req{Method::Set_SelectedDataLogSignals, signals};
        Response exp_res{Code::Ok, JSON_EMPTY_DICT};

        send_request_and_expect_response(req, exp_res);

        THEN("get should return with the two signals")
        {
            Request req{Method::Get_SelectedDataLogSignals, JSON_EMPTY_DICT};
            Response exp_res{Code::Ok, signals};

            send_request_and_expect_response(req, exp_res);
        }
    }
}

TEST_CASE_METHOD(ServerTestFixture, "invalid set selected data log signals requests return error")
{
    Response exp_res{Code::Error, JSON_EMPTY_DICT};

    SECTION("signal out of lower bound")
    {
        json invalid_signals = json(std::vector<int>{-1});
        Request req{Method::Set_SelectedDataLogSignals, invalid_signals};

        send_request_and_expect_response(req, exp_res);
    }
    SECTION("signal out of upper bound")
    {
        json invalid_signals = json(std::vector<int>{UINT16_MAX});
        Request req{Method::Set_SelectedDataLogSignals, invalid_signals};

        send_request_and_expect_response(req, exp_res);
    }
    SECTION("signal float")
    {
        json invalid_signals = json(std::vector<double>{1.23});
        Request req{Method::Set_SelectedDataLogSignals, invalid_signals};

        send_request_and_expect_response(req, exp_res);
    }
    SECTION("two many signals")
    {
        std::vector<int> to_many_signals(MAX_SELECTED_DATA_LOG_SIGNALS + 1);
        std::iota(std::begin(to_many_signals), std::end(to_many_signals), 0);

        json invalid_signals = json(to_many_signals);
        Request req{Method::Set_SelectedDataLogSignals, invalid_signals};

        send_request_and_expect_response(req, exp_res);
    }
    SECTION("not json list")
    {
        json invalid_signals = JSON_EMPTY_DICT;
        Request req{Method::Set_SelectedDataLogSignals, invalid_signals};

        send_request_and_expect_response(req, exp_res);
    }
    SECTION("not unique signals")
    {
        json invalid_signals =
            json(std::vector<DataLogSignal>{DataLogSignal::ImuAccelerationX, DataLogSignal::ImuAccelerationX});
        Request req{Method::Set_SelectedDataLogSignals, invalid_signals};

        send_request_and_expect_response(req, exp_res);
    }
}

TEST_CASE_METHOD(ServerTestFixture, "invalid set data log signals request keeps previous set signals")
{
    GIVEN("an valid set request with two signals is send")
    {
        json signals = json(std::vector<DataLogSignal>{DataLogSignal::ImuAccelerationX, DataLogSignal::EscStatus0});

        Request req{Method::Set_SelectedDataLogSignals, signals};
        Response exp_res{Code::Ok, JSON_EMPTY_DICT};

        send_request_and_expect_response(req, exp_res);

        WHEN("an invalid set request is send")
        {
            Request req{Method::Set_SelectedDataLogSignals, JSON_EMPTY_DICT};
            Response exp_res{Code::Error, JSON_EMPTY_DICT};

            send_request_and_expect_response(req, exp_res);

            THEN("a get should return the previous two valid signals")
            {
                Request req{Method::Get_SelectedDataLogSignals, JSON_EMPTY_DICT};
                Response exp_res{Code::Ok, signals};

                send_request_and_expect_response(req, exp_res);
            }
        }
    }
}

TEST_CASE_METHOD(ServerTestFixture, "streaming")
{
    SECTION("toggle start & stop")
    {
        GIVEN("a set request sent to start stream")
        {
            Request req{Method::Set_StartStream, JSON_EMPTY_DICT};
            Response exp_res{Code::Ok, JSON_EMPTY_DICT};

            send_request_and_expect_response(req, exp_res);

            THEN("bytes should be sent on stream")
            {
                stream_conn.clear_last_send_msg();

                server.execute();

                const zmq::message_t &msg = stream_conn.get_last_send_msg();
                REQUIRE_FALSE(msg.empty());

                AND_GIVEN("a set request sent to stop stream")
                {
                    Request req{Method::Set_StopStream, JSON_EMPTY_DICT};
                    Response exp_res{Code::Ok, JSON_EMPTY_DICT};

                    send_request_and_expect_response(req, exp_res);

                    THEN("bytes should not be sent on stream")
                    {
                        stream_conn.clear_last_send_msg();

                        server.execute();

                        const zmq::message_t &msg = stream_conn.get_last_send_msg();
                        REQUIRE(msg.empty());
                    }
                }
            }
        }
    }
    SECTION("parse two selected signal bytes")
    {
        GIVEN("two signals pushed to data log queue")
        {
            double exp_imu_acc_x = 3.14;
            uint8_t exp_esc_status_0 = 77;

            queue.push(exp_imu_acc_x, DataLogSignal::ImuAccelerationX);
            queue.push(exp_esc_status_0, DataLogSignal::EscStatus0);

            WHEN("request to select the same two signals to be logged")
            {
                json signals =
                    json(std::vector<DataLogSignal>{DataLogSignal::ImuAccelerationX, DataLogSignal::EscStatus0});

                Request req{Method::Set_SelectedDataLogSignals, signals};
                Response exp_res{Code::Ok, JSON_EMPTY_DICT};

                send_request_and_expect_response(req, exp_res);

                AND_WHEN("streaming starting")
                {
                    Request req{Method::Set_StartStream, JSON_EMPTY_DICT};
                    Response exp_res{Code::Ok, JSON_EMPTY_DICT};

                    send_request_and_expect_response(req, exp_res);

                    THEN("expect selected signal bytes on stream")
                    {
                        size_t exp_size = 4 + 2 * 2 + 8 + 1; // timestamp + 2 * ids + double + uint8_t

                        stream_conn.clear_last_send_msg();

                        server.execute();

                        const zmq::message_t &msg = stream_conn.get_last_send_msg();
                        const uint8_t *const bytes = (const uint8_t *)msg.data();

                        REQUIRE(exp_size == msg.size());
                        // NOLINTBEGIN(bugprone-suspicious-memory-comparison)
                        REQUIRE(std::memcmp(&exp_imu_acc_x, &bytes[6], sizeof(double)) == 0);
                        REQUIRE(std::memcmp(&exp_esc_status_0, &bytes[16], sizeof(uint8_t)) == 0);
                        // NOLINTEND(bugprone-suspicious-memory-comparison)
                    }
                }
            }
        }
    }
}
