#include <catch.h>

#include <zmq/zmq.hpp>
#include <nlohmann/json.hpp>
#include <streamer_server_msg.h>

using json = nlohmann::ordered_json;
using namespace streamer::msg;

enum class Status {
    Received = 0,
    Lost = 1
};

class Mail : public Base<Status>
{
public:
    using Base::Base;

    const Status status() { return _get_metadata(); }
protected:
    const std::string _get_metadata_key() override { return "status"; }
};

TEST_CASE("to msg and back")
{
    Status exp_status = Status::Received;
    json exp_data = {{"data", "my love..."}};

    Mail exp_mail{exp_status, exp_data};
    Mail act_mail{};

    zmq::message_t msg{};

    exp_mail.to_msg(msg);
    act_mail.from_msg(msg);

    REQUIRE(exp_status == act_mail.status());
    REQUIRE(exp_data == act_mail.data());
}

TEST_CASE("invalid msg")
{
    zmq::message_t msg{std::string("not_valid")};

    Mail mail{msg};

    REQUIRE(mail.valid() == false);
}
