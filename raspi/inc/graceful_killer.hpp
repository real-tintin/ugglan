#ifndef GRACEFUL_KILLER_HPP
#define GRACEFUL_KILLER_HPP

#include <atomic>
#include <csignal>

class GracefulKiller
{
  public:
    GracefulKiller()
    {
        std::signal(SIGINT, GracefulKiller::_signal_handler);
        std::signal(SIGTERM, GracefulKiller::_signal_handler);
    }

    bool kill()
    {
        return (_sigint_called);
    }

  private:
    inline static std::atomic_bool _sigint_called{false};

    static void _signal_handler(int value)
    {
        _sigint_called = true;
    };
};

#endif /* GRACEFUL_KILLER_HPP */
