#ifndef POLLER_HH
#define POLLER_HH

#include <functional>
#include <vector>

#include <poll.h>

#include "file_descriptor.hh"

class Poller
{
public:
  struct Action
  {
    struct Result
    {
      enum class Type { Continue, Exit, Cancel } result;
      unsigned int exit_status;
      Result( const Type & s_result = Type::Continue, const unsigned int & s_status = EXIT_SUCCESS )
        : result( s_result ), exit_status( s_status ) {}
    };

    typedef std::function<Result(void)> CallbackType;

    FileDescriptor & fd;
    enum PollDirection : short { In = POLLIN, Out = POLLOUT } direction;
    CallbackType callback;
    std::function<bool(void)> when_interested;
    bool active;
    std::function<int(void)> timeout_amount;

    Action( FileDescriptor & s_fd,
            const PollDirection & s_direction,
            const CallbackType & s_callback,
            const std::function<int(void)> & s_timeout_amount = [] () { return -1; },
    const std::function<bool(void)> & s_when_interested = [] () { return true; })
      : fd( s_fd ), direction( s_direction ), callback( s_callback ),
        when_interested( s_when_interested ), active( true ),
        timeout_amount(s_timeout_amount) {}

    unsigned int service_count( void ) const;
  };

private:
  std::vector< Action > actions_;
  std::vector< pollfd > pollfds_;

  std::function<void(void)> timeout_func;

public:
  struct Result
  {
    enum class Type { Success, Timeout, Exit } result;
    unsigned int exit_status;
    Result( const Type & s_result, const unsigned int & s_status = EXIT_SUCCESS )
      : result( s_result ), exit_status( s_status ) {}
  };

  Poller() : actions_(), pollfds_(), timeout_func([] () {}) {}
  void add_action( Action action );
  Result poll( unsigned int id );
  void on_timeout(std::function<void(void)> callback);
  void run_threads();
};

namespace PollerShortNames {
typedef Poller::Action::Result Result;
typedef Poller::Action::Result::Type ResultType;
typedef Poller::Action::PollDirection Direction;
typedef Poller::Action Action;
typedef Poller::Result::Type PollResult;
}

#endif /* POLLER_HH */
