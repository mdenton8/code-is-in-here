#include <algorithm>
#include <cassert>
#include <numeric>
#include <thread>

#include "poller.hh"
#include "util.hh"

using namespace std;
using namespace PollerShortNames;

// TODO cannot be called after run_threads
void Poller::add_action( Poller::Action action )
{
  actions_.push_back( action );
  pollfds_.push_back( { action.fd.fd_num(), 0, 0 } );
}

// TODO cannot be called after run_threads
void Poller::on_timeout(std::function<void(void)> callback) {
  timeout_func = callback;
}

unsigned int Poller::Action::service_count( void ) const
{
  return direction == Direction::In ? fd.read_count() : fd.write_count();
}

void Poller::run_threads() {
  vector<thread> threads;
  for ( unsigned int i = 0; i < actions_.size(); i++ ) {
    threads.emplace_back([this, i] () { poll(i); } );
  }

  for ( unsigned int i = 0; i < actions_.size(); i++ ) {
    threads.at(i).join();
  }
}

Poller::Result Poller::poll(unsigned int i)
{
  assert( pollfds_.size() == actions_.size() );

  /* tell poll whether we care about each fd */
  // for ( unsigned int i = 0; i < actions_.size(); i++ ) {
  assert( pollfds_.at( i ).fd == actions_.at( i ).fd.fd_num() );
  pollfds_.at( i ).events = (actions_.at( i ).active and actions_.at( i ).when_interested())
                            ? actions_.at( i ).direction : 0;

  /* don't poll in on fds that have had EOF */
  if ( actions_.at( i ).direction == Direction::In
       and actions_.at( i ).fd.eof() ) {
    pollfds_.at( i ).events = 0;
  }
  // }

  /* Quit if no member in pollfds_ has a non-zero direction */
  // sorry bye error handling
  // if ( not accumulate( pollfds_.begin(), pollfds_.end(), false,
  // [] ( bool acc, pollfd x ) { return acc or x.events; } ) ) {
  //   // return Result::Type::Exit;
  //   throw runtime_error("Want to exit");
  // }

  while (true) {

    int timeout_time = actions_.at(i).timeout_amount();
    if ( 0 == SystemCall( "poll", ::poll( &pollfds_[ 0 ], pollfds_.size(), timeout_time ) ) ) {
      timeout_func();
    }

    // for ( unsigned int i = 0; i < pollfds_.size(); i++ ) {
    if ( pollfds_[ i ].revents & (POLLERR | POLLHUP | POLLNVAL) ) {
      // return Result::Type::Exit;
      throw runtime_error("Received poller error");
    }

    if ( pollfds_[ i ].revents & pollfds_[ i ].events ) {
      /* we only want to call callback if revents includes
      the event we asked for */
      const auto count_before = actions_.at( i ).service_count();
      auto result = actions_.at( i ).callback();

      if ( count_before == actions_.at( i ).service_count() ) {
        // throw runtime_error( "Poller: busy wait detected: callback did not read/write fd" );
      }

      switch ( result.result ) {
      case ResultType::Exit:
        return Result( Result::Type::Exit, result.exit_status );
      case ResultType::Cancel:
        actions_.at( i ).active = false;
      case ResultType::Continue:
        break;
      }
    }
    // }

  }

  return Result::Type::Success;
}
