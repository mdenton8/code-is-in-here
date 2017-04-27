#include <iostream>

#include "controller.hh"
#include "timestamp.hh"

using namespace std;

static const int cutoff = 75;
  
/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ), the_window_size(1.0),  num_acks(0),
  old_window_size(0.0), old_num_acks(0)
{}

/* Get current window size, in datagrams */
unsigned int Controller::window_size( void )
{
  /* Default: fixed window size of 100 outstanding datagrams */
  

  if ( debug_ || true) {
    cerr << "At time " << timestamp_ms()
	 << " window size is " << the_window_size << endl;
  }


  return (the_window_size < 1) ? 1 : (unsigned int)(the_window_size);
}

/* A datagram was sent */
void Controller::datagram_was_sent( const uint64_t sequence_number,
				    /* of the sent datagram */
				    const uint64_t send_timestamp )
                                    /* in milliseconds */
{
  /* Default: take no action */

  if ( debug_ ) {
    cerr << "At time " << send_timestamp
	 << " sent datagram " << sequence_number << endl;
  }
}

/* An ack was received */
void Controller::ack_received( const uint64_t sequence_number_acked,
			       /* what sequence number was acknowledged */
			       const uint64_t send_timestamp_acked,
			       /* when the acknowledged datagram was sent (sender's clock) */
			       const uint64_t recv_timestamp_acked,
			       /* when the acknowledged datagram was received (receiver's clock)*/
			       const uint64_t timestamp_ack_received )
                               /* when the ack was received (by sender) */
{
  
  // if we cut in half, wait until one whole window has cleared.
  if ((timestamp_ack_received - send_timestamp_acked) > cutoff && old_num_acks >= old_window_size)
  {
    old_window_size = the_window_size;
    old_num_acks = 0;


    num_acks = 0;
    the_window_size = (the_window_size > 1) ? the_window_size / 1.3 : 1;
  }
  else
  {

    // if in "recovery" state, wait until out to start increasing window size again.
    if (old_num_acks < old_window_size)
      old_num_acks++;
    else {
      num_acks++;
      if (num_acks >= the_window_size)
        the_window_size += 0.45;
    }
  }

  if ( debug_ || true) {
    cerr << "At time " << timestamp_ack_received
	 << " received ack for datagram " << sequence_number_acked
	 << " (send @ time " << send_timestamp_acked
	 << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
	 << endl;
  }
}
void Controller::notify_timeout( void ) {
  the_window_size = (the_window_size > 1) ? the_window_size / 5 : 1;
  num_acks = 0;
}
/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms( void )
{
  return 1000; /* timeout of one second */
}
