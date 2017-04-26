#include <iostream>

#include "controller.hh"
#include "timestamp.hh"

using namespace std;


  
/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ), the_window_size(1.0), seq_number_sent(0), seq_number_acked(0),  num_acks(0)
{}

/* Get current window size, in datagrams */
unsigned int Controller::window_size( void )
{
  /* Default: fixed window size of 100 outstanding datagrams */
  

  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
	 << " window size is " << the_window_size << endl;
  }

  return (the_window_size < 1.0) ? 1.0 : (unsigned int)(the_window_size);
}

/* A datagram was sent */
void Controller::datagram_was_sent( const uint64_t sequence_number,
				    /* of the sent datagram */
				    const uint64_t send_timestamp )
                                    /* in milliseconds */
{
  /* Default: take no action */
  seq_number_sent = sequence_number;

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
  // if (seq_number_sent <= sequence_number_acked && seq_number_sent + the_window_size > sequence_number_acked) {
  seq_number_acked = sequence_number_acked;
  num_acks++;
  // }
  // cerr << "Num_acks: " << num_acks << ", window size: " << the_window_size << endl;
  if (num_acks >= the_window_size) {
    the_window_size += 0.1;
    num_acks = 0;
  }

  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
	 << " received ack for datagram " << sequence_number_acked
	 << " (send @ time " << send_timestamp_acked
	 << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
	 << endl;
  }
}
void Controller::notify_timeout( void ) {
  the_window_size = (the_window_size > 1.0) ? the_window_size / 3.0 : 1.0;
  num_acks = 0;
}
/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms( void )
{
  return 1000; /* timeout of one second */
}
