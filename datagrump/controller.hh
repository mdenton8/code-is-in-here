#ifndef CONTROLLER_HH
#define CONTROLLER_HH

#include <cstdint>
#include <map>
#include <vector>
#include <mutex>
// #include <deque>

/* Congestion controller interface */

class Controller
{
private:
  bool debug_; /* Enables debugging output */
  uint64_t bw_time_window;
  uint64_t rtt_time_window;
  uint64_t curr_rtt_estimate; // ms
  double curr_bw_estimate; // bytes/ms

  uint64_t delivered_bytes;

  double pacing_gain, cwnd_gain; // varying parameters for searching space

  uint64_t phase;
  uint64_t time_to_change_phase;

  // map seq_num to sent timestamp
  std::map<uint64_t, uint64_t> packet_send_time;
  std::map<uint64_t, uint64_t> packet_ack_time;
  std::map<uint64_t, uint64_t> packet_ack_sent_time;

  // timestamp of ACK received, and rtt estimate
  std::multimap<uint64_t, uint64_t> rtt_estimates;

  std::multimap<uint64_t, double> bw_estimates;

  // map seq_num to delivered
  std::map<uint64_t, uint64_t> packet_delivered;

  std::mutex global_lock;

public:
  /* Public interface for the congestion controller */
  /* You can change these if you prefer, but will need to change
     the call site as well (in sender.cc) */

  /* Default constructor */
  Controller( const bool debug );

  /* Get current window size, in datagrams */
  unsigned int window_size( void );

  /* A datagram was sent */
  void datagram_was_sent( const uint64_t sequence_number,
			  const uint64_t send_timestamp );

  /* An ack was received */
  void ack_received( const uint64_t sequence_number_acked,
		     const uint64_t send_timestamp_acked,
		     const uint64_t recv_timestamp_acked,
		     const uint64_t timestamp_ack_received );

  /* How long to wait (in milliseconds) if there are no acks
     before sending one more datagram */
  unsigned int timeout_ms( void );

  void notify_timeout( void );

  double get_bw_estimate();
  double get_pacing_gain();
};

#endif
