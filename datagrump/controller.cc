#include <iostream>

#include "controller.hh"
#include "timestamp.hh"

#include <limits>

using namespace std;

// TODO a lot of shit in this controller is not thread-safe. That shall be fixed
// TODO need to sleep the sender when window not open

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ), bw_time_window(500), rtt_time_window(2000),
    curr_rtt_estimate(40), curr_bw_estimate(1.0), delivered_bytes(0),
    pacing_gain(1.0), cwnd_gain(1.5),
    phase(0), time_to_change_phase(40),
    packet_send_time(), packet_ack_time(), packet_ack_sent_time(),
    rtt_estimates(), bw_estimates(), packet_delivered(),
    global_lock()
{}

/* Get current window size, in datagrams */
unsigned int Controller::window_size( void )
{
  std::lock_guard<std::mutex> lock(global_lock);
  // need bytes / 1500
  // curr_rtt_estimate and curr_bw_estimate use ms. Multiplying together gets bytes.

  unsigned int packets = cwnd_gain * (curr_rtt_estimate * curr_bw_estimate) / 1472;

  // if ( debug_ || true) {
  //   cerr << "At time " << timestamp_ms()
  //        << " window size is " << packets
  //        << " and curr_rtt_estimate * curr_bw_estimate = " << (curr_rtt_estimate * curr_bw_estimate)
  //        << endl;
  // }

  // if (timestamp_ms() % 4 == 0) // randomly set the window to 60 so we can estimate bw better TODO obviously needs to be better
  //   return 60;

  return (packets < 1) ? 1 : packets;
}

/* A datagram was sent */
void Controller::datagram_was_sent( const uint64_t sequence_number,
                                    /* of the sent datagram */
                                    const uint64_t send_timestamp )
/* in milliseconds */
{
  std::lock_guard<std::mutex> lock(global_lock);
  // TODO garbage collection
  packet_send_time[sequence_number] = send_timestamp;
  packet_delivered[sequence_number] = delivered_bytes;

  if ( debug_ ) {
    cerr << "At time " << send_timestamp
         << " sent datagram " << sequence_number << endl;
  }
}


template<typename T>
static T calcMaxInTimeWindow(uint64_t time_window, uint64_t curr_time,
                             map<uint64_t, T>& m) {

  auto left_time_window = m.lower_bound (curr_time - time_window);
  auto right_time_window = m.upper_bound (curr_time);

  T max_bw = numeric_limits<T>::min();
  for (auto it = left_time_window; it != right_time_window; ++it)
    if (it->second > max_bw)
      max_bw = it->second;

  return max_bw;
}

template<typename T>
static T calcMinInTimeWindow(uint64_t time_window, uint64_t curr_time,
                             map<uint64_t, T>& m) {

  auto left_time_window = m.lower_bound (curr_time - time_window);
  auto right_time_window = m.upper_bound (curr_time);

  T min_rtt = numeric_limits<T>::max();
  for (auto it = left_time_window; it != right_time_window; ++it)
    if (it->second < min_rtt)
      min_rtt = it->second;

  return min_rtt;
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
  std::lock_guard<std::mutex> lock(global_lock);
  // TODO garbage collection
  packet_ack_time[sequence_number_acked] = timestamp_ack_received;
  packet_ack_sent_time[sequence_number_acked] = recv_timestamp_acked;


  cerr << "--------------------------------------------" << endl;
  cerr << "Packet sent time: " << send_timestamp_acked << endl;

  uint64_t rtt_est = timestamp_ack_received -
                     send_timestamp_acked; // convert to ms
  cerr << "RTT_est (ms): " << rtt_est << endl;
  rtt_estimates[timestamp_ack_received] = rtt_est;
  // TODO change 200 here
  curr_rtt_estimate = calcMinInTimeWindow(rtt_time_window, timestamp_ack_received, rtt_estimates);

  // TODO incorporate one-way delay instead? Look into Kleinrock paper????

  // TODO let sharp changes in delivery rate make the time window smaller?

  // TODO check 1472
  // NOTE that bw_estimates is in bytes/ms
  delivered_bytes += 1472;
  double bw_est = (delivered_bytes - packet_delivered[sequence_number_acked]) /
                  ((double)rtt_est);
  bw_estimates[timestamp_ack_received] = bw_est;
  cerr << "Delivered between ACK and now: " << (delivered_bytes - packet_delivered[sequence_number_acked]) << endl;
  cerr << "BW_est (Mbps): " << bw_est * 8 / 1000.0 << endl;


  // TODO does not update when ACKs are not being received?
  curr_bw_estimate = calcMaxInTimeWindow(bw_time_window, timestamp_ack_received, bw_estimates);

  cerr << "Curr bw estimate (Mbps): " << (curr_bw_estimate * 8 / 1000.0) << endl;
  cerr << "Curr rtt estimate (ms): " << (curr_rtt_estimate) << endl;


  if ( debug_ || true) {
    cerr << "At time " << timestamp_ack_received
         << " received ack for datagram " << sequence_number_acked
         << " (send @ time " << send_timestamp_acked
         << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
         << endl;
  }

  // screw with pacing_gain

  // update phase if necessary
  if (timestamp_ack_received >= time_to_change_phase) {
    phase ++;
    // change phase curr_rtt_estimate ms away from now (TODO maybe drain phase should get extra time if Probe phase got extra time?)
    time_to_change_phase = timestamp_ack_received + curr_rtt_estimate;
  }

  // update pacing_gain based on phase
  if (phase == 0) {
    // pacing_gain goes up
    pacing_gain = 1.25; // TODO constant
  } else if (phase == 1) {
    // TODO may not want to do if bandwidth estimate increases
    pacing_gain = 0.75;
  } else if (phase <= 6) { // TODO constant
    pacing_gain = 1;
  } else {
    phase = 0;
  }
}
void Controller::notify_timeout( void ) {
  std::lock_guard<std::mutex> lock(global_lock);
}
/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms( void )
{
  std::lock_guard<std::mutex> lock(global_lock);
  return 1000; /* timeout of one second */
}

double Controller::get_bw_estimate()
{
  std::lock_guard<std::mutex> lock(global_lock);
  return curr_bw_estimate;
}

double Controller::get_pacing_gain()
{
  std::lock_guard<std::mutex> lock(global_lock);
  return pacing_gain;
}