#include <iostream>

#include "controller.hh"
#include "timestamp.hh"

#include <limits>

using namespace std;

// TODO a lot of shit in this controller is not thread-safe. That shall be fixed
// TODO need to sleep the sender when window not open

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ), bw_time_window(320), rtt_time_window(30000),
    curr_rtt_estimate(40), curr_bw_estimate(5.0), delivered_bytes(0),
    pacing_gain(1.0), cwnd_gain(2),
    phase(0), time_to_change_phase(40),
    packet_send_time(), packet_ack_time(), packet_ack_sent_time(),
    rtt_estimates(), bw_estimates(), packet_delivered(),
    global_lock()
{}

static bool start_up = false;
static double prev_phase_bw_estimate;
static int startup_bw_counter;
static bool start_up_drain = false;

static double curr_bw_slope_estimate = 0.0;
static double prev_bw_sample = 0.0;
static uint64_t prev_bw_sample_timestamp = 0;


static double extra_gain = 1.0;

/* Get current window size, in datagrams */
unsigned int Controller::window_size( void )
{
  std::lock_guard<std::mutex> lock(global_lock);
  // need bytes / 1500
  // curr_rtt_estimate and curr_bw_estimate use ms. Multiplying together gets bytes.
  unsigned int packets;

  packets = cwnd_gain * (curr_rtt_estimate * curr_bw_estimate) / 1472;

  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
         << " window size is " << packets
         << " and curr_rtt_estimate * curr_bw_estimate = " << (curr_rtt_estimate * curr_bw_estimate)
         << endl;
  }


  if (packets <= 1 && !start_up) {
    // low window--transition into startup phase.
    time_to_change_phase = timestamp_ms() + curr_rtt_estimate;
    prev_phase_bw_estimate = curr_bw_estimate;
    start_up = true;
    startup_bw_counter = 0;
    pacing_gain = 1.5 / 0.6931471;
    cwnd_gain = 1.5 / 0.6931471;
    extra_gain = 1.0;
  }

  return packets < 1 ? 1 : packets;
}

/* A datagram was sent */
void Controller::datagram_was_sent( const uint64_t sequence_number,
                                    /* of the sent datagram */
                                    const uint64_t send_timestamp )
/* in milliseconds */
{
  std::lock_guard<std::mutex> lock(global_lock);
  packet_send_time[sequence_number] = send_timestamp;
  packet_delivered[sequence_number] = delivered_bytes;

  if ( debug_ ) {
    cerr << "At time " << send_timestamp
         << " sent datagram " << sequence_number << endl;
  }
}


template<typename T>
static T calcMaxInTimeWindow(uint64_t time_window, uint64_t curr_time,
                             multimap<uint64_t, T>& m) {

  auto left_time_window = (curr_time > time_window) ? m.lower_bound (curr_time - time_window) : m.begin();
  auto right_time_window = m.upper_bound (curr_time);

  T max_bw = numeric_limits<T>::min();
  for (auto it = left_time_window; it != right_time_window; ++it)
    if (it->second > max_bw)
      max_bw = it->second;

  return max_bw;
}

template<typename T>
static T calcMinInTimeWindow(uint64_t time_window, uint64_t curr_time,
                             multimap<uint64_t, T>& m) {

  auto left_time_window = (curr_time > time_window) ? m.lower_bound (curr_time - time_window) : m.begin();
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
  packet_ack_time[sequence_number_acked] = timestamp_ack_received;
  packet_ack_sent_time[sequence_number_acked] = recv_timestamp_acked;


  uint64_t rtt_est = timestamp_ack_received -
                     send_timestamp_acked; // convert to ms

  rtt_estimates.emplace(timestamp_ack_received, rtt_est);
  curr_rtt_estimate = calcMinInTimeWindow(rtt_time_window, timestamp_ack_received, rtt_estimates);

  // NOTE that bw_estimates is in bytes/ms
  delivered_bytes += 1472;
  double bw_est = (delivered_bytes - packet_delivered[sequence_number_acked]) /
                  ((double)rtt_est);
  bw_estimates.emplace(timestamp_ack_received, bw_est);


  // estimate gradient
  if (timestamp_ack_received != prev_bw_sample_timestamp) {
    double new_slope = (bw_est - prev_bw_sample) / (timestamp_ack_received - prev_bw_sample_timestamp);
    // update gradient
    curr_bw_slope_estimate = 0.8 * curr_bw_slope_estimate + 0.2 * new_slope;
    // update prev values
    prev_bw_sample = bw_est;
    prev_bw_sample_timestamp = timestamp_ack_received;
  }

  curr_bw_estimate = calcMaxInTimeWindow(bw_time_window, timestamp_ack_received, bw_estimates);

  if ( debug_) {
    cerr << "At time " << timestamp_ack_received
         << " received ack for datagram " << sequence_number_acked
         << " (send @ time " << send_timestamp_acked
         << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
         << endl;
  }

  // screw with pacing_gain

  if (start_up) {
    if (timestamp_ack_received >= time_to_change_phase) {
      time_to_change_phase = timestamp_ack_received + curr_rtt_estimate;

      // after one RTTProp, check to see if we have reached a plateau in bandwidth

      if (1.25 * prev_phase_bw_estimate > curr_bw_estimate)
        startup_bw_counter++;
      else
        startup_bw_counter = 0; // else reset

      if (startup_bw_counter >= 3) {
        // exit startup if we have hit a plateau
        start_up = false;
        start_up_drain = true;
        pacing_gain = 1 / pacing_gain;
        cwnd_gain = 1.00;
        time_to_change_phase = timestamp_ack_received + startup_bw_counter * curr_rtt_estimate;
      }
      // record bw_estimate
      prev_phase_bw_estimate = curr_bw_estimate;
    }
  }
  else {
    if (start_up_drain) {
      // stay in drain until RTT=RTTprop, then reset pacing gain and time_to_change_phase
      if (1.1 * curr_rtt_estimate > rtt_est) {
        start_up_drain = false;
        pacing_gain = 1.0;
        time_to_change_phase = timestamp_ack_received + curr_rtt_estimate;
        prev_phase_bw_estimate = curr_bw_estimate;
      }
    }
    else { // if not in startup or startup drain, do steady state
      // update phase if necessary

      if (timestamp_ack_received >= time_to_change_phase) {
        if (!(phase == 0 && 1.1 * prev_phase_bw_estimate < curr_bw_estimate)) {
          phase ++;
          extra_gain = 1.0;
        } else {
          // extra_gain += 0.1;
        }
        prev_phase_bw_estimate = curr_bw_estimate;
        // change phase curr_rtt_estimate ms away from now
        time_to_change_phase = timestamp_ack_received + curr_rtt_estimate;
      }

      // if rtt becomes too long, start draining immediately (fallback for everything else)
      if (rtt_est > curr_rtt_estimate * 3.0 && phase != 1) {
        phase = 1;
        time_to_change_phase = timestamp_ack_received + curr_rtt_estimate;
        extra_gain = 1.0;
      }
      // update pacing_gain based on phase

      // do not transition back to probing phase if our bandwidth is decreasing
      if (phase > 6 && curr_bw_slope_estimate > 0)  {
        phase = 0;
      }

      if (phase == 0) {
        phase = 0;
        // pacing_gain goes up
        pacing_gain = extra_gain * 1.25;
        cwnd_gain = extra_gain * 1.5;
      } else if (phase == 1) {
        pacing_gain = 0.75;
        cwnd_gain = 0.8;
      } else {
        pacing_gain = 1;
        cwnd_gain = 1.25;
      }
    }
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
  return 500; /* timeout of one second */
}

double Controller::get_bw_estimate()
{
  std::lock_guard<std::mutex> lock(global_lock);
  return curr_bw_estimate;
}

double Controller::get_pacing_gain()
{
  std::lock_guard<std::mutex> lock(global_lock);

  // currently just lowers pacing_gain if slope is negative.
  if (!start_up && !start_up_drain && phase > 1 && curr_bw_slope_estimate < 1) {
    // based on slope
    return 0.7;
  }


  return pacing_gain;
}