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

// TODO FIXME use downward slope to adjust pacing gain
// TODO FIXME don't probe upwards during downwards slope

static double curr_bw_slope_estimate = 0.0;
// static double prev_bw_sample = 0.0;
// static uint64_t prev_bw_sample_timestamp = 0;
static uint64_t curr_bw_estimate_timestamp = 0;


static double extra_gain = 1.0;

/* Get current window size, in datagrams */
unsigned int Controller::window_size( void )
{
  std::lock_guard<std::mutex> lock(global_lock);
  // need bytes / 1500
  // curr_rtt_estimate and curr_bw_estimate use ms. Multiplying together gets bytes.
  unsigned int packets;

  packets = cwnd_gain * (curr_rtt_estimate * curr_bw_estimate) / 1472;

  // if ( debug_ || true) {
  //   cerr << "At time " << timestamp_ms()
  //        << " window size is " << packets
  //        << " and curr_rtt_estimate * curr_bw_estimate = " << (curr_rtt_estimate * curr_bw_estimate)
  //        << endl;
  // }

  // if (timestamp_ms() % 4 == 0) // randomly set the window to 60 so we can estimate bw better TODO obviously needs to be better
  //   return 60;

  // TODO FIXME try turning off start_up

  if (packets <= 1 && !start_up) { // TODO maybe set 1 to be higher, or judge based on bandwidth
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
                             multimap<uint64_t, T>& m,
                             uint64_t& ts) {

  auto left_time_window = (curr_time > time_window) ? m.lower_bound (curr_time - time_window) : m.begin();
  auto right_time_window = m.upper_bound (curr_time);

  T max_bw = numeric_limits<T>::min();
  for (auto it = left_time_window; it != right_time_window; ++it)
    if (it->second > max_bw) {
      max_bw = it->second;
      ts = it->first;
    }

  return max_bw;
}

template<typename T>
static T calcMinInTimeWindow(uint64_t time_window, uint64_t curr_time,
                             multimap<uint64_t, T>& m) {

  auto left_time_window = (curr_time > time_window) ? m.lower_bound (curr_time - time_window) : m.begin();
  auto right_time_window = m.upper_bound (curr_time);

  T min_rtt = numeric_limits<T>::max();
  for (auto it = left_time_window; it != right_time_window; ++it) {
    if (it->second < min_rtt)
      min_rtt = it->second;
  }

  return min_rtt;
}


// credit to http://stackoverflow.com/questions/18939869/how-to-get-the-slope-of-a-linear-regression-line-using-c
double Controller::bw_slope() {
  cerr << "Last bw estimate timestamp: " << curr_bw_estimate_timestamp << endl;
  auto last_bw_estimate = bw_estimates.lower_bound(curr_bw_estimate_timestamp);
  auto end = bw_estimates.end();

  double avgX = 0.0, avgY = 0.0;

  uint64_t n = 0;

  for (auto it = last_bw_estimate; it != end; ++it) {
    avgX += (it->first - curr_bw_estimate_timestamp); // free to shift, only need slope
    avgY += it->second;
    n++;
  }

  avgX /= n;
  avgY /= n;

  cerr << "AvgX: " << avgX << ", avgY: " << avgY << endl;

  double numerator = 0.0;
  double denominator = 0.0;

  for (auto it = last_bw_estimate; it != end; ++it) {
    numerator += (it->first - avgX) * (it->second - avgY);
    denominator += (it->first - avgX) * (it->first - avgX);
  }

  if (denominator == 0) {
    return 0.0; // don't know slope yet
  }

  return numerator / denominator;
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

  rtt_estimates.emplace(timestamp_ack_received, rtt_est);
  // TODO change 200 here
  curr_rtt_estimate = calcMinInTimeWindow(rtt_time_window, timestamp_ack_received, rtt_estimates);

  // TODO incorporate one-way delay instead? Look into Kleinrock paper????

  // TODO let sharp changes in delivery rate make the time window smaller?

  // TODO check 1472
  // NOTE that bw_estimates is in bytes/ms
  delivered_bytes += 1472;
  double bw_est = (delivered_bytes - packet_delivered[sequence_number_acked]) /
                  ((double)rtt_est);
  bw_estimates.emplace(timestamp_ack_received, bw_est);
  cerr << "Delivered between ACK and now: " << (delivered_bytes - packet_delivered[sequence_number_acked]) << endl;
  cerr << "BW_est (Mbps): " << bw_est * 8 / 1000.0 << endl;


  // TODO REMOVE silly slope calculation doesn't work very well
  // estimate gradient
  // if (timestamp_ack_received != prev_bw_sample_timestamp) {
  //   double new_slope = (bw_est - prev_bw_sample) / (timestamp_ack_received - prev_bw_sample_timestamp);
  //   // update gradient
  //   curr_bw_slope_estimate = 0.8 * curr_bw_slope_estimate + 0.2 * new_slope;
  //   // update prev values
  //   prev_bw_sample = bw_est;
  //   prev_bw_sample_timestamp = timestamp_ack_received;
  // }

  // TODO does not update when ACKs are not being received?
  // // update with max on positive slope, with larger time window
  // if (curr_bw_slope_estimate > 0)
  curr_bw_estimate = calcMaxInTimeWindow(bw_time_window, timestamp_ack_received, bw_estimates, curr_bw_estimate_timestamp);
  // else // on negative slope, update with min and more frequently // TODO FIXME does this perpetuate negative slop which will prevent us from probing?
  //   curr_bw_estimate = calcMaxInTimeWindow(90, timestamp_ack_received, bw_estimates);


  // recalculate slope
  curr_bw_slope_estimate = bw_slope();


  cerr << "Current estimation of slope: " << curr_bw_slope_estimate << endl;
  cerr << "Curr bw estimate (Mbps): " << (curr_bw_estimate * 8 / 1000.0) << endl;
  cerr << "Curr rtt estimate (ms): " << (curr_rtt_estimate) << endl;



  if ( debug_ || true) {
    cerr << "At time " << timestamp_ack_received
         << " received ack for datagram " << sequence_number_acked
         << " (send @ time " << send_timestamp_acked
         << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
         << endl;

  }

  // TODO delay-based for low-bandwidth?
  // TODO if bandwidth decreasing, adjust pacing_gain according to the bandwidth gradient and a multiplication of the amount of time

  // screw with pacing_gain

  if (start_up) {
    cerr << "Received ACK in startup!" << endl;
    if (timestamp_ack_received >= time_to_change_phase) {
      time_to_change_phase = timestamp_ack_received + curr_rtt_estimate;

      // after one RTTProp, check to see if we have reached a plateau in bandwidth

      // pacing_gain = pacing_gain * 2 / 0.6931471;
      // cwnd_gain = cwnd_gain * 2 / 0.6931471;
      if (1.25 * prev_phase_bw_estimate > curr_bw_estimate)
        startup_bw_counter++;
      else
        startup_bw_counter = 0; // else reset

      if (startup_bw_counter >= 3) { // TODO maybe change
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
      cerr << "Received ACK in startup drain! RTT_est = " << rtt_est << ", curr rtt estimate = " << curr_rtt_estimate << endl;
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
        // change phase curr_rtt_estimate ms away from now (TODO maybe drain phase should get extra time if Probe phase got extra time?)
        time_to_change_phase = timestamp_ack_received + curr_rtt_estimate;
      }

      // if rtt becomes too long, start draining immediately (fallback for everything else)
      if (rtt_est > curr_rtt_estimate * 2.0 && phase != 1) {
        cerr << "immediately start draining!!!!!!!!!! RTT very high!" << endl;
        phase = 1;
        time_to_change_phase = timestamp_ack_received + curr_rtt_estimate;
        extra_gain = 1.0;
      }
      // update pacing_gain based on phase

      // do not transition back to probing phase if our bandwidth is decreasing
      // TODO FIXME together with the pacing_gain thing with low slope below, we may never be able to recover from the drop in pacing_gain
      if (phase > 6 && curr_bw_slope_estimate > -5.0)  {// TODO constant
        phase = 0;
      }

      if (phase == 0) {
        phase = 0;
        // pacing_gain goes up
        cout << "Extra gain: " << extra_gain << endl;
        pacing_gain = extra_gain * 1.25; // TODO constant
        cwnd_gain = extra_gain * 1.5;
      } else if (phase == 1) {
        // TODO may not want to do if bandwidth estimate increases
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

  // TODO FIXME need to shift on major shifts of slope, especially downward slop
  // TODO FIXME current starting values 320 and curr_bw_estimate really make no sense,
  // currently just lowers pacing_gain if slope is negative.
  if (!start_up && !start_up_drain && phase > 1 && curr_bw_slope_estimate < 1) {
    // based on slope
    uint64_t time_since_bw_estimate = (timestamp_ms() - curr_bw_estimate_timestamp);
    return 1.0 + (curr_bw_slope_estimate * time_since_bw_estimate) / curr_bw_estimate;
    // return 0.7;
  }


  return pacing_gain;
}