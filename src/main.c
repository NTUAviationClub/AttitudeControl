#include "pid.h"
#include "rc_filter.h"
#include "stdio.h"

void RC_Test(void) {
  DTYPE dt = 0.01;

  FT mf;
  mf.size = RC_MEMORY_SIZE;
  mf.last_pos = 0;
  mf.last_time = 0.0f;
  mf.current_input = 0;
  mf.current_output = 0;
  DTYPE _last_input[mf.size];
  DTYPE _last_output[mf.size];
  DTYPE _param[RC_PARAM_SIZE];
  mf.last_input = _last_input;
  mf.last_output = _last_output;
  mf.param = _param;
  mf.param[RC_CUTOFF] = 1;
  // initialize
  for (int i = 0; i < mf.size; i++) {
    _last_input[i] = 0;
    _last_output[i] = 0;
  }
  // start testing filter
  DTYPE in = 0.0f;
  for (int i = 0; i < 100; i++) {
    DTYPE _now = i * dt;
    if (_now > 0.0f) {
      in = 1.0f;
    }
    mf.current_input = in;
    rc_update(_now, &mf);
    if ((i % 1) == 0) {
      printf("t: %lf, x: %lf\r\n", _now, mf.current_output);
    }
  }
}

DTYPE plant(DTYPE x, DTYPE u) { return -0.5f * x + 2 * u; }

void PID_Test(void) {
  DTYPE plant_dt = 0.001;
  DTYPE pid_dt = 0.005;
  DTYPE setpoint = 0.3;
  // set up lpf
  FT mf;
  mf.size = RC_MEMORY_SIZE;
  mf.last_pos = 0;
  mf.last_time = 0.0f;
  mf.current_input = 0;
  mf.current_output = 0;
  DTYPE mf_last_input[mf.size];
  DTYPE mf_last_output[mf.size];
  DTYPE mf_param[RC_PARAM_SIZE];
  mf.last_input = mf_last_input;
  mf.last_output = mf_last_output;
  mf.param = mf_param;
  mf.param[RC_CUTOFF] = 1;
  // initialize
  for (int i = 0; i < mf.size; i++) {
    mf_last_input[i] = 0;
    mf_last_output[i] = 0;
  }
  // set up pid
  FT mp;
  mp.size = RC_MEMORY_SIZE;
  mp.last_pos = 0;
  mp.last_time = 0.0f;
  mp.current_input = 0;
  mp.current_output = 0;
  DTYPE _last_input[mp.size];
  DTYPE _last_output[mp.size];
  DTYPE _param[PID_PARAM_SIZE];
  mp.last_input = _last_input;
  mp.last_output = _last_output;
  mp.param = _param;
  mp.param[PID_KP] = 2.0f;
  mp.param[PID_KI] = 0.2f;
  mp.param[PID_KD] = -1.0f;
  mp.param[PID_Ilimit] = 3.0f;
  mp.param[PID_SIdt] = 0.0f;
  for (int i = 0; i < mp.size; i++) {
    _last_input[i] = 0;
    _last_output[i] = 0;
  }
  // start the loop
  DTYPE x = 6.0f;
  DTYPE u = 0.0f;
  DTYPE last_pid_t = 0.0f;
  for (DTYPE t = 0; t < 2.0f; t = t + plant_dt) {
    x = x + plant_dt * plant(x, u);
    if ((t - last_pid_t) >= pid_dt) {
      mp.current_input = x;
      pid_update(t, setpoint, &mp, PID_NOT_QUEUE, &mf);
      u = mp.current_output;
      printf("t: %lf, x: %lf, u: %lf \r\n", t, x, u);
      last_pid_t = t;
    }
  }
}

int main() {
  // RC_Test();
  PID_Test();
  return 0;
}