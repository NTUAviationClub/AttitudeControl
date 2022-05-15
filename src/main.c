#include "complimentary_filter.h"
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

void CF_Test() {
  DTYPE alpha = 0.5, beta = 0.5;
  DTYPE wx = 0.1, wy = 0, wz = 0, gbx = 0.01, gby = 0, gbz = 0;
  Qtn Ori = {.w = 1, .x = 0, .y = 0, .z = 0};
  // Init
  DTYPE t = 0, dt = 0.01;
  cf_init(t, alpha, beta);
  while (t <= 1) {
    t += dt;
    Qtn q_w = {.w = 0, .x = wx, .y = wy, .z = wz};
    scal(&q_w, -0.5 * dt, &q_w);
    add(&Ori, &q_w, &Ori);
    normalize_inplace(&Ori);
    cf_update(t, &Ori, wx + gbx, wy + gby, wz + gbz);
    // cf_update(t, &Ori, wx, wy, wz);
    DTYPE qw, qx, qy, qz, bx, by, bz;
    cf_get_filtered_qtn(&qw, &qx, &qy, &qz);
    cf_get_bias(&bx, &by, &bz);
    printf("[Qtn] gt: %lf, %lf, %lf, %lf, filtered: %lf, %lf, %lf, %lf\r\n",
           Ori.w, Ori.x, Ori.y, Ori.z, qw, qx, qy, qz);
    printf("[Bias] gt: %lf, %lf, %lf, filtered: %lf, %lf, %lf\r\n", gbx, gby,
           gbz, bx, by, bz);
  }
}

int main() {
  // RC_Test();
  // PID_Test();
  CF_Test();
  return 0;
}