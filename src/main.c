#include "cascade_attitude_control.h"
#include "complimentary_filter.h"
#include "pid.h"
#include "rc_filter.h"
#include "stdio.h"

void RC_Test(void) {
  DTYPE dt = 0.01;

  FT mf;
  rc_init_malloc(&mf, 1);
  /*
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
  */
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
  FT mf;
  FT mp;
  /*
  // set up lpf
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
  */
  pid_init_malloc(&mp, &mf, 1, 2, 0.2, -1, 3);
  // start the loop
  DTYPE x = 0.0f;
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
    scal(&q_w, 0.5 * dt, &q_w);
    mul(&Ori, &q_w, &q_w);
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

Qtn qt_plant(DTYPE _dt, Qtn _x_k, DTYPE *_wx, DTYPE *_wy, DTYPE *_wz, DTYPE _cx,
             DTYPE _cy, DTYPE _cz) {
  *_wx = *_wx + _dt * plant(*_wx, _cx);
  *_wy = *_wy + _dt * plant(*_wy, _cy);
  *_wz = *_wz + _dt * plant(*_wz, _cz);
  Qtn w_qt = {.w = 0,
              .x = 0.5f * _dt * *_wx,
              .y = 0.5f * _dt * *_wy,
              .z = 0.5f * _dt * *_wz};
  Qtn dq, res;
  mul(&_x_k, &w_qt, &dq);
  // printf("in plant dq: %g, %g, %g, %g\r\n", dq.w, dq.x, dq.y, dq.z);
  add(&dq, &_x_k, &res);
  // printf("in plant x_k: %g, %g, %g, %g\r\n", _x_k.w, _x_k.x, _x_k.y, _x_k.z);
  // printf("in plant res: %g, %g, %g, %g\r\n", res.w, res.x, res.y, res.z);
  return res;
}

void CAC_Test(void) {
  DTYPE wx = 0, wy = 0, wz = 0, t = 0, dt = 0.01;
  DTYPE ar, ap, ay, gr = 0.2, gp = 0, gy = 0;
  DTYPE cx = 0, cy = 0, cz = 0;
  Qtn Ori = {.w = 1, .x = 0, .y = 0, .z = 0};
  cac_init();
  cac_set_attitude_control_tau(0.5);
  cac_set_rate_control_pid(CAC_ROLL, 2, 0.1, 0.1);
  cac_set_rate_control_pid(CAC_PITCH, 2, 0.1, 0.1);
  cac_set_rate_control_pid(CAC_YAW, 2, 0.1, 0.1);
  cac_set_rpy_setpoint(gr, gp, gy);
  while (t <= 1) {
    t += dt;
    Ori = qt_plant(dt, Ori, &wx, &wy, &wz, cx, cy, cz);
    cac_set_qtn_measured(&Ori);
    cac_set_gyr_estimate(wx, wy, wz);
    cac_update_attitude_control(t);
    cac_update_rate_control(t);
    cac_get_command(&cx, &cy, &cz);
    get_RPY(&Ori, &ar, &ap, &ay);
    Qtn gq;
    from_RPY(&gq, gr, gp, gy);
    // printf("desired rpy: %g, %g, %g actual: %g, %g, %g\r\n", gr, gp, gy, ar,
    // ap, ay);
    if (((int)(t / dt)) % 10 == 0) {
      DTYPE rs, ps, ys;
      cac_get_angular_rate_setpoint(&rs, &ps, &ys);
      printf("angular rate setpoint: %g, %g, %g\r\n", rs, ps, ys);
      printf("control: %g, %g, %g\r\n", cx, cy, cz);
      printf("angular rate: %g, %g, %g\r\n", wx, wy, wz);
      printf("desired q: %g, %g, %g, %g actual: %g, %g, %g, %g\r\n", gq.w, gq.x,
             gq.y, gq.z, Ori.w, Ori.x, Ori.y, Ori.z);
    }
  }
}

int main() {
  RC_Test();
  // PID_Test();
  // CF_Test();
  // CAC_Test();
  return 0;
}