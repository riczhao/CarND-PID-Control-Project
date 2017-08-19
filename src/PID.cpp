#include "PID.h"
#include <iostream>
#include <chrono>
#include <stdio.h>
#include <math.h>


using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double p, double i, double d) {
  Kp = p;
  Ki = i;
  Kd = d;
  prev_cte = 0.;
  first_step = true;
  sum_cte = 0;
  steps = 0;
  err = 0;
}

double PID::UpdateError(double cte) {
  double Sp, Si, Sd;
  Sp = -Kp * cte;

  if (first_step) {
    prev_cte = cte;
    first_step = false;
  }

  steps += 1;
  Sd = -Kd * (cte - prev_cte);

  sum_cte = (avg_win * sum_cte + cte) / (avg_win + 1);
  Si = -Ki * sum_cte;

  err += cte * cte;

  return Sp + Si + Sd;
}

double PID::TotalError() {
  return err/steps;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

Twiddle::Twiddle(PID *_pid, double p_[], double dp_[], int max_steps_) {
  pid = _pid;
  for (int i=0; i<3; i++) {
    p[i] = p_[i];
    dp[i] = dp_[i];
  }
  best_err = 1.E200;
  max_steps = max_steps_;
  state = -1;
  p_i = 0;

  pid->Init(p[0], p[1], p[2]);
}

void Twiddle::loop_start(void) {
   p[p_i] += dp[p_i];
   pid->Init(p[0], p[1], p[2]);
   state = 0;
}

void Twiddle::print_err(double err, char *prefix) {
  cout << prefix << " err " << err <<
    " p " << p[0] << "," << p[1] << "," << p[2] <<
    " dp " << dp[0] << "," << dp[1] << "," << dp[2] <<
    " p_i " << p_i << " state " << state << endl;
}

extern bool twiddle_drive;
void Twiddle::updateError(double cte, bool &need_reset, double &out_value) {
  out_value = pid->UpdateError(cte);
  need_reset = false;

  double err = pid->TotalError();
  /* off the road */
  if (twiddle_drive && fabs(cte) > 4.)
    err = 1E100;
  else if (pid->steps < max_steps)
    return;

  print_err(err);
  need_reset = true;

  switch (state) {
  case -1:
    best_err = err;
    print_err(err, "best");
    save_best_p();
    p_i = 0;
    loop_start();
    break;
  case 0:
    if (err < best_err) {
      best_err = err;
      print_err(err, "best");
      save_best_p();
      next_p();
      loop_start();
    } else {
      p[p_i] -= 2*dp[p_i];
      pid->Init(p[0], p[1], p[2]);
      state = 1;
    }
    break;
  case 1:
    if (err < best_err) {
      best_err = err;
      print_err(err, "best");
      save_best_p();
    } else {
      p[p_i] += dp[p_i];
      dp[p_i] *= 0.9;
    }
    next_p();
    loop_start();
    break;
  }
}
