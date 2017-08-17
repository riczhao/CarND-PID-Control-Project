#include "PID.h"
#include <chrono>
#include <stdio.h>


using namespace std;
using namespace std::chrono;

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
  prev_timestamp = 0;
  first_step = true;
  sum_cte = 0;
  sum_time = 0;
  err = 0;
}

void PID::UpdateError(double cte) {
}

double PID::TotalError() {
  return err/(double)sum_time;
}

void PID::step(double cte, double speed, double angle, double& steer_value)
{
  double Sp, Si, Sd;
  Sp = -Kp * cte;

  if (first_step) {
    prev_cte = cte;
    prev_timestamp = duration_cast< milliseconds >(
           system_clock::now().time_since_epoch()).count();
    first_step = false;
  }
  long long ts = duration_cast< milliseconds >(
           system_clock::now().time_since_epoch()).count();
  long long dt = ts - prev_timestamp;
  printf("dt:%lld\n", dt);
  prev_timestamp = ts;
  dt = prev_cte == cte ? 1 : dt;
  sum_time += dt;

  Sd = -Kd * (cte - prev_cte) / dt;

  sum_cte += cte * dt;
  Si = -Ki * sum_cte;

  steer_value = Sp + Si + Sd;
  steer_value = steer_value > 1 ? 1 : steer_value;
  steer_value = steer_value < -1 ? -1 : steer_value;

  err += cte * cte;
}
