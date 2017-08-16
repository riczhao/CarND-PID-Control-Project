#include "PID.h"

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
}

void PID::UpdateError(double cte) {
}

double PID::TotalError() {
}

void PID::step(double cte, double speed, double angle, double& steer_value)
{
  double Sp, Si, Sd;
  Sp = -Kp * cte;

  if (first_step) {
    prev_cte = cte;
    first_step = false;
  }
  Sd = -Kd * (cte - prev_cte);

  sum_cte += cte;
  Si = -Ki * sum_cte;

  steer_value = Sp + Si + Sd;
  steer_value = steer_value > 1 ? 1 : steer_value;
  steer_value = steer_value < -1 ? -1 : steer_value;
}
