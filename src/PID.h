#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  double UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  void step(double cte, double speed, double angle, double& steer_value);
  int steps;
private:
  bool first_step;
  double prev_cte;
  double sum_cte;
  double err;
};

/////////////////////////////////////////////////////////////////////////////////

/* we need to use event driven twiddle class */
class Twiddle {
public:
  PID *pid;
  double p[3];
  double dp[3]
  double best_err;
  int max_steps;
  Twiddle(double p1, double p2, double p3, double dp1, double dp2, double dp3);
  void updateError(double cte, bool &need_reset);
private:
  int state; // -1:no err received 0:+dp 1:-dp
  int p_i;
  void next_p(void) {
    p_i += 1;
    p_i %= 3;
  }
  void loop_start(void); /* start of the corresponding original twiddle loop */
};

#endif /* PID_H */
