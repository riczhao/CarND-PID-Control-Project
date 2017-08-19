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
  const int avg_win = 20;
  double err;
};

/////////////////////////////////////////////////////////////////////////////////

/* we need to use event driven twiddle class */
class Twiddle {
public:
  PID *pid;
  double p[3];
  double dp[3];
  double best_p[3], best_dp[3];
  double best_err;
  int max_steps;
  Twiddle(PID *_pid, double p_[], double dp_[], int max_steps_);
  void updateError(double cte, bool &need_reset, double &out_value);
private:
  int state; // -1:no err received 0:+dp 1:-dp
  int p_i;
  void next_p(void) {
    p_i += 1;
    p_i %= 3;
  }
  void loop_start(void); /* start of the corresponding original twiddle loop */
  void print_err(double err, char *prefix="");
  void save_best_p(void) {
    for (int i=0; i<3; i++) {
      best_p[i] = p[i];
      best_dp[i] = dp[i];
    }
  }
};

#endif /* PID_H */
