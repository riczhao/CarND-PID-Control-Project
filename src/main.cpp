#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

bool initialized = false;

static PID pid_s;
static double target_speed = 10;
static bool twiddle_speed = false;
static double p_s[] = {4.48205,0.0164232,0.232212};
static double dp_s[] = {0.0309031,0.0164232,0.0225284};
static Twiddle twiddle_s(&pid_s, p_s, dp_s, 400);

static PID pid_d;
bool twiddle_drive = false;
static double p_d[] = {1.17422,-0.00135816,-0.0723838};
static double dp_d[] = {0.18,0.06561,0.09};
static Twiddle twiddle_d(&pid_d, p_d, dp_d, 1000);
static int max_steps_d;

int main()
{
  uWS::Hub h;
  PID pid;

  // TODO: Initialize the pid variable.
  pid_s.Init(p_s[0], p_s[1], p_s[2]);
  pid_d.Init(p_d[0], p_d[1], p_d[2]);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value = 0;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          double set_speed = 0;
          double err_speed = speed - target_speed;
          bool need_reset = false;
          if (initialized && twiddle_speed) {
            twiddle_s.updateError(err_speed, need_reset, set_speed);
          } else {
            set_speed = pid_s.UpdateError(err_speed);
          }

          if (initialized && twiddle_drive) {
            if (!max_steps_d) {
              if (false/*pid_d.steps > 1000 && target_speed < 80*/) {
                need_reset = true;
                target_speed += 1;
                pid_d.Init(p_d[0], p_d[1], p_d[2]);
                printf("increase speed to %f\n", target_speed);
              } else if (pid_d.steps > 10000 || fabs(cte) > 2.) { /* at road boundary */
                max_steps_d = pid_d.steps;
                twiddle_d = Twiddle(&pid_d, p_d, dp_d, max_steps_d);
                need_reset = true;
                printf("reinitialized twiddle_d maxstep %d\n", max_steps_d);
              } else { /* still measuring max steps */
                steer_value = pid_d.UpdateError(cte);
              }
            } else {
              twiddle_d.updateError(cte, need_reset, steer_value);
              if (twiddle_d.best_err < 0.3) {
                for (int i=0; i<3; i++) {
                  p_d[i] = twiddle_d.best_p[i];
                }
                max_steps_d = 0;
                need_reset = true;
                pid_d.Init(p_d[0], p_d[1], p_d[2]);
                printf("recalculating max steps\n");
              }
            }
          } else {
            steer_value = pid_d.UpdateError(cte);
          }

          if (!initialized || need_reset) {
              std::string msg = "42[\"reset\",{}]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              if (!twiddle_speed)
                pid_s.Init(p_s[0], p_s[1], p_s[2]);
              if (!twiddle_drive)
                pid_d.Init(p_d[0], p_d[1], p_d[2]);
              initialized = true;
          } else {
              json msgJson;
              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = set_speed;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              //std::cout << msg << std::endl;
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }

#if 0
          pid.step(cte, speed, angle, steer_value);
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          if (fabs(cte) > 4. && !measure_steps) {
            std::string msg = "42[\"reset\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            printf("%s:%d measure_steps %d\n", __func__, __LINE__, measure_steps);
            measure_steps = pid.steps;
            printf("%s:%d measure_steps %d\n", __func__, __LINE__, measure_steps);
            best_err = pid.TotalError();
            std::cout << "p:" << p[0]<<"," <<p[1]<<","<<p[2] << " dp:" << dp[0]<<"," <<dp[1]<<","<<dp[2]<<std::endl;
            std::cout << "p:" << p[0]<<"," <<p[1]<<","<<p[2]<<" err:"<<best_err
               << " p_i:" << p_i <<" p_state:" << p_state << std::endl;
            std::cout << "dp:" << dp[0]<<"," <<dp[1]<<","<<dp[2]<<std::endl;
            p_state = 0;
            p[p_i] += dp[p_i];
            pid.Init(p[0], p[1], p[2]);
            std::cout << "measure_steps: "<<measure_steps<<std::endl;
          } if (measure_steps&& measure_steps <= pid.steps) {
            double err = pid.TotalError();
            std::cout << "p:" << p[0]<<"," <<p[1]<<","<<p[2]<<" err:"<<err
               << " p_i:" << p_i <<" p_state:" << p_state << std::endl;
            std::cout << "dp:" << dp[0]<<"," <<dp[1]<<","<<dp[2]<<std::endl;
            switch (p_state) {
            case 0:
              if (err < best_err) {
                std::cout << "p:" << p[0]<<"," <<p[1]<<","<<p[2] << " dp:" << dp[0]<<"," <<dp[1]<<","<<dp[2]<<std::endl;
                best_err = err;
                p_i++;
                p_i = p_i > 2 ? 0 : p_i;
                p_state = 0;
                p[p_i] += dp[p_i];
                if (best_err < 0.1) {
                  dp[0] = dp[1] = dp[2] = 0.1;
                  measure_steps = 0.;
                  p_i = p_state = 0;
                }
              } else {
                p[p_i] -= 2*dp[p_i];
                p_state = 1;
              }
              break;
            case 1:
              if (err < best_err) {
                std::cout << "p:" << p[0]<<"," <<p[1]<<","<<p[2] << " dp:" << dp[0]<<"," <<dp[1]<<","<<dp[2]<<std::endl;
                best_err = err;
              } else {
                p[p_i] += dp[p_i];
                dp[p_i] *= 0.9;
              }
              p_i++;
              p_i = p_i > 2 ? 0 : p_i;
              p_state = 0;
              p[p_i] += dp[p_i];

              if (best_err < 0.1) {
                dp[0] = dp[1] = dp[2] = 0.1;
                measure_steps = 0;
                p_i = p_state = 0;
              }
 
              break;
            }
            std::string msg = "42[\"reset\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            pid.Init(p[0], p[1], p[2]);
          } else {
            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = 0.3;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
#endif
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
