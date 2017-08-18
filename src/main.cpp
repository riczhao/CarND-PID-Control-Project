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

  double p[] = {1, 0, 0};
  double dp[] = {1, 1, 1};
  int measure_steps= 0;
  double best_err;

  int p_i = 0;
  int p_state = 0; // 0:check +dp 1:check -dp 

static PID pid_speed;

int main()
{
  uWS::Hub h;

  PID pid;

  // TODO: Initialize the pid variable.
  pid.Init(p[0], p[1], p[2]);

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
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
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
