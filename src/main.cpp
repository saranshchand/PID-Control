#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  PID pid_speed;
  bool twiddle = false;
  
  /**
   * TODO: Initialize the pid variable.
   */
  std::vector<double> p = {0.095639, 0.00088705, 0.80838}; //updated  twiddle start for best error
  std::vector<double> dp = {0.01, 0.0001, 0.1};
  
  double tolerance = 0.000000001;
  double best_cte_twiddle = 100000;
  int stage_twiddle = 0;
  int index_twiddle = 0;
  int steps = 0;
  double total_cte = 0;
  
  if (!twiddle)
  {
    //potential values tuned through twiddle
    //{0.133688, 0.00194405, 1.71359}
    pid.Init(0.105639, 0.00088705, 0.80838); 
    pid_speed.Init(0.104739, 0.00177705, 2.10838);
  }
  

  h.onMessage([&pid, &twiddle, &p, &dp, &tolerance, &best_cte_twiddle, &stage_twiddle, &index_twiddle, &steps, &total_cte, &pid_speed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          
          if (twiddle)
          {
            //insert code
            if (steps > 1400)
            {
              if (stage_twiddle == 0)
              {
                //first part of loop              
                best_cte_twiddle = total_cte;  
                p[index_twiddle] += dp[index_twiddle];
                stage_twiddle = 1;
              }
              else if (stage_twiddle == 1)
              {
                if (total_cte < best_cte_twiddle)
                {
                  best_cte_twiddle = total_cte;
                  dp[index_twiddle] *= 1.1;
                  index_twiddle = (index_twiddle+1)%3;
                  p[index_twiddle] += dp[index_twiddle];
                  stage_twiddle = 1; //not done with this area till we get best error
                }
                else
                {
                  p[index_twiddle] -= 2*dp[index_twiddle];
                  if (p[index_twiddle] < 0) //since we are decrementing, we don't want to accidentally make it negative
                  {
                    p[index_twiddle] = 0; //set it back to 0                                                                        
                  }
                  index_twiddle = (index_twiddle+1)%3;
                  stage_twiddle = 2;

                }
              }
              else if (stage_twiddle == 2)
              {
                if (total_cte < best_cte_twiddle)
                {
                  best_cte_twiddle = total_cte;
                  dp[index_twiddle] *= 1.1;
                  index_twiddle = (index_twiddle+1)%3;
                  p[index_twiddle] += dp[index_twiddle];
                  stage_twiddle = 1;                
                }
                else
                {
                  p[index_twiddle] += dp[index_twiddle];
                  dp[index_twiddle] *= 0.9;
                  index_twiddle = (index_twiddle+1)%3;
                  p[index_twiddle] += dp[index_twiddle];
                  stage_twiddle = 1;
                }

              }

              if ((std::accumulate(dp.begin(), dp.end(), 0) < tolerance) && (steps > 1500)) //basically never going to happen
              {
                //reset simulator to beginning 
                //steps = 0;
                //pid.Init(p[0], p[1], p[2]); 
                //stage_twiddle = 0;
                std::cout << "Final gains " << p[0] << ", " << p[1] << ", " << p[2] << std::endl; 
                
              }
              else
              {
                std::cout << "gains " << p[0] << ", " << p[1] << ", " << p[2] << "Error: " << total_cte << std::endl;
                steps = 0;
                total_cte = 0;
                std::string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
              }              
            }
            else
            {
              if (steps == 0)
              {
                pid.Init(p[0], p[1], p[2]);  
                pid_speed.Init(0.104739, 0.00177705, 2.10838);
              }
              steps++;            
              pid.UpdateError(cte);
              pid_speed.UpdateError(speed - 30);
              steer_value = pid.TotalError();
              total_cte += pow(cte, 2);
              total_cte += pow(speed - 30, 2);
              //std::cout << total_cte << std::endl;

              json msgJson;
              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = pid_speed.TotalError();
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              //std::cout << msg << std::endl;
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);              
            }
            
          }
          else
          {
            pid.UpdateError(cte);
            steer_value = pid.TotalError();
            //+ std::abs(20*steer_value)
            pid_speed.UpdateError(speed - 30.0);
            

            // DEBUG
            //std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
            //          << std::endl;

            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = pid_speed.TotalError();;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            //std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            
          }
          
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}