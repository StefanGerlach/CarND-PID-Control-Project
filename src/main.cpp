#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"

#include <math.h>
#include <numeric>
#include <vector>


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

int main()
{
  uWS::Hub h;

  PID pid;

  // Initialize the pid variable.
  // Just init with values from Lesson 17.
  std::vector<double> param({0.107141, -1.18156e-05, 0.268861});
  std::vector<double> last_param(param);

  std::vector<double> delta_param({0.1, 0.1, 0.1});
  std::vector<int> twiddle_direction({0, 0, 0});

  pid.Init(param[0], param[1], param[2]);

  // Initialize the max error
  // If this error is exceeded, the simulator is reseted
  double max_error = 2.0;

  // Twiddle?
  bool twiddle_mode = true;
  int twiddle_index = 0;

  // Init best error
  double best_error = -1.0;
  double twid_tolerance = 0.05;

  // Define the number of frames to use for twiddling
  unsigned long twiddle_steps = 4096;
  unsigned long current_step = 0;

  h.onMessage([&pid, // pass everything needed by reference into lambda function
               &max_error,
               &best_error,
               &twiddle_mode,
               &twiddle_index,
               &twiddle_direction,
               &twiddle_steps,
               &twid_tolerance,
               &current_step,
               &param,
               &last_param,
               &delta_param](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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

          /*
          * Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          // Increment current step
          if(twiddle_mode)
            current_step++;

          // Update errors in PID-Controller
          pid.UpdateError(cte);

          // Get updated steering value
          double steer_value = pid.GetUpdatedSteering();
          
          // Get the last error
          double last_error = pid.GetLastError();
          // std::cout << "Last Error : " << pid.GetLastError() << std::endl;

          // Create the JSON message for simulator
          json msgJson;
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          bool reset_simulator = twiddle_mode && (current_step > twiddle_steps);
          if(twiddle_mode) {

            // Check if twiddle is DONE?
            double sum_of_deltas = static_cast<double>(std::accumulate(delta_param.begin(), delta_param.end(), 0.0));
            if(sum_of_deltas < twid_tolerance) {
              // Twiddle is done!
              std::cout << "Twiddle finished! Params:" << std::endl;
              for(const auto& p : param)
                std::cout << p << std::endl;

              exit(0);
            }
          }

          // Reset simulator if error is too large
          if((fabs(last_error) > max_error) || reset_simulator) {

            if(twiddle_mode && (current_step > 10)) {
              // This iteration has an average error of:
              double avg_err = fabs(pid.TotalError() / static_cast<double>(current_step));
              if(best_error < 0)
                best_error = avg_err;

              bool new_record = false;
              bool new_param = false;

              std::cout << "This Twiddle-run had abs-avg-error: " << avg_err << " best_error: "<< best_error << std::endl;
              std::cout << "The run had : " << current_step << " steps." << std::endl;
              std::cout << "Params:" << std::endl;
              for(const auto& p : param)
                std::cout << p << " ";

              std::cout << std::endl;

              if(twiddle_direction[twiddle_index] == 0) {
                // Twiddle has tried to tune the parameter in the negative direction
                if(avg_err < best_error) {
                  best_error = avg_err;
                  // Since we found a new optimum, we reset the delta to 0.1
                  delta_param[twiddle_index] = 0.1;
                  last_param[twiddle_index] = param[twiddle_index];
                  new_record = true;
                  new_param = true;
                } else {
                  // Update Params for twiddle
                  param[twiddle_index] = last_param[twiddle_index];
                  param[twiddle_index] += delta_param[twiddle_index];
                  twiddle_direction[twiddle_index] = 1;
                }
              }
              else {
                // Twiddle has tried to tune the parameter in the positive direction
                if(avg_err < best_error) {
                  best_error = avg_err;
                  // Since we found a new optimum, we reset the delta to 0.1
                  delta_param[twiddle_index] = 0.1;
                  last_param[twiddle_index] = param[twiddle_index];
                  new_record = true;
                  new_param = true;
                } else {
                  // Update Params for twiddle
                  param[twiddle_index] = last_param[twiddle_index];
                  param[twiddle_index] -= delta_param[twiddle_index];
                  twiddle_direction[twiddle_index] = 0;
                }
              }

              if(!new_record) {
                delta_param[twiddle_index] *= 0.9;
              }

              if(new_param) {
                twiddle_index += 1;
                twiddle_index = twiddle_index % static_cast<int>(param.size());
              }
            }

            // Reset Steps
            current_step = 0;

            // Reset PID-controller
            pid.Init(param[0], param[1], param[2]);

            // Send Reset command to simulator!
            // Found by looking in :
            // https://github.com/udacity/self-driving-car-sim/blob/a796708484d7411296f9bab51f764327d6cd24da/Assets/1_SelfDrivingCar/Scripts/project_4/CommandServer_pid.cs
            std::string msg = "42[\"reset\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
          else {

            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = 0.5;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            //std::cout << msg << std::endl;
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
