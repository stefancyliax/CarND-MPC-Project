#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos)
  {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x)
{
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++)
  {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order)
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++)
  {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++)
  {
    for (int i = 0; i < order; i++)
    {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main()
{
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2')
    {
      string s = hasData(sdata);
      if (s != "")
      {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry")
        {
          /*****************************
          Read in data from simulator
          *****************************/
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"]; // global x position of the waypoints, the sim always provides the next 6 waypoints
          vector<double> ptsy = j[1]["ptsy"]; // global y position of the waypoints
          double px = j[1]["x"];              // global x position of the vehicle
          double py = j[1]["y"];              // global y position of the vehicle
          double psi = j[1]["psi"];           // orientation of the vehicle in radians converted from the Unity format to the standard format expected in most mathematical functions
          double v = j[1]["speed"];           // speed in mpg
          
          // Debug output
          // cout << "reported state px, py: " << px << ", " << py << endl;
          // cout << "psi, v: " << psi << ", " << v << endl;

          /******************************
           * update state with delay 
           * First predict postion of vehicle after delay and then use this state
           * This is done in map coordinates
          ******************************/
          const double dt = 0.1;
          const double Lf = 2.67;

          // use current throttle_value to estimate acceleration
          double throttle_value = j[1]["throttle"];
          double a = throttle_value * 7.5;
          double v_ms = v * 0.44704; // correction factor mph to m/s

          // use current steering angle
          double steer_value = j[1]["steering_angle"];
          double delta = -steer_value;

          // predict map coordinates of vehicle after dt=0.1s
          px = px + v_ms * cos(psi) * dt;
          py = py + v_ms * sin(psi) * dt;
          psi = psi + v_ms / Lf * delta * dt;
          v = v + a * dt;

          // Debug output
          // cout << "predicted state state px, py: " << px << ", " << py << endl;
          // cout << "psi, v: " << psi << ", " << v << endl << endl;

          /*****************************
          fit polynominal to waypoints
          *****************************/
          // loop over waypoints and transform from map space to vehicle space
          for (int i = 0; i < ptsx.size(); i++)
          {
            double xd = ptsx[i] - px;
            double yd = ptsy[i] - py;
            ptsx[i] = (xd * cos(-psi) - yd * sin(-psi));
            ptsy[i] = (xd * sin(-psi) + yd * cos(-psi));
          }

          // pointer magic as in project video to get polyfit to work
          double *ptrx = &ptsx[0];
          Eigen::Map<Eigen::VectorXd> ptsx_vehicle(ptrx, 6);
          double *ptry = &ptsy[0];
          Eigen::Map<Eigen::VectorXd> ptsy_vehicle(ptry, 6);

          // polyfit onto waypoints in vehicle coordinate system
          auto coeffs = polyfit(ptsx_vehicle, ptsy_vehicle, 3);

          /******************************
           * generate state of vehicle 
          ******************************/
          // calculate cte as horizontal distance to polynominal at x=0.
          double cte = polyeval(coeffs, 0);
          // calculate epsi
          double epsi = -atan(coeffs[1]);

          // set current (or predicted state) as working state for optimizer
          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;

          /******************************
          call model predictive control to calculate steering angle and throttle
          ******************************/
          auto vars = mpc.Solve(state, coeffs);
          // take steering and throttle from mpc
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          steer_value = -vars[0] / deg2rad(25);
          throttle_value = vars[1];

          /*******************************
          Display the waypoints/reference line
          *******************************/
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // display fitted polynominal as reference line
          // display 25*2.5 m = 62.5 m in front of the vehicle
          float poly_inc = 2.5;
          for (int i = 1; i < 25; i++)
          {
            next_x_vals.push_back(i * poly_inc);
            next_y_vals.push_back(polyeval(coeffs, i * poly_inc));
          }

          // // alternative: display waypoints
          // next_x_vals = ptsx;
          // next_y_vals = ptsy;

          //Display the MPC predicted trajectory
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for (int i = 2; i < vars.size(); i++)
          {
            if (i % 2 == 0)
            {
              mpc_x_vals.push_back(vars[i]);
            }
            else
            {
              mpc_y_vals.push_back(vars[i]);
            }
          }

          //// alternative: display fitted polynominal
          // float poly_inc = 2.5;
          // for (double i = 0; i < 25; i++){
          //   mpc_x_vals.push_back(i * poly_inc);
          //   mpc_y_vals.push_back(polyeval(coeffs, i * poly_inc));
          // }


          /******************************
           * create message for sending back to simulator
          ******************************/
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          // Use delay to calculate state in the future and use this as initial state
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
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

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
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
