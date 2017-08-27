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

#define LATENCY 100 // Latency in milliseconds.

// for convenience
using json = nlohmann::json;

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];     // Waypoints (x) in Map Coordinates.
          vector<double> ptsy = j[1]["ptsy"];     // Waypoints (y) in Map Coordinates.
          double px = j[1]["x"];                  // Vehicle x position in Map Coordinates.
          double py = j[1]["y"];                  // Vehicle y position in Map Coordinates.
          double psi = j[1]["psi"];               // Vehicle driving angle in Map Coordinates.
          double v = j[1]["speed"];               // Velocity (mph).
          double delta = j[1]["steering_angle"];  // Steering angle in radians.

          double latency_s = (double)LATENCY/1000.0; // Latency in seconds.

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          double steer_value;
          double throttle_value;

          Eigen::VectorXd state(6);
          Eigen::VectorXd coeffs(4);

          // Fit a polynomial to the x and y points.
          unsigned int ptscount = ptsx.size();
          Eigen::VectorXd veh_ptsx(ptscount);
          Eigen::VectorXd veh_ptsy(ptscount);
          double cos_psi = cos(psi);
          double sin_psi = sin(psi);
          for(unsigned int i = 0; i < ptscount; ++i) {
            double dx = ptsx[i] - px;
            double dy = ptsy[i] - py;
            veh_ptsx[i] = dx * cos_psi + dy * sin_psi;
            veh_ptsy[i] = dy * cos_psi - dx * sin_psi;
          }
          coeffs = polyfit(veh_ptsx, veh_ptsy, 3);

          // State non-zero as it includes latency values.
          v *= MPC::mph_to_mps; // mph -> m/s
          psi = -v*delta/MPC::Lf*latency_s;
          px = v*cos(psi)*latency_s;
          py = v*sin(psi)*latency_s;

          double cte = polyeval(coeffs, px);
          double epsi = psi - atan(coeffs[1] + 2*coeffs[2]*px + 2*coeffs[3]*pow(px,2));

          state << px, py, psi, v, cte, epsi;

          auto solution = mpc.Solve(state, coeffs, LATENCY);

          steer_value = solution[0];
          throttle_value = solution[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -steer_value/deg2rad(25);
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory in the vehicle's coordinate system
          // The points are displayed in the simulator connected by a Green line.
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          unsigned int num_mpc_points = solution.size()/2;
          for (unsigned int p = 1; p < num_mpc_points; ++p) {
            mpc_x_vals.push_back(solution[p * 2]);
            mpc_y_vals.push_back(solution[p * 2 + 1]);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the polynomial fitted to the waypoints/reference line in the vehicle's coordinate system
          // The points are displayed in the simulator connected by a Yellow line.
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          const unsigned int point_offset = 5;
          for (unsigned int p = 1; p < 20; ++p) {
            next_x_vals.push_back(point_offset * p);
            next_y_vals.push_back(polyeval(coeffs, point_offset * p));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(LATENCY));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
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
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
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
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
