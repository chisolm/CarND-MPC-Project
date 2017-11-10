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
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double steering_angle = j[1]["steering_angle"];
          double throttle = j[1]["throttle"];

          Eigen::VectorXd eptsx(6);
          Eigen::VectorXd eptsy(6);

          // Transform to vehicle coordinate space
          for (int i = 0; i < ptsx.size(); i++) {
            eptsx[i] = ((ptsx[i] - px) * cos(-psi) - (ptsy[i] - py) *
                sin(-psi));
            eptsy[i] = ((ptsx[i] - px) * sin(-psi) + (ptsy[i] - py) *
                cos(-psi));
          }
          auto coeffs = polyfit(eptsx, eptsy, 3);
          std::cout << "eptsx ";
          for (int i = 0; i < eptsx.size(); i++) {
            std::cout << eptsx[i] << ", ";
          }
          std::cout << std::endl;
          std::cout << "eptsy ";
          for (int i = 0; i < eptsy.size(); i++) {
            std::cout << eptsy[i] << ", ";
          }
          std::cout << std::endl;
          std::cout << "coeffs ";
          for (int i = 0; i < coeffs.size(); i++) {
            std::cout << coeffs[i] << ", ";
          }
          std::cout << std::endl;

          // calculate the cross track error
          double cte = polyeval(coeffs, 0);
          // calculate the orientation error
          double epsi = -atan(coeffs[1]);

          // px, py and psi are now 0 after transform above
          // state << px, py, psi, v, cte, epsi;
          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;

          double delay = .1;  // project set for 100ms
          double Lf = 2.67;  // Would be nice to pull from model

          // Expected position 100ms into the future
          const double px_fut = 0 + (v * cos(0) * delay);
          const double py_fut = 0 + (v * sin(0) * delay);
          const double psi_fut = 0 - (v * steering_angle * delay / Lf);
          // const double v_fut is unused
          const double cte_fut = cte + (v * sin(epsi) * delay);
          // const double epsi_fut = epsi - (v * atan(coeffs[1]) * delay / Lf);
          const double epsi_fut = epsi - (v * steering_angle * delay / Lf);

          // New state delay seconds into the furture.  Note v not estimated.
          state << px_fut, py_fut, psi_fut, v, cte_fut, epsi_fut;
          auto vars = mpc.Solve(state, coeffs);

          // These are the values returned by my MPC model
          state << vars[0], vars[1], vars[2], vars[3], vars[4], vars[5];
          std::cout << "x = " << vars[0] << std::endl;
          std::cout << "y = " << vars[1] << std::endl;
          std::cout << "psi = " << vars[2] << std::endl;
          std::cout << "v = " << vars[3] << std::endl;
          std::cout << "cte = " << vars[4] << std::endl;
          std::cout << "epsi = " << vars[5] << std::endl;
          std::cout << "delta = " << vars[6] << std::endl;
          std::cout << "a = " << vars[7] << std::endl;
          std::cout << std::endl;


          /*
          * Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          double steer_value;
          double throttle_value;

          steer_value = vars[6];
          throttle_value = vars[7];

          json msgJson;
          // Steer angle in model is constrained to -.41 to .41.
          msgJson["steering_angle"] = -steer_value;
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory
          vector<double> mpc_x_vals = mpc.mpc_x;
          vector<double> mpc_y_vals = mpc.mpc_y;

          std::cout << "mpc_x_vals ";
          for (int i = 0; i < mpc_x_vals.size(); i++) {
            std::cout << mpc_x_vals[i] << ", ";
          }
          std::cout << std::endl;
          std::cout << "mpc_y_vals ";
          for (int i = 0; i < mpc_y_vals.size(); i++) {
            std::cout << mpc_y_vals[i] << ", ";
          }
          std::cout << std::endl;

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          // Generate the points from the coeffs
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          double poly_inc = 2.5;
          int num_points = 25;
          for (int i = 0; i < num_points; i++) {
            next_x_vals.push_back(poly_inc * i);
            next_y_vals.push_back(polyeval(coeffs, poly_inc * i));
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

          this_thread::sleep_for(chrono::milliseconds(100));
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
