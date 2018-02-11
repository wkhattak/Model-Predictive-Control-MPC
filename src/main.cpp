#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <fstream>
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
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
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
  ofstream out_file;
  out_file.open("./results.csv");
  out_file << "error_cte\terror_orientation\tsteering_radians\tthrottle" << endl;
  
  h.onMessage([&mpc,&out_file](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"]; // 6 waypoints x
          vector<double> ptsy = j[1]["ptsy"]; // 6 waypoints y
          double px = j[1]["x"]; // car position x
          double py = j[1]["y"]; // car position y
          double psi = j[1]["psi"]; // car orientation
          double v = j[1]["speed"]; // car velocity
          // Below 2 are used for estimating future state based on latency of 100 ms
          double steer_value = j[1]["steering_angle"]; // steering value [-1,+1]
          double throttle_value = j[1]["throttle"]; // throttle value [-1,+1]
         
          /*************************************************************
          * Step 1: Transform waypoints (px,py) from global co-ordinate system to vehicle co-ordinate system 
          *************************************************************/
          // Make x,y co-ordinates 0
          for (unsigned int i = 0; i < ptsx.size(); i++) {
            double transformed_x = ptsx[i] - px;
            double transformed_y = ptsy[i] - py;
            
            // Rotate points
            ptsx[i] = transformed_x*cos(-psi) - transformed_y*sin(-psi);
            ptsy[i] = transformed_x*sin(-psi) + transformed_y*cos(-psi);
          }
          
          double* ptr_x = &ptsx[0]; // pointer at first x position
          double* ptr_y = &ptsy[0]; // pointer at first y position
          Eigen::Map<Eigen::VectorXd> waypoints_x(ptr_x, 6);
          Eigen::Map<Eigen::VectorXd> waypoints_y(ptr_y, 6);
          
          /*************************************************************
          * Step 2: Fit a polynomial to the transformed waypoints (order 3 as we have 6 points)
          *************************************************************/
          auto coeffs = polyfit(waypoints_x, waypoints_y, 3);
          
          /*************************************************************
          * Step 3: Calculate initial CTE & Orientation error
          *************************************************************/
          // Using 0 because co-ordinates have been shifted so car's x,y & psi are all 0.
          // Returned value is the y, which is basically our CTE
          double cte = polyeval(coeffs, 0); 
          
          // Actual equation --> psi - atan(coeffs[1])
          // but as psi is 0, so only left with -atan(coeffs[1])
          double epsi = -atan(coeffs[1]); 
          
          /*************************************************************
          * Step 4: Choose state after 100ms for solving
          *************************************************************/
          // Get the state after 100 ms & use the calculated state as current state
          
          // This is the length from front to CoG that has a similar radius.
          const double Lf = 2.67; 
          // 0.1 = 100 ms latency
          const double x_future = v * 0.1;
          const double y_future = 0.0;
          const double psi_future = - v/Lf * steer_value * 0.1;         
          const double v_future = v + throttle_value * 0.1;
          const double cte_future = cte + v * sin(epsi) * 0.1;
          const double epsi_future = epsi - v/Lf * steer_value * 0.1;
          
          Eigen::VectorXd state(6);
          state << x_future, y_future, psi_future, v_future, cte_future, epsi_future;
          
          /*************************************************************
          * Step 5: Call the MPC solver
          *************************************************************/
          auto vars = mpc.Solve(state, coeffs);
          
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          
          // Multiplying by Lf provides the effective turning angle
          steer_value = vars[0]/(deg2rad(25)*Lf);
          throttle_value = vars[1];

          json msgJson;
          
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          
          for (unsigned int i = 4; i < vars.size(); i ++) {
            if (i%2 == 0) mpc_x_vals.push_back(vars[i]); // every even value is x
            else mpc_y_vals.push_back(vars[i]); // every odd value is y
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          int num_points = 50; 
          for (int i = 0; i < num_points; i++) { 
            next_x_vals.push_back(2*i);// 2 units (meters) distance in x
            next_y_vals.push_back(polyeval(coeffs, 2*i)); // corresponding y for above x
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
      
          out_file << vars[2] << "\t" << vars[3] << "\t" << steer_value << "\t" << throttle_value << endl;
          
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";

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
      } 
      else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } 
    else {
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
  } 
  else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}