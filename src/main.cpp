#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"


// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x, y, s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;

  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  int lane = 1;    // 0 is left lane, 1 is middle lane (car starts here), and 2 is right lane.

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;


          ///// NOTES_AV /////

          double goal_v = 49.5;    // Goal velocity of 49.5 MPH.

          int previous_path_size = previous_path_x.size();    // Previous path size in number of points (50).

          //if (previous_path_size > 0) {
          //  car_s = end_path_s;
          //}

          bool tailgating = false;    // Driving too closely behind another vehicle.

          // Sparse waypoints to be interpolated by Spline.
          vector<double> spline_waypoints_x;
          vector<double> spline_waypoints_y;

          double curr_x = car_x;
          double curr_y = car_y;
          double angle = deg2rad(car_yaw);

          if (previous_path_size < 2) {
            spline_waypoints_x.push_back(car_x - cos(car_yaw));
            spline_waypoints_x.push_back(car_x);

            spline_waypoints_y.push_back(car_y - sin(car_yaw));
            spline_waypoints_y.push_back(car_y);
          } else {
            spline_waypoints_x.push_back(previous_path_x[previous_path_size - 2]);
            spline_waypoints_x.push_back(previous_path_x[previous_path_size - 1]);

            spline_waypoints_y.push_back(previous_path_y[previous_path_size - 2]);
            spline_waypoints_y.push_back(previous_path_y[previous_path_size - 1]);
          }

          // Location of car in 30, 60, and 90 meters.
          vector<double> spline_wp30 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> spline_wp60 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> spline_wp90 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          spline_waypoints_x.push_back(spline_wp30[0]);
          spline_waypoints_x.push_back(spline_wp60[0]);
          spline_waypoints_x.push_back(spline_wp90[0]);

          spline_waypoints_y.push_back(spline_wp30[1]);
          spline_waypoints_y.push_back(spline_wp60[1]);
          spline_waypoints_y.push_back(spline_wp90[1]);

          for (int i = 0; i < spline_waypoints_x.size(); i++) {
            // Shift car reference angle to 0 degree.
            double shift_x = spline_waypoints_x[i] - curr_x;
            double shift_y = spline_waypoints_y[i] - curr_y;

            spline_waypoints_x[i] = shift_x * cos(0 - angle) - shift_y * sin(0 - angle);
            spline_waypoints_y[i] = shift_x * sin(0 - angle) + shift_y * cos(0 - angle);
          }

          // Create a spline
          tk::spline s;

          // Add x, y points on the spline
          s.set_points(spline_waypoints_x, spline_waypoints_y);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (int i = 0; i < previous_path_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);    // AV: Shouldn't we get x, y from end of previous path?
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate spline points to travel
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);

          double x_addon = 0.0;
          for (int i = 1; i <= 50 - previous_path_size; i++) {

            if(goal_v > car_speed) {
              car_speed += .224;
            } else if(goal_v < car_speed) {
              car_speed -= .224;
            }

            double N = target_dist / (0.02 * car_speed / 2.24);    // MPH to m/s
            double x_point = x_addon + target_x / N;
            double y_point = s(x_point);
            x_addon = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = x_ref * cos(angle) - y_ref * sin(angle);
            y_point = x_ref * sin(angle) + y_ref * cos(angle);

            x_point += x_ref;
            y_point += y_ref;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          ///// END_AV /////

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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
