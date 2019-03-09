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

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
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


  double ref_v = 0; //unit: m/s. This is the reference/target velocity 
                    //used for generating trajectories. Initially, the car is static
  int lane = 1; //This is the target lane for generating trajectories. 
                // Initially, the car is in the middle lane (left lane is 0)
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ref_v, &lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    
    const double speed_limit = 22; //unit: m/s. Converted from 50mph


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

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * Prediction Module
           *
           * This module predicts the traffic condition in the current lane
           * left lane and right lane when the car reaches the end of the 
           * remaining previous path points, based on the sensor fusion data
           */

          //Whether there is a car ahead in the current lane that will be too close
          bool too_close = false; 
          //Whether the left lane is clear of traffic
          bool left_lane_clear = true;
          //Whether the right lane is clear of traffic
          bool right_lane_clear = true;
          int path_size = previous_path_x.size();
          if (path_size>0) {
            //We are looking at the time point when the car reach to the end of
            //remaining previous path points
            car_s = end_path_s; 
          }
          for (int i=0;i<sensor_fusion.size();++i) {
            double check_d = sensor_fusion[i][6];
            double vx =sensor_fusion[i][3]; 
            double vy =sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            double check_s = sensor_fusion[i][5];

            //Predict the checked vehicle's position when the car reaches to the 
            //end of remaining previous path points
            check_s += 0.02*path_size*check_speed; 
            //Check the traffic ahead in the current lane
            if (check_d>lane*4 && check_d<lane*4+4) {
              if (check_s > car_s && check_s-car_s < 30)
                too_close = true;
            }
            //check the traffic in the left lane
            if (check_d > lane*4-4 & check_d < lane*4) {
              if (check_s-car_s < 30  && car_s - check_s < 30) {
                left_lane_clear = false;
              }
            }
            //check the traffic in the right lane
            if (check_d > lane*4+4 && check_d < lane*4+8) {
              if (check_s-car_s < 30  && car_s - check_s < 30) {
                right_lane_clear = false;
              }
            } 
          }

          /**
           * Behavior Planning Module
           *
           * This module makes high-level driving decisions (target lane and velocity)
           * based on the prediction. Although I did not implement explict cost functions
           * here, the first priority is to avoid collision,  and the second is to get to
           * the lane where the car can reach a good velocity. The acceleration and decceleration
           * is designed to be nearly constant at 5m/s^2 (0.1m/s / 0.02s). As the the acceleration
           * and decceleration is almost constant, the jerk is almost 0
           */
          if (too_close) {
            if (left_lane_clear && lane>0) {
              lane--;
            }
            else if (right_lane_clear && lane<2) {
              lane++;
            }
            else
              ref_v -= 0.1;
          }
          else if (ref_v < speed_limit) {
            ref_v += 0.1;
          }

          /**
           * Trajectory Generation Module
           * This module generates the trajectory based on the high-level decisions
           * on the target lane and velocity. It uses the remaining previous path
           * points as a starting point so that there is no discontinuities. It uses
           * five anchor points: two at the end of the previous planned path and three
           * spaced out ahead. It uses the spline function to generate the new trajectory
           * based on the five anchor points. 
           * Before using the spline function, the map coordinates are first transformed to
           * the coordinates in the car's perspective to avoid the potential numerical problems
           * when the spline goes vertical. After that,  the coordinates are transformed back
           * to the map coordinates
           */

          //Load the previous points not consumed by simulator yet
          for (int i = 0; i < path_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          vector<double> anchor_x;
          vector<double> anchor_y;
          //reference x, y, yaw for the starting point of new trajectory (not
          //including the remaining previous path points)
          double ref_x,  ref_y, ref_yaw;
          //Get the first two anchor points
          if (path_size < 2) { //If there is almost no previous path point
                               //use the current car position and extrapolate
                               //a previous position using the reference yaw
            ref_x = car_x;
            ref_y = car_y;
            //The yaw from the sensor fusion is in unit degree. Converted to rad
            ref_yaw = deg2rad(car_yaw);
            //The extrapolation is rough here as there could be cases where cos(ref_yaw)
            //is 0 and then there will be problem in fitting the spline as the spline
            //function requires that the x value is in increasing order
            //But as this is usually done when the car is starting and in the example it 
            //works fine.
            double prev_x = car_x - cos(ref_yaw);
            double prev_y = car_y - sin(ref_yaw);

            anchor_x.push_back(prev_x);
            anchor_x.push_back(ref_x);
            anchor_y.push_back(prev_y);
            anchor_y.push_back(ref_y);
          } else { //if there are more than two previous path points
                   //use the last two previous path points 
            ref_x = previous_path_x[path_size-1];
            ref_y = previous_path_y[path_size-1];

            double pos_x2 = previous_path_x[path_size-2];
            double pos_y2 = previous_path_y[path_size-2];
            ref_yaw = atan2(ref_y-pos_y2,ref_x-pos_x2);

            anchor_x.push_back(pos_x2);
            anchor_x.push_back(ref_x);
            anchor_y.push_back(pos_y2);
            anchor_y.push_back(ref_y);
          }

          //Three waypoints spaced far out after the previous path points
          vector<double> anchor_30 = getXY(car_s+30, 2+lane*4, map_waypoints_s,
              map_waypoints_x, map_waypoints_y);
          vector<double> anchor_60 = getXY(car_s+60, 2+lane*4, map_waypoints_s,
              map_waypoints_x, map_waypoints_y);
          vector<double> anchor_90 = getXY(car_s+90, 2+lane*4, map_waypoints_s,
              map_waypoints_x, map_waypoints_y);
          anchor_x.push_back(anchor_30[0]);
          anchor_x.push_back(anchor_60[0]);
          anchor_x.push_back(anchor_90[0]);
          anchor_y.push_back(anchor_30[1]);
          anchor_y.push_back(anchor_60[1]);
          anchor_y.push_back(anchor_90[1]);

          //A good reference on cooridinate transformation can be found via
          //http://farside.ph.utexas.edu/teaching/336k/Newtonhtml/node153.html
          //
          //Transform x, y with respect to the car coordinate
          for (int i=0;i<anchor_x.size();++i) {
            //First shift
            double x_tran = anchor_x[i] - ref_x;
            double y_tran = anchor_y[i] - ref_y;
            //Then rotate
            anchor_x[i] = x_tran*cos(ref_yaw) + y_tran*sin(ref_yaw);
            anchor_y[i] = -x_tran*sin(ref_yaw) + y_tran*cos(ref_yaw);

          }

          //Information on the spline function could be found via
          //https://kluge.in-chemnitz.de/opensource/spline/
          tk::spline s;
          s.set_points(anchor_x, anchor_y);
          
          double target_x = 30;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x+target_y*target_y);
          double N = target_dist / (0.02*ref_v); 

          for (int i=0;i<50-path_size;++i) {
            double next_x = (i+1) /N * target_x;
            double next_y = s(next_x);
            //Now need to transform back to the original coordinate;
            //First rotate
            double orig_x = next_x*cos(ref_yaw)-next_y*sin(ref_yaw);
            double orig_y = next_y*cos(ref_yaw)+next_x*sin(ref_yaw);

            //Then shift
            orig_x += ref_x;
            orig_y += ref_y;

            next_x_vals.push_back(orig_x);
            next_y_vals.push_back(orig_y);

          }

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
