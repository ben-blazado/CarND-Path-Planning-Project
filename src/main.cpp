#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

// blazado: project inlcude files
#include <thread>
#include "localization.h"
#include "ticker.h"

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
  
  // blazado: module initizalization
  Ticker::Instance().secs_per_tick(0.2);
  // Behavior behavior;
  Localization localization(map_waypoints_s, map_waypoints_x, map_waypoints_y,
    map_waypoints_dx, map_waypoints_dy, Ticker::Instance().secs_per_tick());
  std::cout<<"run"<<std::endl;    

    
  //Behavior behavior(localization);
  //Trajectory trajectory(localization, behavior);
  
  // localization.Run();
  // localization.Test();
  //behavior.Run();
  //trajectory.Run();
  
  //https://stackoverflow.com/questions/5395309/how-do-i-force-cmake-to-include-pthread-option-during-compilation
  
  double vel=0;
  double old_x;
  double old_y;
  bool first = true;
  int loop_num = 0;
  
  double end_s;
  double end_d;
  

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&localization,&vel,&old_x, &old_y, &first, &loop_num, &end_s, &end_d]
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

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
           
          //localization.Update(car_x, car_y, car_s, car_d, 
          //    car_yaw, car_speed);
          
          //trajectory.GetXYVals(next_x_vals, next_y_vals);
          
          double curr_s;
          double curr_d;
          int path_remaining =  previous_path_x.size();
          double acc = 4.4704;  // m/s2
          // std::cout << "*** car vel mps  " << vel << std::endl;
          //std::cout << "*** car speed mph " << car_speed << std::endl;
          //vel = (car_speed / 2.237);
          
          if (path_remaining > 0) {
            //curr_s = end_path_s;
            //curr_d = end_path_d;
            double end_x = previous_path_x.back();
            double end_y = previous_path_y.back();
            
            FrenetP end_f = localization.CalcSD ({end_x, end_y});
            
            curr_s = end_s;
            curr_d = end_d;

            std::cout << "Start XY " << end_x << " " << end_y << std::endl;
            std::cout << "Start SD " << curr_s << " " << curr_d << std::endl;
            
            for (int i = 0; i < previous_path_x.size(); i ++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }
          }
          else {
            FrenetP f = localization.CalcSD({car_x, car_y});          
            curr_s = f.s;
            curr_d = f.d;
            vector<double> sd;
            sd = getFrenet(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);
            std::cout << "********** No path remaining " << curr_s << " " << curr_d << std::endl;      
            std::cout << "********** No path remaining " << car_s << " " << car_d << std::endl;      
            std::cout << "********** No path remaining " << sd[0] << " " << sd[1] << std::endl;    

            std::cout << "********** " << std::endl;    
            std::cout << "********** No path remaining " << car_x << " " << car_y << std::endl;                
            CartP p = localization.CalcXY({curr_s, curr_d});
            std::cout << "********** No path remaining " << p.x << " " << p.y << std::endl;    
            p = localization.CalcXY({car_s, car_d});
            std::cout << "********** No path remaining " << p.x << " " << p.y << std::endl;    
            vector<double>xy = getXY(sd[0], sd[1], map_waypoints_s, map_waypoints_x, map_waypoints_y);
            std::cout << "********** No path remaining " << xy[0] << " " << xy[1] << std::endl;    
            //exit(1);
            
          }
          std::cout << " before FOR  curr_s " << curr_s << " " << curr_d << std::endl;
          //curr_d = 6.0;
          for (int i = 0; i < 50 - path_remaining; i ++) {
            std::cout << "vel " << vel << std::endl;
            if (vel < 21) {     // (49 / 2.237))
              std::cout << "increasing vel " << vel << std::endl;
              vel += acc * 0.02;
            }
            curr_s += vel * 0.02;
            //curr_d = 6.0;
            std::cout << "   curr_s " << curr_s << " " << curr_d << std::endl;
            // curr_s = fmod (curr_s, Localization::kMaxSVal_);
            CartP p = localization.CalcXY({curr_s, curr_d});
            /*
            if (next_x_vals.size() > 0) {
              old_x = next_x_vals.back();
              old_y = next_y_vals.back();
              double dist = distance(p.x, p.y, old_x, old_y);
              std::cout << "dist to prev " << dist << std::endl;
              std::cout << "adding point " << p.x << " " << p.y << std::endl;
              if (dist > 50)
                std::cout << "******  added far away point ******" << p.x << " " << p.y << std::endl << std::flush;;
                // exit(0);
            }
            */
            next_x_vals.push_back(p.x);
            next_y_vals.push_back(p.y);
          } 
          /*
          double curr_x = car_x;
          double curr_y = car_y;
          std::cout << "adding points" << std::endl;
          for (int i=0; i < 50; i ++) {
            curr_x += 5*(cos(deg2rad(car_yaw))) * 0.02;
            curr_y += 5*(sin(deg2rad(car_yaw))) * 0.02;
            
            next_x_vals.push_back(curr_x);
            next_y_vals.push_back(curr_y);
          }
          */
          
          end_s = curr_s;
          end_d = curr_d;
          
          {
            double end_x = next_x_vals.back();
            double end_y = next_y_vals.back();
        
            FrenetP end_f = localization.CalcSD ({end_x, end_y});
          
            std::cout << "End XY " << end_x << " " << end_y << std::endl;
            std::cout << "End SD " << end_f.s << " " << end_f.d << std::endl;
          }
          
          loop_num ++;
          //if (loop_num == 3)
          //  exit(0);
          std::cout << "******  END  ******" << std::endl;
          
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
  
  std::cout << "about to run" << std::endl;
  
  h.run();
  
  std::cout << "ended" << std::endl;
}