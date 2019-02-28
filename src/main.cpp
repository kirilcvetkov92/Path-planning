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
using namespace std;
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
    int lane = 1;
    double ref_vel = 0;
    
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel]
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


          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
            int prev_size = previous_path_x.size();


            bool too_close = false;
            bool too_close1 = false;
            bool too_close2 = false;
            
            double min_speed = 999;
            double min_pos = 999;
            double min_speed1 = 999;
            double min_pos1 = 999;
            double min_speed2 = 999;
            double min_pos2 = 9;
            if(prev_size>0)
            {
                car_s = end_path_s;
            }
            
            for(int i=0; i<sensor_fusion.size(); i++)
            {
                // get car lane
                float d = sensor_fusion[i][6];

                if(d>0*4 && d<4*(0+1))
                {
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double speed = sqrt(vx*vx + vy*vy);
                    double s = sensor_fusion[i][5];
                    
                    double r = lane==0?0:10;

                    
                    double future_car_s = s + (prev_size*0.02)*speed+r;
                    
                    
                    if((future_car_s>car_s) && ((future_car_s-car_s)<30))
                    {
                        if(future_car_s-car_s<min_pos)
                        {
                            min_pos = future_car_s-car_s;
                            min_speed = speed;
                        }
                        too_close = true;
                    }
                }
                
                else if(d>(1)*4 && d<4*(1+1))
                {
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double speed = sqrt(vx*vx + vy*vy);
                    double s = sensor_fusion[i][5];
                    
                    double r = lane==1?0:10;

                    double future_car_s = s + (prev_size*0.02)*speed+r;
                    

                    if((future_car_s>car_s) && ((future_car_s-car_s)<30))
                    {
                        if(future_car_s-car_s<min_pos)
                        {
                            min_pos1 = future_car_s-car_s;
                            min_speed1 = speed;
                        }
                        too_close1 = true;
                    }
                }
                else if(d>(2)*4 && d<4*(2+1))
                {
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double speed = sqrt(vx*vx + vy*vy);
                    double s = sensor_fusion[i][5];
                    
                    double r = lane==2?0:10;

                    double future_car_s = s + (prev_size*0.02)*speed+r;
                    

                    
                    if((future_car_s>car_s) && ((future_car_s-car_s)<30))
                    {
                        if(future_car_s-car_s<min_pos)
                        {
                            min_pos2 = future_car_s-car_s;
                            min_speed2 = speed;
                        }
                        too_close2 = true;
                    }
                }
            }
            
            cout<<too_close<<" "<<too_close1<<" "<<too_close2<<endl;
            if(lane==0 and too_close and !too_close1)
            {
                lane = 1;
            }
            else if (lane==1 and too_close1 and !too_close)
            {
                lane = 0;
            }
            else if(lane==1 and too_close1 and !too_close2)
            {
                lane = 2;
            }
            else if (lane==2 and too_close2 and !too_close1)
            {
                lane=1;
            }
            else if ((too_close1 and lane==1) | (too_close2 and lane==2) | (too_close and lane==0))
            {
                double speed_ = 0;
                if(too_close1 and lane==1)
                    speed_ = min_speed1;
                else if (too_close2 and lane==2)
                {
                    speed_ = min_speed2;
                }
                else
                speed_ = min_speed;
                ref_vel =max(speed_*2.24, ref_vel-0.224);
            }
            else
            if(ref_vel<49.5)
            {
                ref_vel+=0.224;
            }
       
            
            vector<double> ptsX;
            vector<double> ptsY;
            
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);
            
            
            // for initial stability
            if (prev_size<2)
            {
                double prev_car_x = car_x - 1*cos(car_yaw);
                double prev_car_y = car_y - 1*sin(car_yaw);
                ptsX.push_back(prev_car_x);
                ptsX.push_back(ref_x);
                ptsY.push_back(prev_car_y);
                ptsY.push_back(ref_y);

            }
            else
            {
                ref_x = previous_path_x[prev_size-1];
                ref_y = previous_path_y[prev_size-1];
                
                double ref_x2 = previous_path_x[prev_size-2];
                double ref_y2 = previous_path_y[prev_size-2];
                ref_yaw = atan2(ref_y-ref_y2,ref_x-ref_x2);
                
                ptsX.push_back(ref_x2);
                ptsX.push_back(ref_x);
                ptsY.push_back(ref_y2);
                ptsY.push_back(ref_y);
            }
            
          vector<double> next_wp0 = getXY(car_s+30, (4*lane)+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, (4*lane)+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, (4*lane)+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
         ptsX.push_back(next_wp0[0]);
         ptsX.push_back(next_wp1[0]);
         ptsX.push_back(next_wp2[0]);

        ptsY.push_back(next_wp0[1]);
        ptsY.push_back(next_wp1[1]);
        ptsY.push_back(next_wp2[1]);
            
        for(int i=0; i<ptsX.size(); i++)
        {
            double shift_x = ptsX[i]-ref_x;
            double shift_y = ptsY[i]-ref_y;
            
            ptsX[i] = (shift_x * cos(-ref_yaw) - shift_y*sin(-ref_yaw));
            ptsY[i] = (shift_x * sin(-ref_yaw) + shift_y*cos(-ref_yaw));
        }
        
            tk::spline s;
            
            s.set_points(ptsX, ptsY);
            
            vector<double> next_x_vals;
            vector<double> next_y_vals;
            
            for(int i=0; i<previous_path_x.size(); i++)
            {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back((previous_path_y[i]));
            }
            
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt((target_x*target_x)+(target_y*target_y));
            
            double n = target_dist / (0.02f/2.24 * ref_vel);
            double add_x=0;
            for(int i=0; i< 50 - previous_path_x.size(); i++)
            {
                double x_point = add_x + (target_x/n);
                double y_point = s(x_point);
                
                add_x = x_point;
                
                double x_ref = x_point;
                double y_ref = y_point;
                
                x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
                y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));
                
                x_point+=ref_x;
                y_point+=ref_y;
                
                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
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
