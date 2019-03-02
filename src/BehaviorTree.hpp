//
//  BehaviorTree.cpp
//  path_planning
//
//  Created by Kiril Cvetkov on 3/1/19.
//

#include <stdio.h>
#include <iostream>
#include <list>
#include <math.h>
#include <algorithm>
#include <vector>
#include <uWS/uWS.h>

#include "helpers.h"
#include "spline.h"
#include "json.hpp"
#include "math.h"

using namespace std;

struct Map{
    vector<double> waypoints_x;
    vector<double> waypoints_y;
    vector<double> waypoints_s;
    vector<double> waypoints_dx;
    vector<double> waypoints_dy;
};
struct CarStatus {
    
    double car_x;
    double car_y;
    double car_s;
    double car_d;
    double car_yaw;
    static double car_speed;
    static double lane;
    
    nlohmann::json previous_path_x;
    nlohmann::json previous_path_y;
    
    double end_path_s;
    double end_path_d;
    
    nlohmann::json sensor_fusion;
    int getLane()
    {
        return car_d/4;
    }
};
double CarStatus::car_speed = 0;
double CarStatus::lane = 1;

class Node { 
public:
    virtual bool run(Map &map, CarStatus &carStatus, uWS::WebSocket<uWS::SERVER> &ws) = 0;
};

class CompositeNode : public Node {
private:
    std::list<Node*> children;
public:
    const std::list<Node*>& getChildren() const {return children;}
    void addChild (Node* child) {children.emplace_back(child);}
};

class Selector : public CompositeNode {
public:
    virtual bool run(Map &map, CarStatus &carStatus, uWS::WebSocket<uWS::SERVER> &ws) override {
        for (Node* child : getChildren()) {
            if (child->run(map, carStatus, ws))
                return true;
        }
        return false;
    }
};

class Sequence : public CompositeNode {
public:
    virtual bool run(Map &map, CarStatus &carStatus, uWS::WebSocket<uWS::SERVER> &ws) override {
        for (Node* child : getChildren()) {
            if (!child->run(map, carStatus, ws))
                return false;
        }
        return true;
    }
};

class IsLaneNumberTask : public Node {
    /*Check we are in the middle of the lane*/
    
private:
    int laneNumber;
public:
    IsLaneNumberTask (int laneNumber) : laneNumber(laneNumber) {
        this->laneNumber = laneNumber;
    }
    virtual bool run(Map &map, CarStatus &status, uWS::WebSocket<uWS::SERVER> &ws) override {
        bool res = round(status.car_d) == laneNumber*4+2 and laneNumber==CarStatus::lane;
        if (res)
        {
            cout<<"Lane check passed => We are exactly on lane : "<<laneNumber<<endl;
        }
        return res;
    }
};

class IsSomeoneCloseBeforeYouTask : public Node {
    /*Check if someone is close to you in your track*/
    
private:
    int laneNumber;
    
public:
    IsSomeoneCloseBeforeYouTask (int laneNumber): laneNumber(laneNumber){
    }
    virtual bool run(Map &map, CarStatus &status, uWS::WebSocket<uWS::SERVER> &ws) override {
        auto &sensor_fusion = status.sensor_fusion;
        bool found = false;
        
        double minDistance=9999;
        double minSpeed=99999;
        int previos_size = status.previous_path_x.size();
        
        double car_s = status.car_s;
        if(previos_size>0)
        {
            car_s = status.end_path_s;
        }
        
        for(int i=0; i<sensor_fusion.size(); i++)
        {
            // get car lane
            float d = sensor_fusion[i][6];
            
            if(d>laneNumber*4 && d<4*(laneNumber+1))
            {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double speed = sqrt(vx*vx + vy*vy);
                double s = sensor_fusion[i][5];
                
                double future_car_s = s + (previos_size*0.02)*speed;
                
                if(future_car_s>car_s && future_car_s-car_s<30)
                {
                    double d = future_car_s-car_s;
                    if(minDistance>d)
                    {
                        minDistance=d;
                        minSpeed = speed*2.24;
                    }
                    
                    //cout<<"true"<<"IsSomeoneC`
                }
            }
        }
        //CarStatus::car_speed = 49.5
        if(found)
        {
            cout<<"Decreasing car speed"<<endl;
            CarStatus::car_speed = max(CarStatus::car_speed-0.224, min(CarStatus::car_speed, minSpeed));
        }
        else
        {
            cout<<"Increasing car speed"<<endl;
            
            CarStatus::car_speed = min(CarStatus::car_speed+0.224f, 49.5);
        }
        //
        cout<<CarStatus::car_speed<<"SPEED"<<endl;
        cout<<"IsSomeoneCloseBeforeYouTask : Lane : "<<laneNumber<<" found:"<<found<<endl;
        return found;
    }
};



//
//class ChangeVelocityToTheClosestInLane : public Node {
//    /*Check if someone is close to you in your track*/
//
//private:
//    int laneNumber;
//
//public:
//    ChangeVelocityToTheClosestInLane (int laneNumber=-1){
//
//    }
//    virtual bool run(Map &map, CarStatus &status, uWS::WebSocket<uWS::SERVER> &ws) override {
//        auto &sensor_fusion = status.sensor_fusion;
//
//
//        for(int i=0; i<sensor_fusion.size(); i++)
//        {
//            // get car lane
//            float d = sensor_fusion[i][6];
//
//            if(d>laneNumber*4 && d<4*(laneNumber+1))
//            {
//                double vx = sensor_fusion[i][3];
//                double vy = sensor_fusion[i][4];
//                double speed = sqrt(vx*vx + vy*vy);
//                double s = sensor_fusion[i][5];
//
//                double car_s = status.car_s;
//
//                int previos_size = status.previous_path_x.size();
//
//
//                if(previos_size>0)
//                {
//                    car_s = status.end_path_s;
//                }
//
//
//                double future_car_s = s + (previos_size*0.02)*speed;
//
//
//                if(future_car_s>car_s && future_car_s-car_s<30)
//                {
//                    double d = future_car_s-car_s;
//                    if(minDistance>d)
//                    {
//                        minDistance=d;
//                        minSpeed = speed*2.24;
//                    }
//
//                }
//            }
//        }
//        CarStatus::car_speed = minSpeed;
//        cout<<"ChangeVelocityToTheClosestInLane"<<minSpeed<<endl;
//        return true;
//    }
//
//
//};


class DriveTask : public Node {
    /*Check we are in the middle of the lane*/
    
private:
    int switchLane=-1;
public:
    DriveTask (int switchLane=-1): switchLane(switchLane){
    }
    
    virtual bool run(Map &map, CarStatus &status,  uWS::WebSocket<uWS::SERVER> &ws) override {
        vector<double> ptsX;
        vector<double> ptsY;
        
        auto car_x = status.car_x;
        auto car_y = status.car_y;
        auto car_yaw = status.car_yaw;
        
        double ref_x = status.car_x;
        double ref_y = status.car_y;
        double ref_yaw = 0;//helpers::deg2rad(status->car_yaw);
        
        auto previous_path_x = status.previous_path_x;
        auto previous_path_y = status.previous_path_y;
        int prev_size = previous_path_x.size();
        
        auto car_s = status.car_s;
        
        if(prev_size>0)
        {
            car_s = status.end_path_s;
        }
        
        int lane = CarStatus::lane;
        
        if (switchLane!=-1)
        {
            CarStatus::lane = lane = switchLane;
            cout<<"Switching lane to"<<lane<<endl;
            
        }
        else
        {
            cout<<"Going to the same direction"<<endl;
        }
        
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
        
        vector<double> next_wp0 = helpers::getXY(car_s+30.0, (4*lane)+2, map.waypoints_s, map.waypoints_x, map.waypoints_y);
        vector<double> next_wp1 = helpers::getXY(car_s+60.0, (4*lane)+2, map.waypoints_s, map.waypoints_x, map.waypoints_y);
        vector<double> next_wp2 = helpers::getXY(car_s+90.0, (4*lane)+2, map.waypoints_s, map.waypoints_x, map.waypoints_y);
        
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
        
        double n = target_dist / (0.02f/2.24 * CarStatus::car_speed);
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
        nlohmann::json msgJson;
        
        msgJson["next_x"] = next_x_vals;
        msgJson["next_y"] = next_y_vals;
        
        auto msg = "42[\"control\","+ msgJson.dump()+"]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        
        return true;
    }
};


class IsOtherLaneFeasibleTask : public Node {
    /*Check if someone is close to you in your track*/
    
private:
    int laneNumber;
public:
    IsOtherLaneFeasibleTask (int laneNumber) :  laneNumber(laneNumber) {
    }
    virtual bool run(Map &map, CarStatus &status, uWS::WebSocket<uWS::SERVER> &ws) override {
        auto &sensor_fusion = status.sensor_fusion;
        for(int i=0; i<sensor_fusion.size(); i++)
        {
            // get car lane
            float d = sensor_fusion[i][6];
            
            double car_s = status.car_s;
            
            int previos_size = status.previous_path_x.size();
            
            if(previos_size>0)
            {
                car_s = status.end_path_s;
            }
            
            if(d>laneNumber*4 && d<4*(laneNumber+1))
            {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double speed = sqrt(vx*vx + vy*vy);
                double s = sensor_fusion[i][5];
                
                double r = 5;
                double future_car_s = s + (previos_size*0.02)*speed+r;
                
                if(future_car_s>car_s && future_car_s-car_s<30)
                {
                    return false;
                }
            }
        }
        return true;
    }
};





class AproximateSpeedBefore : public Node {
    /*Check if someone is close to you in your track*/
    
private:
    int laneNumber;
    
public:
    AproximateSpeedBefore (int laneNumber): laneNumber(laneNumber){
    }
    virtual bool run(Map &map, CarStatus &status, uWS::WebSocket<uWS::SERVER> &ws) override {
        auto &sensor_fusion = status.sensor_fusion;
        bool found = false;
        
        double minDistance=9999;
        double minSpeed=99999;
        int previos_size = status.previous_path_x.size();
        
        double car_s = status.car_s;
        if(previos_size>0)
        {
            car_s = status.end_path_s;
        }
        
        for(int i=0; i<sensor_fusion.size(); i++)
        {
            // get car lane
            float d = sensor_fusion[i][6];
            
            if(d>laneNumber*4 && d<4*(laneNumber+1))
            {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double speed = sqrt(vx*vx + vy*vy);
                double s = sensor_fusion[i][5];
                
                double future_car_s = s + (previos_size*0.02)*speed;
                
                if(future_car_s>car_s && future_car_s-car_s<30)
                {
                    double d = future_car_s-car_s;
                    if(minDistance>d)
                    {
                        minDistance=d;
                        minSpeed = speed*2.24;
                    }
                    
                    //cout<<"true"<<"IsSomeoneC`
                }
            }
        }
        //CarStatus::car_speed = 49.5
        if(found)
        {
            cout<<"Decreasing car speed"<<endl;
            CarStatus::car_speed = max(CarStatus::car_speed-0.224, min(CarStatus::car_speed, minSpeed));
        }
        else
        {
            cout<<"Increasing car speed"<<endl;
            
            CarStatus::car_speed = min(CarStatus::car_speed+0.224f, 49.5);
        }
        
        cout<<CarStatus::car_speed<<"SPEED"<<endl;
        cout<<"AproximateSpeed : Lane : "<<laneNumber<<" found:"<<found<<endl;
        return found;
    }
};


class ChangeSpeed : public Node {
    /*Check if someone is close to you in your track*/
    
private:
    double speed;
    
public:
    ChangeSpeed (double speed): speed(speed){
    }
    virtual bool run(Map &map, CarStatus &status, uWS::WebSocket<uWS::SERVER> &ws) override {
        
        if(speed>CarStatus::car_speed)
            CarStatus::car_speed = max(CarStatus::car_speed-0.224, speed);
        
        else
        {
            cout<<"Increasing car speed"<<endl;
            
            CarStatus::car_speed = min(CarStatus::car_speed+0.224f, speed);
        }
    }
    
};


class ColisionDetection : public Node {
    /*Check if someone is close to you in your track*/
    
private:
    int laneNumber;
    
public:
    ColisionDetection (int laneNumber): laneNumber(laneNumber){
    }
    virtual bool run(Map &map, CarStatus &status, uWS::WebSocket<uWS::SERVER> &ws) override {
        auto &sensor_fusion = status.sensor_fusion;
        bool found = false;
        
        int previos_size = status.previous_path_x.size();
        
        double car_s = status.car_s;
        double car_d = status.car_d;
        if(previos_size>0)
        {
            car_s = status.end_path_s;
        }
        
        for(int i=0; i<sensor_fusion.size(); i++)
        {
            // get car lane
            float d = sensor_fusion[i][6];
            
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double speed = sqrt(vx*vx + vy*vy);
            double s = sensor_fusion[i][5];
            
            double future_car_s = s + (previos_size*0.02)*speed;
            double future_car_d = d + (4*0.02)*speed;
            
            if(future_car_s>car_s && future_car_s-car_s<8)
            {
                
                return true;
            }
            
            if(abs(car_s - s)<5 and abs(future_car_d-car_d)<1)
            {
                return true;
            }
        }
        
        return false;
    }
};
