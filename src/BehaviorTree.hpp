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

using namespace std;

struct Map{
    vector<double> &waypoints_x;
    vector<double> &waypoints_y;
    vector<double> &waypoints_s;
    vector<double> &waypoints_dx;
    vector<double> &waypoints_dy;
};

struct CarStatus {
    int lane;

    double car_x;;
    double car_y;
    double car_s;
    double car_d;
    double car_yaw;
    double car_speed;

    vector<double> previous_path_x;
    vector<double> previous_path_y;

    double end_path_s;
    double end_path_d;

    vector<vector<float>> sensor_fusion;
};

class Node {  // This class represents each node in the behaviour tree.
public:
    virtual bool run() = 0;
};

class CompositeNode : public Node {  //  This type of Node follows the Composite Pattern, containing a list of other Nodes.
private:
    std::list<Node*> children;
public:
    const std::list<Node*>& getChildren() const {return children;}
    void addChild (Node* child) {children.emplace_back(child);}
};

class Selector : public CompositeNode {
public:
    virtual bool run() override {
        for (Node* child : getChildren()) {  // The generic Selector implementation
            if (child->run())  // If one child succeeds, the entire operation run() succeeds.  Failure only results if all children fail.
                return true;
        }
        return false;  // All children failed so the entire run() operation fails.
    }
};

class Sequence : public CompositeNode {
public:
    virtual bool run() override {
        for (Node* child : getChildren()) {  // The generic Sequence implementation.
            if (!child->run())  // If one child fails, then enter operation run() fails.  Success only results if all children succeed.
                return false;
        }
        return true;  // All children suceeded, so the entire run() operation succeeds.
    }
};

class IsLaneNumberTask : public Node {
    /*Check we are in the middle of the lane*/

private:
    CarStatus* status;
    int laneNumber;
public:
    IsLaneNumberTask (CarStatus* status, int laneNumber) : laneNumber(laneNumber) {
        this->laneNumber = laneNumber;
    }
    virtual bool run() override {
        return laneNumber== laneNumber*4+2;
    }
};


class IsSomeoneCloseBeforeYouTask : public Node {
    /*Check if someone is close to you in your track*/

private:
    CarStatus* status;
    int laneNumber;

public:
    IsSomeoneCloseBeforeYouTask (CarStatus* status, int laneNumber) : status(status), laneNumber(laneNumber) {
    }
    virtual bool run() override {
        auto &sensor_fusion = status->sensor_fusion;
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

                double &car_s = status->car_s;

                int previos_size = status->previous_path_x.size();
                double future_car_s = s + (previos_size*0.02)*speed;


                if(future_car_s>car_s && future_car_s-car_s<30)
                {
                    return true;
                }
            }
        }
        return false;
    }
};


class DriveTask : public Node {
    /*Check we are in the middle of the lane*/

private:
    CarStatus* status;
    Map *map;
    uWS::WebSocket<uWS::SERVER> &ws;
public:
    DriveTask (CarStatus* status, Map *map, uWS::WebSocket<uWS::SERVER> &ws) : status(status), map(map), ws(ws){

    }
    virtual bool run() override {
        vector<double> ptsX;
        vector<double> ptsY;

        auto &car_x = status->car_x;
        auto &car_y = status->car_y;
        auto &car_yaw = status->car_yaw;

        double &ref_x = status->car_x;
        double &ref_y = status->car_y;
        double ref_yaw = 0;//helpers::deg2rad(status->car_yaw);

        auto &car_s = status->car_s;

        auto &previous_path_x = status->previous_path_x;
        auto &previous_path_y = status->previous_path_y;

        int &lane = status->lane;

        int prev_size = previous_path_x.size();
        // for initial stability
        double ref_vel = 49.5;

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

        vector<double> next_wp0 = helpers::getXY(car_s+30.0, (4*lane)+2, map->waypoints_s, map->waypoints_x, map->waypoints_y);
        vector<double> next_wp1 = helpers::getXY(car_s+60.0, (4*lane)+2, map->waypoints_s, map->waypoints_x, map->waypoints_y);
        vector<double> next_wp2 = helpers::getXY(car_s+90.0, (4*lane)+2, map->waypoints_s, map->waypoints_x, map->waypoints_y);

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
        nlohmann::json msgJson;

        msgJson["next_x"] = next_x_vals;
        msgJson["next_y"] = next_y_vals;

        auto msg = "42[\"control\","+ msgJson.dump()+"]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

    }
};


class IsSomeoneCloseBelowYouTask : public Node {
    /*Check if someone is close to you in your track*/

private:
    CarStatus* status;
    int laneNumber;
public:
    IsSomeoneCloseBelowYouTask (CarStatus* status, int laneNumber) : status(status), laneNumber(laneNumber) {
    }
    virtual bool run() override {
        auto &sensor_fusion = status->sensor_fusion;
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

                double &car_s = status->car_s;

                int previos_size = status->previous_path_x.size();
                double future_car_s = s + (previos_size*0.02)*speed;
                
                double r = 10;

                if(future_car_s>car_s && future_car_s-car_s<30+r)
                {
                    return true;
                }
            }
        }
        return false;
    }
};


//int main() {
////    Sequence *root = new Sequence, *sequence1 = new Sequence;  // Note that root can be either a Sequence or a Selector, since it has only one child.
////    Selector* selector1 = new Selector;  // In general there will be several nodes that are Sequence or Selector, so they should be suffixed by an integer to distinguish between them.
////    DoorStatus* doorStatus = new DoorStatus {false, 5};  // The door is initially closed and 5 meters away.
////    CheckIfDoorIsOpenTask* checkOpen = new CheckIfDoorIsOpenTask (doorStatus);
////    ApproachDoorTask* approach = new ApproachDoorTask (doorStatus, false);
////    OpenDoorTask* open = new OpenDoorTask (doorStatus);
////
////    root->addChild (selector1);
////
////    selector1->addChild (checkOpen);
////    selector1->addChild (sequence1);
////
////    sequence1->addChild (approach);
////    sequence1->addChild (open);
////
////    while (!root->run())  // If the operation starting from the root fails, keep trying until it succeeds.
////        std::cout << "--------------------" << std::endl;
////    std::cout << std::endl << "Operation complete.  Behaviour tree exited." << std::endl;
////    std::cin.get();
//}
