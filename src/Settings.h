//
//  Header.h
//  Path_Planning
//
//  Created by Kiril Cvetkov on 3/4/19.
//

#ifndef Header_h
#define Header_h


namespace Settings {
    static const double SENSOR_FUSION_RADIUS = 100.0;
    static const size_t LANE_D_SIZE = 4;
    static const size_t CLOSE_BELOW_THRESHOLD = 5;

    static const double CONFIDENT_DISTANCE = 30.0;
    
    static const size_t NUMBER_WAY_POINTS = 50;

    static const double TIME_STEP = 0.224;
    
    static const double MAX_SPEED = 49.5;


    
    static const bool DEBUG = false;
}

namespace HyperParams {
    
    static const double SPEED_SCALE = 20;
    static const double DISTANCE_SCALE = 40;
    static const double COUNT_SCALE = 1.9;
}
#endif /* Header_h */
