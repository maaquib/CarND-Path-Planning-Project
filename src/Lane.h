//
// Created by Ansari, Aaqib
//

#ifndef PATH_PLANNING_LANE_H
#define PATH_PLANNING_LANE_H

#include <vector>
#include "Car.h"

class Lane {
public:
    int lane_no;
    std::vector<Car> cars_in_lane;

    Lane(int lane_no);

    void addCar(Car car);

    void clearLane();

    double laneMoveCost(Car ego_car);

private:
    double SAFE_LANE_CHANGE_DIST = 15;
    double MAX_COST = 99999999;
    double MAX_SPEED = 100;
    double distanceCost(Car ego_car);
    double speedCost(Car ego_car);
};


#endif //PATH_PLANNING_LANE_H
