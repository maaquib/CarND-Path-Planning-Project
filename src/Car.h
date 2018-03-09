//
// Created by Ansari, Aaqib
//

#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H


class Car {
public:
    double s;
    double d;
    double speed;
    int lane;

    /**
     * Constructor
     *
     * @param s value in Frenet
     * @param d value in Frenet
     * @param speed of the Car
     * @param lane in which Car is driving
     */
    Car(double s, double d, double speed, int lane);

    double distance(Car other);

    void update(double s, double d, double speed, int lane);

};


#endif //PATH_PLANNING_CAR_H
