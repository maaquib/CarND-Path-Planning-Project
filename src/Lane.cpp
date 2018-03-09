//
// Created by Ansari, Aaqib
//

#include <iostream>
#include <cmath>
#include "Lane.h"

Lane::Lane(int lane_no) {
  this->lane_no = lane_no;
}

void Lane::addCar(Car car) {
  cars_in_lane.push_back(car);
}

void Lane::clearLane() {
  cars_in_lane.clear();
}

/**
 * Accumulates the cost to stay in or change a lane for different cost functions
 *
 * @param ego_car ego car for reference
 * @return total cost to change or stay in lane
 */
double Lane::laneMoveCost(Car ego_car) {
  double total_cost = 0;

  total_cost += distanceCost(ego_car);
  total_cost += speedCost(ego_car);
  return total_cost;
}

/**
 * Returns the cost of distance. The closer the closest car is in a given lane
 * the higher the cost will be. If the car in the given lane is less than
 * the safe lane change distance and extremely high cost is returned
 *
 * TODO: An improvement would be to have different safe lane change distances
 * for cars ahead vs cars behind
 *
 * @param ego_car ego car for reference
 * @return distance cost
 */
double Lane::distanceCost(Car ego_car) {
  double min_car_distance = MAX_COST;

  for (int i = 0; i < cars_in_lane.size(); ++i) {
    double car_distance = fabs(ego_car.distance(cars_in_lane[i]));
    if (car_distance < SAFE_LANE_CHANGE_DIST) {
      return MAX_COST;
    } else if (min_car_distance > car_distance) {
      min_car_distance = car_distance;
    }
  }

  // std::cout << lane_no << ": " << min_car_distance << std::endl;
  return SAFE_LANE_CHANGE_DIST - min_car_distance;
}

/**
 * Returns the cost of changing or staying in a lane based on how fast the
 * closest vehicle is moving
 *
 * @param ego_car ego car for reference
 * @return speed cost
 */
double Lane::speedCost(Car ego_car) {
  double car_ahead_speed = MAX_SPEED;
  double car_ahead_distance = MAX_COST;

  // Avoid making lane shifts at very low speeds
  // TODO: This can be made a more sophisticated cost function in itself
  // by finding the distance the closest car and predicting based on its
  // speed if it would be very close to car when the lane change completes
  if (ego_car.speed < 30) {
    return MAX_COST;
  }


  for (int i = 0; i < cars_in_lane.size(); ++i) {
    double car_distance = ego_car.distance(cars_in_lane[i]);
    if (car_distance > 0 && car_ahead_distance < car_distance) {
      car_ahead_distance = car_distance;
      car_ahead_speed = cars_in_lane[i].speed;
    }
  }
  return MAX_SPEED - car_ahead_speed;
}


