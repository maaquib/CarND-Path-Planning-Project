//
// Created by Ansari, Aaqib
//

#include "Car.h"

Car::Car(double s, double d, double speed, int lane) {
  this->s = s;
  this->d = d;
  this->speed = speed;
  this->lane = lane;
}

void Car::update(double s, double d, double speed, int lane) {
  this->s = s;
  this->d = d;
  this->speed = speed;
  this->lane = lane;
}

double Car::distance(Car other) {
  return other.s - this->s;
}
