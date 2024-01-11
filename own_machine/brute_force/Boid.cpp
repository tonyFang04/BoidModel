#include "header.hpp"
void Boid::set_itsPosition(double x, double y) {
  itsPosition[0] = x;
  itsPosition[1] = y;
}

void Boid::set_itsVelocity(double vx, double vy) {
  itsVelocity[0] = vx;
  itsVelocity[1] = vy;
}

double Boid::get_itsPosition(int i) const {
  return itsPosition[i];
}

double Boid::get_itsVelocity(int i) const {
  return itsVelocity[i];
}

double Boid::getDistance(const Boid & other) const {
  double otherPosition[2];
  for (int i = 0; i < 2; i++) {
    otherPosition[i] = other.get_itsPosition(i);
  }
  return distance(itsPosition,otherPosition);
}

void Boid::setBoundary(double sizeOfBoundary) {
  for (int i = 0; i < 2; i++) {
    if ((itsPosition[i] + itsVelocity[i]) > sizeOfBoundary / 2.0) {
      itsPosition[i] = itsPosition[i] - sizeOfBoundary;
    } if ((itsPosition[i] + itsVelocity[i])  < - sizeOfBoundary / 2.0) {
      itsPosition[i] = itsPosition[i] + sizeOfBoundary;
    }
  }
}
void Boid::setMaxSpeed(double maxSpeed) {
  double normSpeed = norm(itsVelocity);
  for (int i = 0; i < 2; i++) {
    if (normSpeed > maxSpeed) {
      itsVelocity[i] = (1 / normSpeed) * itsVelocity[i] * maxSpeed;
    }
  }
}

void Boid::accelerate(const double (&vec)[2]) {
  for (int i = 0; i < 2; i++) {
    itsVelocity[i] = itsVelocity[i] + vec[i];
  }
}

void Boid::move() {
  for (int i = 0; i < 2; i++) {
    itsPosition[i] = itsPosition[i] + itsVelocity[i];
  }
}
