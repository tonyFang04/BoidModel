#include <iostream>
#include <cmath>
#include <random>
#include <chrono>
#include <fstream>
#include <numeric>
#include <iostream>
#include <algorithm>
#include <vector>
double randnum (double a, double b);

template<int size>
double norm(const double (&vec)[size]) {
  double tot = 0;
  for (int i = 0; i < size; i++) {
    tot += vec[i] * vec[i];
  }
  return sqrt(tot);
}

template<int size>
double distance(const double (&vec1)[size], const double (&vec2)[size]) {
  double difference[size];
  for (int i = 0; i < size; i++) {
    difference[i] = vec1[i] - vec2[i];
  }
  return norm(difference);
}

class Boid {
  private:
    double itsPosition[2];
    double itsVelocity[2];
  public:
    int cellIndex, index;
    void set_itsPosition(double x, double y);
    void set_itsVelocity(double vx, double vy);
    double getDistance(const Boid & other) const;
    void setBoundary(double sizeOfBoundary);
    void setMaxSpeed(double maxSpeed);
    void accelerate(const double (&vec)[2]);
    void move();
    double get_itsPosition(int i) const;
    double get_itsVelocity(int i) const;
};

template<int cellDim,int noOfBoids>
struct CellLinkedList {
  int head[cellDim * cellDim];
  int linkedCellList[noOfBoids];
  void clear();
};

template<int cellDim,int noOfBoids>
void CellLinkedList<cellDim,noOfBoids>::clear() {
  for (int i = 0; i < cellDim * cellDim; i++) {
    head[i] = -1;
  }
  for (int i = 0; i < noOfBoids; i++) {
    linkedCellList[i] = -1;
  }
}
