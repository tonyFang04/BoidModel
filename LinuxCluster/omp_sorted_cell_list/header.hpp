#include <iostream>
#include <cmath>
#include <random>
#include <chrono>
#include <fstream>
#include <numeric>
#include <iostream>
#include <algorithm>
#include <vector>
float randnum (float a, float b);

template<int size>
float norm(const float (&vec)[size]) {
  float tot = 0;
  for (int i = 0; i < size; i++) {
    tot += vec[i] * vec[i];
  }
  return sqrt(tot);
}

template<int size>
float distance(const float (&vec1)[size], const float (&vec2)[size]) {
  float difference[size];
  for (int i = 0; i < size; i++) {
    difference[i] = vec1[i] - vec2[i];
  }
  return norm(difference);
}

class Boid {
  private:
    float itsPosition[2];
    float itsVelocity[2];
  public:
    int cellIndex, index;
    void set_itsPosition(float x, float y);
    void set_itsVelocity(float vx, float vy);
    float getDistance(const Boid & other) const;
    void setBoundary(float sizeOfBoundary);
    void setMaxSpeed(float maxSpeed);
    void accelerate(const float (&vec)[2]);
    void move();
    float get_itsPosition(int i) const;
    float get_itsVelocity(int i) const;
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
