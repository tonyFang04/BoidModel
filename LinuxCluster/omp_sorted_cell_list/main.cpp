#include "header.hpp"
#include <omp.h>
#include <stdio.h>
bool compareByCellIndex(const Boid & A, const Boid & B)
{
    return (A.cellIndex < B.cellIndex) ||
           ((A.cellIndex < B.cellIndex) && (A.index < B.index));
}

bool compareByIndex(const Boid & A, const Boid & B)
{
    return (A.index < B.index);
}

int main() {
  //initialization
  double START;
  double END;
  START = omp_get_wtime();
  const int noOfBoids = 1000000;
  const float sizeOfBoundary = 20000.0;
  const float maxSpeed = 40.0;
  const float maxAcceleration = 5.0;
  const float perception = 200.0;
  const int cellDim = 100;
  const float R = 10000.0;
  const float V = 40.0;
  std::vector<Boid> boid(noOfBoids);
  std::vector<Boid> updates(noOfBoids);
  const int timeStep = 10;
  for (int i = 0; i < noOfBoids; i++) {
    boid[i].index = i;
    float x = R * cos(2 * M_PI * randnum(0.0,1.0)) * sqrt(randnum(0.0,1.0));
    float y = R * sin(2 * M_PI * randnum(0.0,1.0)) * sqrt(randnum(0.0,1.0));
    boid[i].set_itsPosition(x,y);
    updates[i].set_itsPosition(x,y);
    float vx = V * cos(2 * M_PI * randnum(0.0,1.0)) * sqrt(randnum(0.0,1.0));
    float vy = V * sin(2 * M_PI * randnum(0.0,1.0)) * sqrt(randnum(0.0,1.0));
    boid[i].set_itsVelocity(vx,vy);
    updates[i].set_itsVelocity(vx,vy);
  }

  std::ofstream outfile;
  outfile.open("data.csv");
  outfile << updates[0].get_itsPosition(0);
  for (int i = 0; i < noOfBoids; i++) {
    for (int j = 0; j < 2; j++) {
      if (i == 0 and j == 0) {
        continue;
      }
      outfile << ", " << updates[i].get_itsPosition(j);
    }
  }
  outfile << std::endl;

  CellLinkedList<cellDim,noOfBoids> linkedList;

  for (int iteration = 0; iteration < timeStep; iteration++) {
    //main part
    linkedList.clear();
    //mark the boids by order in the array and by its location
    for (int i = 0; i < noOfBoids; i++) {
      int cellRow = floor((boid[i].get_itsPosition(0) + (sizeOfBoundary / 2.0)) / perception);
      int cellCol = floor((boid[i].get_itsPosition(1) + (sizeOfBoundary / 2.0)) / perception);
      boid[i].cellIndex = cellRow * cellDim + cellCol;
      updates[i].cellIndex = boid[i].cellIndex;
    }
    //sort
    std::sort(boid.begin(), boid.end(), compareByCellIndex);
    for (int i = 0; i < noOfBoids; i++) {
      updates[i] = boid[i];
    }
    //insert into cell list
    for (int i = 0; i < noOfBoids; i++) {
      int cellIndex = boid[i].cellIndex;
      linkedList.linkedCellList[i] = linkedList.head[cellIndex];
      linkedList.head[cellIndex] = i;
    }
    #pragma omp parallel
    {
      int nthreads = omp_get_num_threads();
      int chunk = noOfBoids / nthreads;
      #pragma omp for schedule(static,chunk)
      for (int i = 0; i < noOfBoids; i++) {
      float nearbyBoids = 0.0;
      float averageVelocity[2] = {0.0,0.0};
      float centreOfMass[2] = {0.0,0.0};
      float displacement[2] = {0.0,0.0};
      float separation[2] = {0.0,0.0};
      float acceleration[2] = {0.0,0.0};
      int cellRow = floor((boid[i].get_itsPosition(0) + (sizeOfBoundary / 2.0)) / perception);
      int cellCol = floor((boid[i].get_itsPosition(1) + (sizeOfBoundary / 2.0)) / perception);
      for (int ci = cellRow - 1; ci < cellRow + 2; ci++) {
        for (int cj = cellCol - 1; cj < cellCol + 2; cj++) {
          int j = linkedList.head[((ci + cellDim) % cellDim) * cellDim + ((cj + cellDim) % cellDim)];
          if (j != -1) {
            float distance = boid[i].getDistance(boid[j]);
            if (i != j && distance < perception) {
              for (int k = 0; k < 2; k++){
                averageVelocity[k] += boid[j].get_itsVelocity(k);
                centreOfMass[k] += boid[j].get_itsPosition(k);
                displacement[k] = boid[i].get_itsPosition(k) - boid[j].get_itsPosition(k);
                displacement[k] = displacement[k] / (distance + 1.0e-10);
                separation[k] += displacement[k];
              }
              nearbyBoids += 1.0;
            }
            j = linkedList.linkedCellList[j];
            while (j != -1) {
              distance = boid[i].getDistance(boid[j]);
              if (i != j && distance < perception) {
                for (int k = 0; k < 2; k++){
                  averageVelocity[k] += boid[j].get_itsVelocity(k);
                  centreOfMass[k] += boid[j].get_itsPosition(k);
                  displacement[k] = boid[i].get_itsPosition(k) - boid[j].get_itsPosition(k);
                  displacement[k] = displacement[k] / (distance + 1.0e-10);
                  separation[k] += displacement[k];
                }
                nearbyBoids += 1.0;
              }
              j = linkedList.linkedCellList[j];
            }
          }
        }
      }
      if (nearbyBoids > 0) {
        for (int k = 0; k < 2; k++) {
          averageVelocity[k] = averageVelocity[k] / nearbyBoids;
          centreOfMass[k] = centreOfMass[k] / nearbyBoids;
          separation[k] = separation[k] / nearbyBoids;
          acceleration[k] = ((averageVelocity[k] - boid[i].get_itsVelocity(k)) / 4.0) +
                            ((centreOfMass[k] - boid[i].get_itsPosition(k)) / 100.0) +
                            ((boid[i].get_itsVelocity(k) - separation[k]) / 2.0);
        }
        float accelerationNorm = norm(acceleration);
        if (accelerationNorm > maxAcceleration) {
          for (int k = 0; k < 2; k++) {
            acceleration[k] = (acceleration[k] / accelerationNorm) * maxAcceleration;
          }
        }
      }
      updates[i].accelerate(acceleration);
      updates[i].setMaxSpeed(maxSpeed);
      updates[i].setBoundary(sizeOfBoundary);
      updates[i].move();
    }
    }

    //sort
    std::sort(updates.begin(), updates.end(), compareByIndex);
    //output

    outfile << updates[0].get_itsPosition(0);
    for (int i = 0; i < noOfBoids; i++) {
      for (int j = 0; j < 2; j++) {
        if (i == 0 and j == 0) {
          continue;
        }
        outfile << ", " << updates[i].get_itsPosition(j);
      }
    }
    outfile << std::endl;

    //update old array
    for (int i = 0; i < noOfBoids; i++) {
      boid[i] = updates[i];
    }
  }
  END = omp_get_wtime();
  printf("Elapsed time is %f\n", END - START);
  return 0;
}

float randnum (float a, float b) {
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  static std::default_random_engine generator (seed);
  std::uniform_real_distribution<float> distribution (a,b);
  return distribution(generator);
}
