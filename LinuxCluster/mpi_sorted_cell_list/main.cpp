#include "header.hpp"
#include <mpi.h>
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

int main(int argc, char *argv[]) {
  //initialization
  std::ofstream outfile;
  double t1,t2;
  int rank,nprocs;
  MPI_Init(&argc,&argv);
  t1 = MPI_Wtime();
  MPI_Comm_rank(MPI_COMM_WORLD,&rank);
  MPI_Comm_size(MPI_COMM_WORLD,&nprocs);
  MPI_Status status;
  int blocklengths[3]={2,2,2};
  MPI_Datatype types[3]={MPI_FLOAT,MPI_FLOAT,MPI_INT};
  MPI_Aint displacements[3];
  MPI_Datatype BoidType;
  MPI_Aint float_ex;
  MPI_Type_extent(MPI_FLOAT,&float_ex);
  displacements[0]= (MPI_Aint) 0;
  displacements[1]= float_ex + float_ex;
  displacements[2]= float_ex * 4;
  MPI_Type_struct(3,blocklengths,displacements,types,&BoidType);
  MPI_Type_commit(&BoidType);
  int rank_master = 0;

  const int noOfBoids = 1000000;
  const float sizeOfBoundary = 20000.0;
  const float maxSpeed = 40.0;
  const float maxAcceleration = 5.0;
  const float perception = 200.0;
  const int cellDim = 100;
  const float R = 10000.0;
  const float V = 40.0;
  std::vector<Boid> boid(noOfBoids);
  Boid boidBuffer[noOfBoids];
  const int timeStep = 10;
  if (rank == rank_master) {
    for (int i = 0; i < noOfBoids; i++) {
      boidBuffer[i].index = i;
      float x = R * cos(2 * M_PI * randnum(0.0,1.0)) * sqrt(randnum(0.0,1.0));
      float y = R * sin(2 * M_PI * randnum(0.0,1.0)) * sqrt(randnum(0.0,1.0));
      boidBuffer[i].set_itsPosition(x,y);
      float vx = V * cos(2 * M_PI * randnum(0.0,1.0)) * sqrt(randnum(0.0,1.0));
      float vy = V * sin(2 * M_PI * randnum(0.0,1.0)) * sqrt(randnum(0.0,1.0));
      boidBuffer[i].set_itsVelocity(vx,vy);
    }
  }
  MPI_Bcast(boidBuffer,noOfBoids,BoidType,rank_master,MPI_COMM_WORLD);
  for (int i = 0; i < noOfBoids; i++) {
    boid[i] = boidBuffer[i];
  }

  int count;
  if (noOfBoids % nprocs != 0) {
    std::cout << "Code only works when the modulus of noOfBoids divided by no of processes is zero" << std::endl;
    abort();
  }
  count = noOfBoids / nprocs;
  int start = count * rank;
  int stop = start + count;
  Boid updateAtEachProc[count];

  outfile.open("data.csv");
  if (rank == rank_master) {
    outfile << boid[0].get_itsPosition(0);
    for (int i = 0; i < noOfBoids; i++) {
      for (int j = 0; j < 2; j++) {
        if (i == 0 and j == 0) {
          continue;
        }
        outfile << ", " << boid[i].get_itsPosition(j);
      }
    }
    outfile << std::endl;
  }

  CellLinkedList<cellDim,noOfBoids> linkedList;

  for (int iteration = 0; iteration < timeStep; iteration++) {
    //main part
    linkedList.clear();
    //mark the boids by order in the array and by its location
    for (int i = 0; i < noOfBoids; i++) {
      int cellRow = floor((boid[i].get_itsPosition(0) + (sizeOfBoundary / 2.0)) / perception);
      int cellCol = floor((boid[i].get_itsPosition(1) + (sizeOfBoundary / 2.0)) / perception);
      boid[i].cellIndex = cellRow * cellDim + cellCol;
    }
    //sort
    std::sort(boid.begin(), boid.end(), compareByCellIndex);
    for (int i = 0; i < count; i++) {
      updateAtEachProc[i] =  boid[i + start];
    }
    //insert into cell list
    for (int i = 0; i < noOfBoids; i++) {
      int cellIndex = boid[i].cellIndex;
      linkedList.linkedCellList[i] = linkedList.head[cellIndex];
      linkedList.head[cellIndex] = i;
    }
    for (int i = start; i < stop; i++) {
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
      updateAtEachProc[i - start].accelerate(acceleration);
      updateAtEachProc[i - start].setMaxSpeed(maxSpeed);
      updateAtEachProc[i - start].setBoundary(sizeOfBoundary);
      updateAtEachProc[i - start].move();
    }

    MPI_Allgather(updateAtEachProc,count,BoidType,&boidBuffer,count,BoidType,MPI_COMM_WORLD);
    //sort
    for (int i = 0; i < noOfBoids; i++) {
      boid[i] = boidBuffer[i];
    }
    std::sort(boid.begin(), boid.end(), compareByIndex);
    //output

    if (rank == rank_master) {
      outfile << boid[0].get_itsPosition(0);
      for (int i = 0; i < noOfBoids; i++) {
        for (int j = 0; j < 2; j++) {
          if (i == 0 and j == 0) {
            continue;
          }
          outfile << ", " << boid[i].get_itsPosition(j);
        }
      }
      outfile << std::endl;
    }
  }
  t2 = MPI_Wtime();
  if (rank == rank_master) {
    printf( "Elapsed time is %f\n", t2 - t1 );
  }
  MPI_Finalize();
  return 0;
}

float randnum (float a, float b) {
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  static std::default_random_engine generator (seed);
  std::uniform_real_distribution<float> distribution (a,b);
  return distribution(generator);
}
