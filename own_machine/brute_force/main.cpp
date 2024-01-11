#include "header.hpp"
int main() {
  //initialization
  const int noOfBoids = 20000;
  const double sizeOfBoundary = 20000.0;
  const double maxSpeed = 40.0;
  const double maxAcceleration = 5.0;
  const double perception = 200.0;
  const double R = 10000.0;
  const double V = 40.0;
  Boid boid[noOfBoids];
  Boid updates[noOfBoids];
  const int timeStep = 15;
  for (int i = 0; i < noOfBoids; i++) {
    double x = R * cos(2 * M_PI * randnum(0.0,1.0)) * sqrt(randnum(0.0,1.0));
    double y = R * sin(2 * M_PI * randnum(0.0,1.0)) * sqrt(randnum(0.0,1.0));
    boid[i].set_itsPosition(x,y);
    updates[i].set_itsPosition(x,y);
    double vx = V * cos(2 * M_PI * randnum(0.0,1.0)) * sqrt(randnum(0.0,1.0));
    double vy = V * sin(2 * M_PI * randnum(0.0,1.0)) * sqrt(randnum(0.0,1.0));
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

  for (int iteration = 0; iteration < timeStep; iteration++) {
    //main part
    for (int i = 0; i < noOfBoids; i++) {
      double nearbyBoids = 0.0;
      double averageVelocity[2] = {0.0,0.0};
      double centreOfMass[2] = {0.0,0.0};
      double displacement[2] = {0.0,0.0};
      double separation[2] = {0.0,0.0};
      double acceleration[2] = {0.0,0.0};
      for (int j = 0; j < noOfBoids; j++) {
        double distance = boid[i].getDistance(boid[j]);
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
        double accelerationNorm = norm(acceleration);
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
  return 0;
}

double randnum (double a, double b) {
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  static std::default_random_engine generator (seed);
  std::uniform_real_distribution<double> distribution (a,b);
  return distribution(generator);
}
