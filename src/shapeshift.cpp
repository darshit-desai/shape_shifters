#include "../include/shapeshift.hpp"

using std::vector;

Shapeshifters::Shapeshifters() {
  center_x = 0;
  center_y = 0;
  radius = 9.6;
  circumRadius = 9.6;
}

void Shapeshifters::setCenter(double x, double y) {
  xCenter = x;
  yCenter = y;
}

void Shapeshifters::setNumberOfRobots(double n) { numberOfRobots = n; }

int Shapeshifters::getNumberOfRobots() { return numberOfRobots; }

vector<vector<double>> Shapeshifters::shapeCircle() {
  vector<vector<double>> shapeCircle(numberOfRobots, vector<double>(2, 0));
  double angle = 0;
  double angleIncrement = 2 * M_PI / numberOfRobots;
  for (int i = 0; i < numberOfRobots; i++) {
    shapeCircle[i][0] = xCenter + radius * cos(angle);
    shapeCircle[i][1] = yCenter + radius * sin(angle);
    angle += angleIncrement;
  }
  return shapeCircle;
}

vector<vector<double>> Shapeshifters::shapeSquare() {
  vector<vector<double>> shapeSquare(numberOfRobots, vector<double>(2, 0));
  double x = xCenter - sideLength / 2;
  double y = yCenter - sideLength / 2;
//   for (int i = 0; i < numberOfRobots; i++) {
//     if (i < numberOfRobots / 4) {
//       x = x + sideLength / (numberOfRobots / 4);
//     } else if (i < numberOfRobots / 2) {
//       y = y + sideLength / (numberOfRobots / 4);
//     } else if (i < 3 * numberOfRobots / 4) {
//       x = x - sideLength / (numberOfRobots / 4);
//     } else {
//       y = y - sideLength / (numberOfRobots / 4);
//     }
//     shapeSquare[i][0] = x;
//     shapeSquare[i][1] = y;
//   }
//   return shapeSquare;
}

vector<vector<double>> Shapeshifters::shapeSquare() {
  vector<vector<double>> shapeSquare(numberOfRobots,
                                            vector<double>(2, 0));
  double x = xCenter - circumRadius;
  double y = yCenter + (3 * circumRadius / 2 - circumRadius);
  for (int i = 0; i < numberOfRobots; i++) {
    if (i < numberOfRobots / 3) {
      x = x + circumRadius / (numberOfRobots / 3);
      y = y - circumRadius / (numberOfRobots / 3);
    } else if (i < 2 * numberOfRobots / 3) {
      x = x + circumRadius / (numberOfRobots / 3);
      y = y + circumRadius / (numberOfRobots / 3);
    } else {
      x = x - 2 * (circumRadius / (numberOfRobots / 3));
    }
    shapeSquare[i][0] = x;
    shapeSquare[i][1] = y;
  }

  return shapeSquare;
}