#include <stdio.h>
#include <time.h>
#include "E101.h"

class sensorTestResult {
  private double whitePosition;
  private int numOfWhite;
  
  public sensorTestResult(double whitePosition, int numOfWhite) {
    this.whitePosition = whitePosition;
    this.numOfWhite = numOfWhite;
  }
  
  public double get_white_position() {
    return whitePosition;
  }
  
  public int get_num_of_white() {
    return numOfWhite;
  }
}

int cameraWidth = 320;
int cameraHeight = 240;
int motorLeft = 1;
int motorRight = 2;

void set_movement(double direction, bool reverse) {
  double left = (int)(255.0 * (direction + 1.0) / 2.0);
  double right = (int)(255.0 * (direction - 1.0) / 2.0);
  
  if (reverse) {
    set_motor(motorLeft, -right);
    set_motor(motorRight, -left);
  }
  else {
    set_motor(motorLeft, left);
    set_motor(motorRight, right);
  }
}

sensorTestResult get_white_position() {
  take_picture();
  
  double sum = 0.0;
  int numOfWhite = 0;
  for (int i = 0; i < cameraWidth; i++) {
    char white = get_pixel(cameraHeight / 2, i, 3);
    if (white > 127) {
      sum += i - cameraWidth / 2.0;
      numOfWhite++;
    }
  }
  
  if (numOfWhite == 0) {
    return 0;
  }
  
  return new sensorTestResult((sum / numOfWhite) / (cameraWidth / 2), numOfWhite);
}

void reset() {
  set_motor(motorLeft, 0);
  set_motor(motorRight, 0);
}

int main() {
  init();
  
  while (true) {
    sensorTestResult result = get_white_position();
    set_movement(result.get_white_position(), result.get_num_of_white() == 0);
    printf("%f\n", result.get_white_position());
    sleep1(0, 100000);
  }
  
  reset();
  
  return 0;
}
