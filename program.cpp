#include <stdio.h>
#include <time.h>
#include "E101.h"

class sensorTestResult {
	private: 
		double whitePosition;
		int numOfWhite;

	public:
		sensorTestResult(double whitePosition, int numOfWhite) {
			sensorTestResult::whitePosition = whitePosition;
			sensorTestResult::numOfWhite = numOfWhite;
		}

		double get_white_position() {
			return whitePosition;
		}

		int get_num_of_white() {
			return numOfWhite;
		}
};

int cameraWidth = 320;
int cameraHeight = 240;
int motorLeft = 1;
int motorRight = 2;

void set_movement(double direction, bool reverse) {
	double left = (int)(255.0 * (direction + 1.0) / 2.0);
	double right = (int)(255.0 * (direction - 1.0) / 2.0);

	// make the robot slower for testing purposes.
	left /= 4.0;
	right /= 4.0;

	if (reverse) {
		set_motor(motorLeft, -left);
		set_motor(motorRight, -right);
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
		return sensorTestResult(0.0, 0);
	}

	return sensorTestResult((sum / numOfWhite) / (cameraWidth / 2), numOfWhite);
}

void reset() {
	set_motor(motorLeft, 0);
	set_motor(motorRight, 0);
}

int main() {
  init();
  
  while (true) {
    sensorTestResult result = get_white_position();
    
    if (result.get_num_of_white() == 0) {
		set_movement(0.25, true);
	}
	else {
		set_movement(result.get_white_position(), false);
	}
    
    printf("%f\n", result.get_white_position());
    sleep1(0, 100000);
  }
  
  reset();
  
  return 0;
}
