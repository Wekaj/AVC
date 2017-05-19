#include <stdio.h>
#include <time.h>
#include "E101.h"

class sensorTestResult {
	private: 
		double whitePosition;
		int numOfWhite;

	public:
		sensorTestResult() {
			whitePosition = 0.0;
			numOfWhite = 0;
		}
	
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

int delay = 10000;
int cameraWidth = 320;
int cameraHeight = 240;
int motorLeft = 1;
int motorRight = 2;
double kp = 0.6;
double kd = 0.0001;
double ki = 0.001;
double maxSpeed = 254.0;

void set_movement(double direction, bool reverse) {
	// prevent the direction from exceeding the motors' speed limits.
	if (direction > 1.0) {
		direction = 1.0;
	}
	else if (direction < -1.0) {
		direction = -1.0;
	}

	double left = (int)(maxSpeed * (direction + 1.0) / 2.0);
	double right = (int)(maxSpeed * (direction - 1.0) / 2.0);
	
	left /= 2.5;
	right /= 2.5;

	if (reverse) {
		set_motor(motorLeft, -left);
		set_motor(motorRight, -right);
	}
	else {
		set_motor(motorLeft, left);
		set_motor(motorRight, right);
	}
}

sensorTestResult get_white_position(int row) {
	take_picture();
	
	double sum = 0.0;
	int numOfWhite = 0;
	for (int i = 0; i < cameraWidth; i++) {
		char white = get_pixel(row, i, 3);
		if (white > 100) {
			sum += (cameraWidth - i) - cameraWidth / 2.0;
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
		sensorTestResult results[4];
		double totalError = 0.0;
		for (int i = 0; i < 4; i++) {
			results[i] = get_white_position(cameraHeight / 2 + cameraHeight * i / 8 - 1);
			totalError += results[i].get_white_position();
		}

		double positionSignal = results[0].get_white_position() * kp;

		double difference = results[0].get_white_position() - results[1].get_white_position();
		double derivativeSignal = (difference / (delay / 1000000.0)) * kd;

		double integralSignal = totalError * ki;
		
		double percentageWhite = (double)results[0].get_num_of_white() / cameraWidth;
		
		printf("Position: %f\n", positionSignal);
		printf("Derivative: %f\n", derivativeSignal);
		printf("Integral: %f\n", integralSignal);
		
		//if (percentageWhite >= 0.75) {
			// turn left.
		//}
		//else if (percentageWhite >= 0.25) {
			// if the pixels are mainly on the left, turn left.
			// otherwise:
			// check vertically for a forwards path. if it exists, go forwards.
			// otherwise, turn right.
		//}
		if (percentageWhite > 0.0)	{
			set_movement(positionSignal + derivativeSignal + integralSignal, false);
		}
		else {
			set_movement(0.25, true);
		}

		printf("%f\n", results[0].get_white_position());
		sleep1(0, delay);
	}

	reset();

	return 0;
}
