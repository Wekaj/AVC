#include <stdio.h>
#include <time.h>
#include "E101.h"

int cameraWidth = 320;
int cameraHeight = 240;
int motorLeft = 1;
int motorRight = 2;

void set_movement(double direction) {
	set_motor(motorLeft, (int)(255.0 * (direction + 1.0) / 2.0));
	set_motor(motorRight, (int)(255.0 * (direction - 1.0) / 2.0));
}

double get_white_position() {
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
	return (sum / numOfWhite) / (cameraWidth / 2);
}

void reset() {
	set_motor(motorLeft, 0);
	set_motor(motorRight, 0);
}

int main() {
	init();
	
	while (true) {
		double direction = get_white_position();
		set_movement(direction);
		printf("%f\n", direction);
		sleep1(0, 100000);
	}
	
	reset();
	
	return 0;
}
