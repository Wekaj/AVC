#include <stdio.h>
#include <time.h>
#include "E101.h"

int connect_to_server(char server_addr[15], int port);
int send_to_server(char message[24]);
int receive_from_server(char message[24]);

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

int cameraWidth = 320;
int cameraHeight = 240;
int motorLeft = 1;
int motorRight = 2;
double kp = 0.6;
double kd = 0.025;
double ki = 0.001;
double maxSpeed = 254.0;
double speed = 0.3;
double totalError = 0.0;
double lastPosition = 0.0;
double cameraTime = 1.0 / 24.0;
int whiteCounter = 0;
int quadrant = 3;
int reverseCounter = 0;

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
	
	left *= speed;
	right *= speed;

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

int get_column(int column) {
	int whitePixels = 0;
	for (int i = cameraHeight / 4; i < cameraHeight * 3 / 4; i++) {
		char white = get_pixel(i, column, 3);
		if (white > 90) {
			whitePixels++;
		}
	}
	return whitePixels;
}

sensorTestResult calculateAverageResult(int centre, int numOfTries, int spacing) {
	double whitePosition = 0.0;
	int numOfWhite = 0;
	for (int i = -numOfTries / 2; i < numOfTries / 2; i++) {
		sensorTestResult testResult = get_white_position(centre + spacing * i);
		
		whitePosition += testResult.get_white_position();
		numOfWhite += testResult.get_num_of_white();
	}
	whitePosition /= numOfTries;
	numOfWhite /= numOfTries;
	return sensorTestResult(whitePosition, numOfWhite);
}

double calculatePositionSignal(sensorTestResult result, double kp) {
	return result.get_white_position() * kp;
}

double calculateDerivativeSignal(sensorTestResult result, double lastPosition, double kd) {
	double difference = result.get_white_position() - lastPosition;
	return (difference / cameraTime) * kd;
}

double calculateIntegralSignal(sensorTestResult result, double totalError, double ki) {
	return totalError * ki;
}

double calculatePIDSignal(sensorTestResult result, double kp, double kd, double ki) {
	totalError += result.get_white_position();
	
	double totalSignal = calculatePositionSignal(result, kp)
		+ calculateDerivativeSignal(result, lastPosition, kd)
		+ calculateIntegralSignal(result, totalError, ki);
		
	lastPosition = result.get_white_position();
	
	return totalSignal;
}

void doQuadrantZero() {
	take_picture();
	
	for (int i = 50; i < cameraWidth - 50; i++) {
		char white = get_pixel(cameraWidth / 2, i, 3);
		if (white > 90) {
			printf("■");
		}
		else {
			printf(" ");
		}
	}
	printf("\n");
}

void doQuadrantOne() {
	take_picture();
	
	sensorTestResult result = calculateAverageResult(cameraHeight * 3 / 4, 3, cameraHeight / 8);
		
	double percentageWhite = (double)result.get_num_of_white() / cameraWidth;
	if (percentageWhite > 0.2 && percentageWhite < 0.4 && result.get_white_position() > -0.25 && result.get_white_position() < 0.25) {
		whiteCounter++;
	}
	else {
		whiteCounter = 0;
	}
	
	if (whiteCounter > 100) {
		quadrant = 2;
		
		char pass[24];
		connect_to_server((char*)"130.195.6.196", 1024);
		send_to_server((char*)"Please");
		receive_from_server(pass);
		send_to_server(pass);
		
		whiteCounter = 0;
		
		sleep1(2, 0);
	}
}

void doQuadrantTwo() {
	take_picture();
	
	sensorTestResult result = calculateAverageResult(cameraHeight * 3 / 4, 3, cameraHeight / 8);
	
	double signal = calculatePIDSignal(result, kp, kd, ki);
	
	double percentageWhite = (double)result.get_num_of_white() / cameraWidth;
	if (percentageWhite >= 0.8) {
		whiteCounter++;
		
		if (whiteCounter > 2) {
			quadrant = 3;
			speed = 0.3;
			set_movement(0.0, false);
			sleep1(0, 500000);
		}
	}
	else if (percentageWhite > 0.0)	{
		set_movement(signal, false);
		whiteCounter = 0;
	}
	else {
		set_movement(0.25, true);
		sleep1(0, 200000);
		totalError = 0.0;
		whiteCounter = 0;
	}
}

void doQuadrantThree() {
	take_picture();
	
	sensorTestResult result = get_white_position(cameraHeight * 3 / 4);
	
	double percentageWhite = (double)result.get_num_of_white() / cameraWidth;
	if (percentageWhite >= 0.75) {
		printf("Left path.\n");
		sleep1(0, 600000);
		set_motor(motorLeft, -0.175 * maxSpeed);
		set_motor(motorRight, -0.2 * maxSpeed);
		sleep1(0, 575000);
		set_movement(0.0, false);
	}
	else if (percentageWhite >= 0.6) {
		if (result.get_white_position() < 0.2) {
			printf("Left path.\n");
			sleep1(0, 600000);
			set_motor(motorLeft, -0.175 * maxSpeed);
			set_motor(motorRight, -0.2 * maxSpeed);
			sleep1(0, 575000);
			set_movement(0.0, false);
		}
		else {
			printf("Right path.\n");
			sleep1(0, 600000);
			set_motor(motorLeft, 0.2 * maxSpeed);
			set_motor(motorRight, 0.175 * maxSpeed);
			sleep1(0, 575000);
			set_movement(0.0, false);
		}
	}
	else if (percentageWhite > 0.0)	{
		set_movement(calculatePIDSignal(result, kp, kd, 0) * 1.3, false);
	}
	else {
		set_movement(-lastPosition * 2 / 3, true);
		totalError = 0.0;
		sleep1(0, 900000);
		set_movement(0.0, false);
		sleep1(0, 900000);
	}
}

int main() {
	init();
	
	while (true) {
		switch (quadrant) {
			case 0:
				doQuadrantZero();
				break;
			case 1:
				doQuadrantOne();
				break;
			case 2:
				doQuadrantTwo();
				break;
			case 3:
				doQuadrantThree();
				break;
		}
	}

	return 0;
}
