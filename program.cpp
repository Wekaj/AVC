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
double totalError = 0.0;
double lastPosition = 0.0;
double cameraTime = 1.0 / 24.0;
int whiteCounter = 0;
int quadrant = 3;

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
	double sum = 0.0;
	int numOfWhite = 0;
	for (int i = 0; i < cameraWidth; i++) {
		char white = get_pixel(row, i, 3);
		if (white > 115) {
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
		take_picture();
		
		sensorTestResult result = get_white_position(cameraHeight * 3 / 4);
		totalError += result.get_white_position();

		double positionSignal = result.get_white_position() * kp;

		double difference = result.get_white_position() - lastPosition;
		double derivativeSignal = (difference / cameraTime) * kd;

		double integralSignal = totalError * ki;
		
		double percentageWhite = (double)result.get_num_of_white() / cameraWidth;
		
		//printf("Position: %f\n", positionSignal);
		//printf("Derivative: %f\n", derivativeSignal);
		//printf("Integral: %f\n", integralSignal);
		
		printf("Percentage: %f\n", percentageWhite);
		
		if (quadrant == 1) {
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
		else if (quadrant == 3) {
			printf("Quadrant 3.");
			if (percentageWhite >= 0.8) {
				printf("Full junction.");
				sleep1(0, 200000);
				set_movement(-0.5, false);
				sleep1(0, 200000);
			}
			else if (percentageWhite >= 0.4) {
				if (result.get_white_position() > 0.2) {
					printf("Left path.");
					sleep1(0, 200000);
					set_movement(-1.0, false);
					sleep1(0, 200000);
				}
				else if (get_white_position(cameraHeight / 2).get_num_of_white() > 0) {
					printf("Forward path.");
					sleep1(0, 200000);
					set_movement(0.0, false);
					sleep1(0, 200000);
				}
				else {
					printf("Right path.");
					sleep1(0, 200000);
					set_movement(1.0, false);
					sleep1(0, 200000);
				}
			}
			else if (percentageWhite > 0.0)	{
				set_movement(positionSignal + derivativeSignal + integralSignal, false);
			}
			else {
				set_movement(0.25, true);
				integralSignal = 0.0;
				sleep1(0, 200000);
			}
		}
		else if (percentageWhite >= 0.8) {
			whiteCounter++;
			
			if (whiteCounter > 2) {
				quadrant = 3;
				set_movement(0.0, false);
				sleep1(0, 500000);
			}
		}
		else if (percentageWhite > 0.0)	{
			set_movement(positionSignal + derivativeSignal + integralSignal, false);
			whiteCounter = 0;
		}
		else {
			set_movement(0.25, true);
			sleep1(0, 200000);
			integralSignal = 0.0;
			whiteCounter = 0;
		}
		
		lastPosition = result.get_white_position();

		//printf("%f\n", result.get_white_position());
	}

	reset();

	return 0;
}
