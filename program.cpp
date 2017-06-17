#include <stdio.h>
#include <time.h>
#include "E101.h"

//  functions used for communicating with the gates wirelessly.
int connect_to_server(char server_addr[15], int port);
int send_to_server(char message[24]);
int receive_from_server(char message[24]);

// exists so that sensor test methods can return both the average white position and the number of white pixels.
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

		// gets the average white position of the test.
		double get_white_position() {
			return whitePosition;
		}

		// gets the number of white pixels detected in the test.
		int get_num_of_white() {
			return numOfWhite;
		}
};

// the camera's dimensions.
int cameraWidth = 320;
int cameraHeight = 240;

// the motor pins.
int motorLeft = 1;
int motorRight = 2;

// the maximum value that the set_motor function can take.
double maxSpeed = 254.0;

// the speed modifier that is applied to the motors when using the set_movement function.
double speed = 0.35;

// the speeds for each quadrant.
double quadrantTwoSpeed = 0.4;
double quadrantThreeSpeed = 0.35;
double quadrantFourSpeed = 0.35;

// the PID movement constants.
double kp = 0.6;
double kd = 0.025;
double ki = 0.001;

// the sum of the previous error signals, used to calculate the integral signal.
double totalError = 0.0;

// the average white position from the last picture taken.
double lastPosition = 0.0;

// how long the camera takes to take a picture.
double cameraTime = 1.0 / 24.0;

// how large a pixel's white component should be for it to be considered white.
int whiteThreshold = 90;

// determines which quadrant's functionality is currently being used.
int quadrant = 3;

// used to count how long the robot detects certain patterns of white for.
int whiteCounter = 0;

// the IR sensor pins.
int frontSensor;
int leftSensor;
int rightSensor;

// how close a wall must be for it to be considered adjacent to the robot.
int frontWallDistance;
int sideWallDistance;

// the PID constants for quadrant four. they will probably need to be quite small.
double kp2 = 0.05;
double kd2 = 0.005;
double ki2 = 0.005;

// sets the robot's movement based off of a direction value ranging from -1.0 (left) to 1.0 (right).
// the second parameter should be 'true' only if you want the robot to reverse in the provided direction.
void set_movement(double direction, bool reverse) {
	// prevent the direction from exceeding the motors' speed limits.
	if (direction > 1.0) {
		direction = 1.0;
	}
	else if (direction < -1.0) {
		direction = -1.0;
	}

	// calculate the speed for each motor using the direction value.
	double left = (int)(maxSpeed * (direction + 1.0) / 2.0);
	double right = (int)(maxSpeed * (direction - 1.0) / 2.0);
	
	// multiply the speeds by the speed modifier.
	left *= speed;
	right *= speed;

	if (reverse) {
		// if the reverse parameter is true, set the motors to turn in the opposite direction.
		set_motor(motorLeft, -left);
		set_motor(motorRight, -right);
	}
	else {
		// otherwise, move normally.
		set_motor(motorLeft, left);
		set_motor(motorRight, right);
	}
}

// returns a sensor test result for the provided row of the camera.
sensorTestResult test_row(int row) {
	// the sum of the white pixels' horizontal positions.
	double sum = 0.0;

	// the number of white pixels found.
	int numOfWhite = 0;

	for (int i = 0; i < cameraWidth; i++) {
		char white = get_pixel(row, i, 3);

		// if the pixel's white component is above a certain threshold, it is considered white and is taken into 
		// account.
		if (white >= whiteThreshold) {
			// the position of the pixel along the row is flipped (cameraWidth - i) and centred (- cameraWidth / 2.0) 
			// when added to the sum.
			sum += (cameraWidth - i) - cameraWidth / 2.0;

			// keep track of the number of white pixels detected.
			numOfWhite++;
		}
	}

	// if there were no white pixels, return a result with all values at 0 (to avoid dividing by zero in the next 
	// part).
	if (numOfWhite == 0) {
		return sensorTestResult(0.0, 0);
	}

	// the average white position is the sum of white positions divided by the number of white pixels, which is then 
	// divided by half of the camera width so that it ranges from -1.0 to 1.0.
	double whitePosition = (sum / numOfWhite) / (cameraWidth / 2);

	return sensorTestResult(whitePosition, numOfWhite);
}

// returns the average of multiple row tests.
sensorTestResult average_row_test(int centre, int numOfTests, int spacing) {
	// the sum of all row tests' white positions.
	double whitePosition = 0.0;
	
	// the sum of all row tests' numbers of white pixels.
	int numOfWhite = 0;

	for (int i = -numOfTests / 2; i < numOfTests / 2; i++) {
		// perform a test i number of spacings away from the central row.
		sensorTestResult testResult = test_row(centre + spacing * i);
		
		// add the test's results to the sums.
		whitePosition += testResult.get_white_position();
		numOfWhite += testResult.get_num_of_white();
	}

	// divide by the number of tests to get the averages.
	whitePosition /= numOfTests;
	numOfWhite /= numOfTests;

	return sensorTestResult(whitePosition, numOfWhite);
}

// gets the number of white pixels in a column.
int num_of_white_in_column(int column) {
	int numOfWhite = 0;

	for (int i = cameraHeight / 4; i < cameraHeight * 3 / 4; i++) {
		char white = get_pixel(i, column, 3);
		if (white >= whiteThreshold) {
			numOfWhite++;
		}
	}

	return numOfWhite;
}

double calculate_position_signal(sensorTestResult result, double kp) {
	// the position signal is the average white position multiplied by a constant.
	return result.get_white_position() * kp;
}

double calculate_derivative_signal(sensorTestResult result, double lastPosition, double kd) {
	// the derivative signal is the difference between the current and last average white positions, divided by the 
	// period between them and multiplied by a constant.
	double difference = result.get_white_position() - lastPosition;
	return (difference / cameraTime) * kd;
}

double calculate_integral_signal(sensorTestResult result, double totalError, double ki) {
	// the integral is sum of the previous errors multiplied by a constant.
	return totalError * ki;
}

// calculates the total PID signal. this method also adds the current error to the total error and overrides the last
// position variable with the current position.
double calculate_pid_signal(sensorTestResult result, double kp, double kd, double ki) {
	totalError += result.get_white_position();
	
	double totalSignal = calculate_position_signal(result, kp)
		+ calculate_derivative_signal(result, lastPosition, kd)
		+ calculate_integral_signal(result, totalError, ki);
		
	lastPosition = result.get_white_position();
	
	return totalSignal;
}

// communicates with the gate wirelessly to tell it to open.
void open_gate() {
	char pass[24];
	connect_to_server((char*)"130.195.6.196", 1024);
	send_to_server((char*)"Please");
	receive_from_server(pass);
	send_to_server(pass);
}

// causes the robot to take a sharp left turn.
void turn_left() {
	set_motor(motorLeft, -0.175 * maxSpeed);
	set_motor(motorRight, -0.2 * maxSpeed);

	// turn for a set period of time.
	sleep1(0, 575000);

	// continue forward.
	set_movement(0.0, false);
}

// causes the robot to take a sharp right turn.
void turn_right() {
	set_motor(motorLeft, 0.2 * maxSpeed);
	set_motor(motorRight, 0.175 * maxSpeed);

	// turn for a set period of time.
	sleep1(0, 575000);

	// continue forward.
	set_movement(0.0, false);
}

// this is a testing mode that displays what the camera sees to the console.
void do_quadrant_zero() {
	take_picture();
	
	for (int i = 50; i < cameraWidth - 50; i++) {
		char white = get_pixel(cameraWidth / 2, i, 3);
		if (white >= whiteThreshold) {
			printf("■");
		}
		else {
			printf(" ");
		}
	}
	printf("\n");
}

void do_quadrant_one() {
	take_picture();
	
	sensorTestResult result = average_row_test(cameraHeight * 3 / 4, 3, cameraHeight / 8);
	
	double percentageWhite = (double)result.get_num_of_white() / cameraWidth;
	if (percentageWhite > 0.2 && percentageWhite < 0.4 && result.get_white_position() > -0.25 && result.get_white_position() < 0.25) {
		// increment the white counter if it appears that the robot is resting on a white line.
		whiteCounter++;
	}
	else {
		// otherwise, reset it.
		whiteCounter = 0;
	}
	
	// once the white counter is high enough, it indicates that the robot has been sitting on the starting line 
	// for long enough, and that it should open the gate wirelessly and begin moving.
	if (whiteCounter > 100) {
		// reset the white counter because it will be used in quadrant two as well.
		whiteCounter = 0;
		
		open_gate();
		
		// wait for the gate to open fully.
		sleep1(1, 0);

		// move into quadrant two.
		quadrant = 2;
	}
}

void do_quadrant_two() {
	speed = quadrantTwoSpeed;

	take_picture();
	
	sensorTestResult result = average_row_test(cameraHeight * 3 / 4, 3, cameraHeight / 8);
	
	double signal = calculate_pid_signal(result, kp, kd, ki);
	
	double percentageWhite = (double)result.get_num_of_white() / cameraWidth;
	if (percentageWhite >= 0.6) {
		// if the percentage of white is high enough, it indicates that the robot is crossing the horizontal line at
		// the start of quadrant three.
		whiteCounter++;
		
		// if this state has been true for long enough, enter quadrant three.
		if (whiteCounter > 2) {
			speed = 0.35;
			set_movement(0.0, false);
			sleep1(0, 500000);
			quadrant = 3;
		}
	}
	else if (percentageWhite > 0.0)	{
		// move normally.
		set_movement(signal, false);

		whiteCounter = 0;
	}
	else {
		// since no white is detected, reverse at an angle.
		set_movement(0.25, true);

		// keep reversing for a set time.
		sleep1(0, 200000);

		// reset the total error.
		totalError = 0.0;

		whiteCounter = 0;
	}
}

void do_quadrant_three() {
	speed = quadrantThreeSpeed;

	take_picture();
	
	sensorTestResult result = average_row_test(cameraHeight * 3 / 4, 3, cameraHeight / 60);
	
	printf("Quadrant 3 ");

	// TODO: detect orange (?) line at the start of quadrant 4 and swap quadrants.
	
	double percentageWhite = (double)result.get_num_of_white() / cameraWidth;
	if (percentageWhite >= 0.6) {
		// the percentage of white is high enough to indicate that the robot is at a full junction,
		// it should therefore turn left.
		printf("Left path.\n");

		// wait before turning.
		sleep1(0, 590000);

		turn_left();
	}
	else if (percentageWhite >= 0.4) {
		// the percentage of white indicates that the robot is at a half junction.
		if (result.get_white_position() < 0) {
			// since the average white position is to the left, turn left.
			printf("Left path.\n");
			
			// wait before turning.
			sleep1(0, 590000);

			turn_left();
		}
		else {
			// otherwise, turn right.
			printf("Right path.\n");

			// wait before turning.
			sleep1(0, 590000);

			turn_right();
		}
	}
	else if (percentageWhite > 0.0)	{
		if (result.get_white_position() > 0.5) {
			// if the white position is too far to the right, turn left.
			set_movement(-0.5, true);
			sleep1(0, 350000);
			set_movement(0.0, false);
			sleep1(0, 300000);
		}
		else if (result.get_white_position() < -0.5) {
			// if it is too far to the left, turn right.
			set_movement(0.5, true);
			sleep1(0, 350000);
			set_movement(0.0, false);
			sleep1(0, 300000);
		}
		else {
			// TODO: this bit of code might need some changing.
			set_movement(calculate_pid_signal(result, kp, kd, 0) * 1.1, false);
		}
	}
	else {
		// since no white is detected, reverse at an angle.
		set_movement(lastPosition * 2 / 3, true);
		totalError = 0.0;
		sleep1(0, 450000);
		set_movement(0.0, false);
		sleep1(0, 450000);
	}
}

void do_quadrant_four() {
	speed = quadrantFourSpeed;

	// this isn't really used for anything, but it causes a brief delay, doesn't it, which is taken into account
	// for the derivative signal.
	take_picture();

	// TODO: get values correctly.
	int frontSensorValue;
	int leftSensorValue;
	int rightSensorValue;

	bool frontWall = frontSensorValue <= frontWallDistance;
	bool leftWall = leftSensorValue <= sideWallDistance;
	bool rightWall = rightSensorValue <= sideWallDistance;

	// TODO: maybe check for left openings and always turn left if one is detected? to make solving every maze possible.

	if (frontWall) {
		// since there is a wall in front of the robot, it needs to turn.
		if (!leftWall) {
			sleep1(0, 250000);

			turn_left();
		}
		else if (!rightWall) {
			sleep1(0, 250000);

			turn_right();
		}
		else {
			// if it is in a dead end, turn right twice to escape.
			// this will not end well, might need some changing.
			turn_right();
			turn_right();
		}

		// reset the PID variables.
		lastPosition = 0.0;
		totalError = 0.0;
	}
	else {
		// TODO: tune constants for quadrant four's PID.
		double error = rightSensorValue - leftSensorValue;

		double positionSignal = error * kp2;

		double difference = error - lastPosition;
		double derivativeSignal = (difference / cameraTime) * kd2;

		double integralSignal = totalError * ki2;

		set_movement(positionSignal + derivativeSignal + integralSignal, false);

		lastPosition = error;
		totalError += error;
	}
}

int main() {
	init();
	
	// loops forever, performing the appropriate actions for the current quadrant.
	while (true) {
		switch (quadrant) {
			case 0:
				do_quadrant_zero();
				break;
			case 1:
				do_quadrant_one();
				break;
			case 2:
				do_quadrant_two();
				break;
			case 3:
				do_quadrant_three();
				break;
			case 4:
				do_quadrant_four();
				break;
		}
	}
	
	return 0;
}
