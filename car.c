#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#define OBSTACLE_THRESHOLD 25
#define SAFE_DISTANCE 60
#define CRITICAL_DISTANCE 15
#define DELAY_MS 100
#define HISTORY_SIZE 10
#define STUCK_THRESHOLD 5

typedef enum
{
	NORTH,
	SOUTH,
	EAST,
	WEST,
	NORTH_EAST,
	NORTH_WEST,
	SOUTH_EAST,
	SOUTH_WEST,
	STOP
} Direction;

typedef struct
{
	int front;
	int back;
	int left;
	int right;
} SensorData;

typedef struct
{
	Direction lastDirections[HISTORY_SIZE];
	int historyIndex;
	int stuckCounter;
	Direction preferredDirection;
	float momentum;
} NavigationState;

NavigationState navState = {.historyIndex = 0,
                            .stuckCounter = 0,
                            .preferredDirection = NORTH,
                            .momentum = 1.0};

int readFrontSensor() { return rand() % 100; }

int readBackSensor() { return rand() % 100; }

int readLeftSensor() { return rand() % 100; }

int readRightSensor() { return rand() % 100; }

void moveNorth() { printf(">>> Moving NORTH (Forward)\n"); }

void moveSouth() { printf(">>> Moving SOUTH (Backward)\n"); }

void moveEast() { printf(">>> Moving EAST (Right)\n"); }

void moveWest() { printf(">>> Moving WEST (Left)\n"); }

void moveNorthEast() { printf(">>> Moving NORTH-EAST (Forward-Right)\n"); }

void moveNorthWest() { printf(">>> Moving NORTH-WEST (Forward-Left)\n"); }

void moveSouthEast() { printf(">>> Moving SOUTH-EAST (Backward-Right)\n"); }

void moveSouthWest() { printf(">>> Moving SOUTH-WEST (Backward-Left)\n"); }

void stopMotors() { printf(">>> EMERGENCY STOP - All sides blocked!\n"); }

SensorData readAllSensors()
{
	SensorData data;
	data.front = readFrontSensor();
	data.back = readBackSensor();
	data.left = readLeftSensor();
	data.right = readRightSensor();
	return data;
}

bool isCriticalSituation(SensorData sensors)
{
	return (
	    sensors.front < CRITICAL_DISTANCE || sensors.back < CRITICAL_DISTANCE ||
	    sensors.left < CRITICAL_DISTANCE || sensors.right < CRITICAL_DISTANCE);
}

bool isStuck(NavigationState *state)
{
	if (state->historyIndex < HISTORY_SIZE)
		return false;

	int backwardMoves = 0;
	int differentDirections = 0;
	Direction firstDir = state->lastDirections[0];

	for (int i = 0; i < HISTORY_SIZE; i++)
	{
		if (state->lastDirections[i] == SOUTH ||
		    state->lastDirections[i] == SOUTH_EAST ||
		    state->lastDirections[i] == SOUTH_WEST)
		{
			backwardMoves++;
		}
		if (state->lastDirections[i] != firstDir)
		{
			differentDirections++;
		}
	}

	return (backwardMoves > HISTORY_SIZE / 2 || differentDirections > 6);
}

float calculateDirectionScore(Direction dir, SensorData sensors,
                              NavigationState *state)
{
	float score = 0.0;

	switch (dir)
	{
	case NORTH:
		score = sensors.front * 2.5;
		if (state->preferredDirection == NORTH)
			score *= 1.5;
		break;
	case SOUTH:
		score = sensors.back * 0.5;
		break;
	case EAST:
		score = sensors.right * 1.2;
		break;
	case WEST:
		score = sensors.left * 1.2;
		break;
	case NORTH_EAST:
		score = (sensors.front * 1.8 + sensors.right * 1.5) / 2;
		break;
	case NORTH_WEST:
		score = (sensors.front * 1.8 + sensors.left * 1.5) / 2;
		break;
	case SOUTH_EAST:
		score = (sensors.back * 0.6 + sensors.right * 0.8) / 2;
		break;
	case SOUTH_WEST:
		score = (sensors.back * 0.6 + sensors.left * 0.8) / 2;
		break;
	default:
		score = 0;
	}

	for (int i = 0; i < HISTORY_SIZE && i < state->historyIndex; i++)
	{
		if (state->lastDirections[i] == dir)
		{
			score *= 0.85;
		}
	}

	score *= state->momentum;

	return score;
}

bool isDirectionSafe(Direction dir, SensorData sensors)
{
	switch (dir)
	{
	case NORTH:
		return sensors.front > OBSTACLE_THRESHOLD;
	case SOUTH:
		return sensors.back > OBSTACLE_THRESHOLD;
	case EAST:
		return sensors.right > OBSTACLE_THRESHOLD;
	case WEST:
		return sensors.left > OBSTACLE_THRESHOLD;
	case NORTH_EAST:
		return (sensors.front > OBSTACLE_THRESHOLD &&
		        sensors.right > OBSTACLE_THRESHOLD);
	case NORTH_WEST:
		return (sensors.front > OBSTACLE_THRESHOLD &&
		        sensors.left > OBSTACLE_THRESHOLD);
	case SOUTH_EAST:
		return (sensors.back > OBSTACLE_THRESHOLD &&
		        sensors.right > OBSTACLE_THRESHOLD);
	case SOUTH_WEST:
		return (sensors.back > OBSTACLE_THRESHOLD &&
		        sensors.left > OBSTACLE_THRESHOLD);
	default:
		return false;
	}
}

Direction escapeManeuver(SensorData sensors)
{
	printf("!!! STUCK DETECTED - Executing escape maneuver\n");

	int maxDist = sensors.front;
	Direction escapeDir = NORTH;

	if (sensors.back > maxDist)
	{
		maxDist = sensors.back;
		escapeDir = SOUTH;
	}
	if (sensors.left > maxDist)
	{
		maxDist = sensors.left;
		escapeDir = WEST;
	}
	if (sensors.right > maxDist)
	{
		maxDist = sensors.right;
		escapeDir = EAST;
	}

	return escapeDir;
}

Direction decideDirectionAdvanced(SensorData sensors, NavigationState *state)
{
	if (sensors.front < CRITICAL_DISTANCE && sensors.back < CRITICAL_DISTANCE &&
	    sensors.left < CRITICAL_DISTANCE && sensors.right < CRITICAL_DISTANCE)
	{
		return STOP;
	}

	if (isStuck(state))
	{
		state->stuckCounter++;
		if (state->stuckCounter > 3)
		{
			state->stuckCounter = 0;
			state->historyIndex = 0;
			return escapeManeuver(sensors);
		}
	}

	Direction allDirections[] = {NORTH, NORTH_EAST, NORTH_WEST, EAST,
	                             WEST,  SOUTH_EAST, SOUTH_WEST, SOUTH};
	float bestScore = -1.0;
	Direction bestDirection = NORTH;

	for (int i = 0; i < 8; i++)
	{
		Direction dir = allDirections[i];
		if (isDirectionSafe(dir, sensors))
		{
			float score = calculateDirectionScore(dir, sensors, state);
			if (score > bestScore)
			{
				bestScore = score;
				bestDirection = dir;
			}
		}
	}

	if (bestScore < 0)
	{
		for (int i = 0; i < 8; i++)
		{
			Direction dir = allDirections[i];
			float score = calculateDirectionScore(dir, sensors, state);
			if (score > bestScore)
			{
				bestScore = score;
				bestDirection = dir;
			}
		}
	}

	state->lastDirections[state->historyIndex % HISTORY_SIZE] = bestDirection;
	state->historyIndex++;

	if (bestDirection == NORTH)
	{
		state->momentum = fmin(state->momentum * 1.1, 1.5);
	}
	else
	{
		state->momentum = fmax(state->momentum * 0.9, 0.5);
	}

	return bestDirection;
}

void executeMovement(Direction dir)
{
	switch (dir)
	{
	case NORTH:
		moveNorth();
		break;
	case SOUTH:
		moveSouth();
		break;
	case EAST:
		moveEast();
		break;
	case WEST:
		moveWest();
		break;
	case NORTH_EAST:
		moveNorthEast();
		break;
	case NORTH_WEST:
		moveNorthWest();
		break;
	case SOUTH_EAST:
		moveSouthEast();
		break;
	case SOUTH_WEST:
		moveSouthWest();
		break;
	case STOP:
		stopMotors();
		break;
	}
}

void printDiagnostics(SensorData sensors, Direction dir, NavigationState *state)
{
	printf("\n========================================\n");
	printf("SENSOR READINGS:\n");
	printf("  Front: %3d cm | Back:  %3d cm\n", sensors.front, sensors.back);
	printf("  Left:  %3d cm | Right: %3d cm\n", sensors.left, sensors.right);
	printf("NAVIGATION STATE:\n");
	printf("  Momentum: %.2f | Stuck Counter: %d\n", state->momentum,
	       state->stuckCounter);
	printf("DECISION: ");
}

int main()
{
	SensorData sensors;
	Direction currentDirection;

	srand(time(NULL));

	printf("===========================================\n");
	printf("ADVANCED OBSTACLE AVOIDANCE SYSTEM STARTED\n");
	printf("===========================================\n\n");

	while (1)
	{
		sensors = readAllSensors();

		currentDirection = decideDirectionAdvanced(sensors, &navState);

		printDiagnostics(sensors, currentDirection, &navState);
		executeMovement(currentDirection);

		usleep(DELAY_MS * 1000);
	}

	return 0;
}
