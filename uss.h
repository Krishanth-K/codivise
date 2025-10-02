#ifndef USS_H
#define USS_H

#include <stdbool.h>
#include <stdio.h>

#define OBSTACLE_THRESHOLD 25
#define SAFE_DISTANCE 60
#define CRITICAL_DISTANCE 15
#define HISTORY_SIZE 10
#define STUCK_THRESHOLD 5

typedef enum
{
	DIR_NORTH = 0,
	DIR_SOUTH,
	DIR_EAST,
	DIR_WEST,
	DIR_NORTH_EAST,
	DIR_NORTH_WEST,
	DIR_SOUTH_EAST,
	DIR_SOUTH_WEST,
	DIR_STOP
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

static inline float fmin_custom(float a, float b) { return (a < b) ? a : b; }

static inline float fmax_custom(float a, float b) { return (a > b) ? a : b; }

static inline void navigation_init(NavigationState *state)
{
	state->historyIndex = 0;
	state->stuckCounter = 0;
	state->preferredDirection = DIR_NORTH;
	state->momentum = 1.0;

	for (int i = 0; i < HISTORY_SIZE; i++)
	{
		state->lastDirections[i] = DIR_NORTH;
	}
}

static inline bool is_stuck(NavigationState *state)
{
	if (state->historyIndex < HISTORY_SIZE)
		return false;

	int backwardMoves = 0;
	int differentDirections = 0;
	Direction firstDir = state->lastDirections[0];

	for (int i = 0; i < HISTORY_SIZE; i++)
	{
		if (state->lastDirections[i] == DIR_SOUTH ||
		    state->lastDirections[i] == DIR_SOUTH_EAST ||
		    state->lastDirections[i] == DIR_SOUTH_WEST)
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

static inline float calculate_direction_score(Direction dir, SensorData sensors,
                                              NavigationState *state)
{
	float score = 0.0;

	switch (dir)
	{
	case DIR_NORTH:
		score = sensors.front * 2.5;
		if (state->preferredDirection == DIR_NORTH)
			score *= 1.5;
		break;
	case DIR_SOUTH:
		score = sensors.back * 0.5;
		break;
	case DIR_EAST:
		score = sensors.right * 1.2;
		break;
	case DIR_WEST:
		score = sensors.left * 1.2;
		break;
	case DIR_NORTH_EAST:
		score = (sensors.front * 1.8 + sensors.right * 1.5) / 2;
		break;
	case DIR_NORTH_WEST:
		score = (sensors.front * 1.8 + sensors.left * 1.5) / 2;
		break;
	case DIR_SOUTH_EAST:
		score = (sensors.back * 0.6 + sensors.right * 0.8) / 2;
		break;
	case DIR_SOUTH_WEST:
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

static inline bool is_direction_safe(Direction dir, SensorData sensors)
{
	switch (dir)
	{
	case DIR_NORTH:
		return sensors.front > OBSTACLE_THRESHOLD;
	case DIR_SOUTH:
		return sensors.back > OBSTACLE_THRESHOLD;
	case DIR_EAST:
		return sensors.right > OBSTACLE_THRESHOLD;
	case DIR_WEST:
		return sensors.left > OBSTACLE_THRESHOLD;
	case DIR_NORTH_EAST:
		return (sensors.front > OBSTACLE_THRESHOLD &&
		        sensors.right > OBSTACLE_THRESHOLD);
	case DIR_NORTH_WEST:
		return (sensors.front > OBSTACLE_THRESHOLD &&
		        sensors.left > OBSTACLE_THRESHOLD);
	case DIR_SOUTH_EAST:
		return (sensors.back > OBSTACLE_THRESHOLD &&
		        sensors.right > OBSTACLE_THRESHOLD);
	case DIR_SOUTH_WEST:
		return (sensors.back > OBSTACLE_THRESHOLD &&
		        sensors.left > OBSTACLE_THRESHOLD);
	default:
		return false;
	}
}

static inline Direction escape_maneuver(SensorData sensors)
{
	int maxDist = sensors.front;
	Direction escapeDir = DIR_NORTH;

	if (sensors.back > maxDist)
	{
		maxDist = sensors.back;
		escapeDir = DIR_SOUTH;
	}
	if (sensors.left > maxDist)
	{
		maxDist = sensors.left;
		escapeDir = DIR_WEST;
	}
	if (sensors.right > maxDist)
	{
		maxDist = sensors.right;
		escapeDir = DIR_EAST;
	}

	return escapeDir;
}

static inline Direction navigation_decide(SensorData sensors,
                                          NavigationState *state)
{
	if (sensors.front < CRITICAL_DISTANCE && sensors.back < CRITICAL_DISTANCE &&
	    sensors.left < CRITICAL_DISTANCE && sensors.right < CRITICAL_DISTANCE)
	{
		return DIR_STOP;
	}

	if (is_stuck(state))
	{
		state->stuckCounter++;
		if (state->stuckCounter > 3)
		{
			state->stuckCounter = 0;
			state->historyIndex = 0;
			return escape_maneuver(sensors);
		}
	}

	Direction allDirections[] = {DIR_NORTH,      DIR_NORTH_EAST, DIR_NORTH_WEST,
	                             DIR_EAST,       DIR_WEST,       DIR_SOUTH_EAST,
	                             DIR_SOUTH_WEST, DIR_SOUTH};
	float bestScore = -1.0;
	Direction bestDirection = DIR_NORTH;

	for (int i = 0; i < 8; i++)
	{
		Direction dir = allDirections[i];
		if (is_direction_safe(dir, sensors))
		{
			float score = calculate_direction_score(dir, sensors, state);
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
			float score = calculate_direction_score(dir, sensors, state);
			if (score > bestScore)
			{
				bestScore = score;
				bestDirection = dir;
			}
		}
	}

	state->lastDirections[state->historyIndex % HISTORY_SIZE] = bestDirection;
	state->historyIndex++;

	if (bestDirection == DIR_NORTH)
	{
		state->momentum = fmin_custom(state->momentum * 1.1, 1.5);
	}
	else
	{
		state->momentum = fmax_custom(state->momentum * 0.9, 0.5);
	}

	return bestDirection;
}

static inline const char *direction_to_string(Direction dir)
{
	switch (dir)
	{
	case DIR_NORTH:
		return "NORTH";
	case DIR_SOUTH:
		return "SOUTH";
	case DIR_EAST:
		return "EAST";
	case DIR_WEST:
		return "WEST";
	case DIR_NORTH_EAST:
		return "NORTH_EAST";
	case DIR_NORTH_WEST:
		return "NORTH_WEST";
	case DIR_SOUTH_EAST:
		return "SOUTH_EAST";
	case DIR_SOUTH_WEST:
		return "SOUTH_WEST";
	case DIR_STOP:
		return "STOP";
	default:
		return "UNKNOWN";
	}
}

static inline bool is_critical_situation(SensorData sensors)
{
	return (
	    sensors.front < CRITICAL_DISTANCE || sensors.back < CRITICAL_DISTANCE ||
	    sensors.left < CRITICAL_DISTANCE || sensors.right < CRITICAL_DISTANCE);
}

#endif
