#ifndef ROBOT
#define ROBOT

#include <vector>
#include <fstream>

#include "directions.h"

using house = std::vector<std::vector<char>>;

class Controller;

class Robot {
	// Robot Data
	float current_battery;
	float max_battery;
	int current_steps;
	int max_steps;
	Controller* controller;

	// House Metadata
	house& model;
	int remaining_dirt;
	int current_row;
	int current_col;
	int step_time;

	// Helper Methods
	int calculate_dirt() const;
public:
	Robot(house& model, float max_battery, int max_steps, int starting_row, int starting_col, int step_time);
	~Robot();
	int clean_house(std::ofstream&);
	int get_dirt_underneath() const;
	bool is_wall(Direction) const;
	float remaining_battery() const;
};

#endif