#ifndef CONTROLLER
#define CONTROLLER

#include <vector>

#include "directions.h"

using std::vector;
using std::pair;

class Robot;

class Controller {
	const Robot* rob;
	pair<int, int> charger_dist;
	vector<Direction> path_to_charger;
	const float starting_battery;
	bool charging;
	bool pathing_to_charger;

	Direction naive_algorithm();
	Direction dfs();
public:
	Controller(const Robot*);
	Direction get_next_step();
};

#endif