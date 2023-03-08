#include <vector>
#include <unordered_set>
#include <stack>
#include <iterator>
#include <random>
#include <string>
#include <algorithm>
#include <iostream>
#include <ctime>

#include "controller.h"
#include "robot.h"
#include "directions.h"

using std::vector;
using std::stack;
using std::pair;
using std::unordered_set;
using std::cout;
using std::endl;

Controller::Controller(const Robot* rob) : rob(rob), charger_dist(pair<int, int>(0, 0)), path_to_charger({})
	, starting_battery(rob->remaining_battery()), charging(false), pathing_to_charger(false) {
	std::srand(static_cast<unsigned int>(std::time(nullptr)));
}

Direction Controller::get_next_step() {	
	return dfs();

	/* Check if we want to go back to the charger (low battery) */
	if (pathing_to_charger || rob->remaining_battery() - 2 < static_cast<int>(path_to_charger.size())) {
		pathing_to_charger = true;
		/* Check if we have arrived at the charger */
		if (charger_dist.first == 0 && charger_dist.second == 0) {
			path_to_charger.clear();  /* Clear list in case we arrived "early" */
			charging = true;
			pathing_to_charger = false;
			goto br;
		}
		/* If we have not yet arrived, backtrack to charger */
		Direction popped = path_to_charger.back();
		path_to_charger.pop_back();
		switch (popped) {
			case Direction::EAST: ++charger_dist.first; break;
			case Direction::WEST: --charger_dist.first; break;
			case Direction::SOUTH: ++charger_dist.second; break;
			case Direction::NORTH: --charger_dist.second; break;
		}
		return popped;
	}
	/* Charge until we hit our starting battery */
	if (charging) {
		br:
		if (rob->remaining_battery() < starting_battery || rob->remaining_battery() < 2)
			return Direction::STAY;
		charging = false;
	}

	/* Proceed to pick a direction with the given algorithm */
	Direction dir = naive_algorithm();
	switch (dir) { /* Push the reverse into path_to_charger vec. */
	case Direction::NORTH:
		path_to_charger.push_back(Direction::SOUTH);
		--charger_dist.second;
		if (charger_dist.first == 0 && charger_dist.second == 0) path_to_charger.clear();
		break;
	case Direction::EAST:
		path_to_charger.push_back(Direction::WEST);
		++charger_dist.first;
		if (charger_dist.first == 0 && charger_dist.second == 0) path_to_charger.clear();
		break;
	case Direction::SOUTH:
		path_to_charger.push_back(Direction::NORTH);
		++charger_dist.second;
		if (charger_dist.first == 0 && charger_dist.second == 0) path_to_charger.clear();
		break;
	case Direction::WEST:
		path_to_charger.push_back(Direction::EAST);
		--charger_dist.first;
		if (charger_dist.first == 0 && charger_dist.second == 0) path_to_charger.clear();
		break;
	}
	return dir;
}

Direction Controller::naive_algorithm() {
	vector<Direction> choice;
	if (rob->get_dirt_underneath() > 0) return Direction::STAY; /* If there's dirt stay still */
	if (!rob->is_wall(Direction::NORTH)) {
		choice.push_back(Direction::NORTH);
	}
	if (!rob->is_wall(Direction::EAST)) {
		choice.push_back(Direction::EAST);
	}
	if (!rob->is_wall(Direction::SOUTH)) {
		choice.push_back(Direction::SOUTH);
	}
	if (!rob->is_wall(Direction::WEST)) {
		choice.push_back(Direction::WEST);
	}
	if (choice.empty()) return Direction::STAY; /* Robert is walled in. xdd */
	return choice[std::rand() % choice.size()];
}

struct Position { 
	int x, y;
	auto operator<=>(const Position&) const = default; /* C++20 spaceship operator pogslide */
	friend std::ostream& operator<<(std::ostream& os, const Position& p) {
		os << "(" << p.x << ", " << p.y << ")";
		return os;
	}
};
class PositionHasher { /* Hash function for Position is necessary since unordered set uses a hash table internally */
public:                /* And it needs to know how to store struct Position. */
	size_t operator()(const Position& p) const { return (p.x + p.y) * (p.x + p.y + 1) / 2 + p.x; } /* Cantor's enumeration of pairs */
};

class Node {
public:
	Position coords;
	vector<std::shared_ptr<Node>> neighbours;
	std::shared_ptr<Node> parent; /* This is necessary for backtracking */
	Node(const Position coords, std::shared_ptr<Node> parent = nullptr) : coords(coords), parent(parent) {}

	bool operator==(const Node& r) const { return coords.x == r.coords.x && coords.y == r.coords.y; }

	Position nCoords() const { return Position{ coords.x, coords.y - 1 }; }
	Position eCoords() const { return Position{ coords.x + 1, coords.y }; }
	Position sCoords() const { return Position{ coords.x, coords.y + 1 }; }
	Position wCoords() const { return Position{ coords.x - 1, coords.y }; }
	Position getCoords(Direction d) const {
		switch (d) {
		case Direction::NORTH: return nCoords();
		case Direction::EAST: return eCoords();
		case Direction::SOUTH: return sCoords();
		case Direction::WEST: return wCoords();
		}
		return Position{ 0, 0 };
	}
};

Direction Controller::dfs() {
	static Node start(Position{ 0, 0 });
	static vector<Direction> path; /* A path back to the charger */
	static unordered_set<Position, PositionHasher> mapped{ Position{0,0} }; /* Vector of nodes the robot knows about */
	static unordered_set<Position, PositionHasher> visited{ Position{0,0} }; /* Vector of nodes the robot has visited */
	static std::shared_ptr<Node> c = std::make_shared<Node>(start); /* Current node */

	cout << "============================================\nPath Stack: [ ";
	for (int i = 0; i < path.size(); i++) {
		cout << path[i] << " ";
	}
	cout << "]" << endl;

	/* Algorithm overrides go here, such as when the robot runs out of battery or if there's dirt */
	if (rob->get_dirt_underneath() > 0) return Direction::STAY; /* If there's dirt stay still */
	/* Need to implement return to charger algo */

	vector<Direction> choice; /* Populate the choice vector */
	if (!rob->is_wall(Direction::NORTH)) {
		c->neighbours.push_back(std::make_shared<Node>(c->nCoords(), c));
		choice.push_back(Direction::NORTH);
		mapped.insert(c->nCoords());
	}
	if (!rob->is_wall(Direction::EAST)) {
		c->neighbours.push_back(std::make_shared<Node>(c->eCoords(), c));
		choice.push_back(Direction::EAST);
		mapped.insert(c->eCoords());
	}
	if (!rob->is_wall(Direction::SOUTH)) {
		c->neighbours.push_back(std::make_shared<Node>(c->sCoords(), c));
		choice.push_back(Direction::SOUTH);
		mapped.insert(c->sCoords());
	}
	if (!rob->is_wall(Direction::WEST)) {
		c->neighbours.push_back(std::make_shared<Node>(c->wCoords(), c));
		choice.push_back(Direction::WEST);
		mapped.insert(c->wCoords());
	}

	cout << "Choice: [ ";
	for (int i = 0; i < choice.size(); i++) {
		cout << choice[i] << " ";
	}
	cout << "]" << endl;

	if (choice.size() == 0) {
		ovrde: 
		if (path.empty()) return Direction::STAY; /* Either we're in an enclosed area, in which we just STAY cause mikeyDoofus */
		Direction dir = path.back(); /* or we've fully explored the branch, so start consuming the path stack. */
		path.pop_back();
		c = c->parent;
		return dir;
	} else {
		/* pchoose gets a priority among the choice for whichever node is not already visited. */
		const auto pchoose = std::find_if(choice.begin(), choice.end(), [](Direction d) {return visited.find(c->getCoords(d)) == visited.end(); });
		if (pchoose == choice.end()) goto ovrde; /* If all nodes are visited, go to override. */
		const auto ind = pchoose - choice.begin();
		cout << "Chosen index: " << ind << "(" << choice[ind] << ")" << endl;
		visited.insert(c->getCoords(choice[ind])); /* Mark the node as visited */
		c = c->neighbours[ind]; /* Sets the current node to the node we're visiting */
		path.push_back(opposite(choice[ind])); /* Return path */
		return choice[ind];
	}
}