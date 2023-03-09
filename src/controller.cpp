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
	std::shared_ptr<Node> parent; /* This pointer is necessary for backtracking */
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

template <typename T>
void printVec(const vector<T>& v, const std::string& label = "") noexcept {
	cout << label << "[ ";
	for (int i = 0; i < v.size(); i++) {
		cout << v[i];
		if (i != v.size() - 1) cout << ", ";
	}
	cout << " ]" << endl;
}

Direction Controller::dfs() {
	/* Could maybe move these static vars into the Controller constructor, but tbf if we're separating naive and dfs, it's probably more organized to do it like this.*/
	static Node start(Position{ 0, 0 });
	static vector<Direction> path; /* A path back to the charger */
	static unordered_set<Position, PositionHasher> mapped{ Position{0,0} }; /* Vector of nodes the robot knows about */
	static unordered_set<Position, PositionHasher> visited{ Position{0,0} }; /* Vector of nodes the robot has visited */
	static std::shared_ptr<Node> c = std::make_shared<Node>(start); /* Current node */

	cout << "============================================\n"; printVec(path, "Path Stack: ");

	/* Need to implement return to charger algo 
	 * Should be similar to the naive algo, but we need to make a copy of the current path.
	 * This is so that we can restore the robot to the place we were at before we started returning.
	 * To restore, basically just flip the path vector's directions, probably with a .map or something
	 * static vector<Direction> returnPath; <- maybe use something like this to store the duplicate path. 
	 * You wrote the naive algorithm return thing, so uhhhh yeah. yoroshiku */
#if 0
	if (pathing_to_charger || rob->remaining_battery() - 2 < static_cast<int>(path.size())) {
		pathing_to_charger = true;
		/* Check if we have arrived at the charger */

		/* NOTE: I'm pretty sure with the way our path stack works now, it will never reach the charger early. 
		 * So we could probably delete this check. Idk test later.                   */
		if (charger_dist.first == 0 && charger_dist.second == 0) {                 /**/
			path_to_charger.clear();  /* Clear list in case we arrived "early" */  /**/
			charging = true;		  /* Set charging to true */                   /**/
			pathing_to_charger = false;                                            /**/
			goto br;                                                               /**/
		}																		   /**/
		/*****************************************************************************/

		/* If we have not yet arrived, backtrack to charger */
		Direction popped = path.back();
		path.pop_back();

		/* And because we'll never arrive earlier, this whole keeping track of the charger dist can also be removed. 
		 * Also worth noting we are now using the coordinate system with Nodes and Position structs, meaning: 
		 * We can just find the current position by querying the Position coords of the node we're on. 
		 * And the charger is simply Position (0,0) if we need it somewhere else.   */
		switch (popped) {                                                         /**/
		case Direction::EAST: ++charger_dist.first; break;					      /**/
		case Direction::WEST: --charger_dist.first; break;					      /**/
		case Direction::SOUTH: ++charger_dist.second; break;					  /**/
		case Direction::NORTH: --charger_dist.second; break;					  /**/
		}																		  /**/
		/****************************************************************************/

		return popped;
	}
	/* Charge until we hit our starting battery */
	if (charging) {
	br:
		if (rob->remaining_battery() < starting_battery || rob->remaining_battery() < 2)
			return Direction::STAY;
		charging = false;
	}
#endif


	if (rob->get_dirt_underneath() > 0) return Direction::STAY; /* If there's dirt stay still */

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

	printVec(choice, "Choice: ");

	/* pchoose gets a priority among the choice for whichever node is not already visited. */
	const auto pchoose = std::find_if(choice.begin(), choice.end(), [](Direction d) { return visited.find(c->getCoords(d)) == visited.end(); });
	if (choice.size() == 0 || pchoose == choice.end()) { /* No choice, or if all nodes are visited */
		if (path.empty()) return Direction::STAY; /* Either we're in an enclosed area or we are done. */
		Direction dir = path.back(); path.pop_back(); /* or we've fully explored the branch, so start consuming the path stack. */
		c = c->parent;
		return dir;
	}
	const auto ind = pchoose - choice.begin();
	cout << "Chosen index: " << ind << "(" << choice[ind] << ")" << endl;
	visited.insert(c->getCoords(choice[ind])); /* Mark the node as visited */
	c = c->neighbours[ind]; /* Sets the current node to the node we're visiting */
	path.push_back(opposite(choice[ind])); /* Return path */

	return choice[ind];
}
