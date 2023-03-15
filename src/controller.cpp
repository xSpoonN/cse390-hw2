#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <iterator>
#include <string>
#include <algorithm>
#include <iostream>

#include "controller.h"
#include "robot.h"
#include "directions.h"

using std::vector;
using std::pair;
using std::unordered_set;
using std::unordered_map;
using std::cout;
using std::endl;

Controller::Controller(const Robot* rob) : rob(rob), charger_dist(pair<int, int>(0, 0)), path_to_charger({})
	, starting_battery(rob->remaining_battery()), charging(false), pathing_to_charger(false) {}

Direction Controller::get_next_step() {	return dfs(); }

struct Position {
	int x, y;
	auto operator<=>(const Position&) const = default; /* C++20 spaceship operator pogslide */
	friend std::ostream& operator<<(std::ostream& os, const Position& p) {
		os << "(" << p.x << ", " << p.y << ")";
		return os;
	}
};
class PositionHasher { /* Hash function for Position is necessary since unordered set uses a hash table internally */
public:
	size_t operator()(const Position& p) const { return (p.x + p.y) * (p.x + p.y + 1) / 2 + p.x; } /* Cantor's enumeration of pairs */
};

class Node {
public:
	Position coords;
	vector<std::shared_ptr<Node>> nb; /* Neighbours of this node */
	std::shared_ptr<Node> parent; /* This pointer is necessary for backtracking */
	Node(const Position coords, std::shared_ptr<Node> parent = nullptr) : coords(coords), parent(parent) {}

	bool operator==(const Node& r) const { return coords.x == r.coords.x && coords.y == r.coords.y; }
	Position getCoords(Direction d) const {
		switch (d) {
		case Direction::NORTH: return Position{ coords.x, coords.y - 1 };
		case Direction::EAST: return Position{ coords.x + 1, coords.y };
		case Direction::SOUTH: return Position{ coords.x, coords.y + 1 };
		case Direction::WEST: return Position{ coords.x - 1, coords.y };
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

Position getPos(const Position cur, const Direction dir) {
	switch (dir) {
	case Direction::NORTH: return Position{ cur.x, cur.y - 1 }; break;
	case Direction::EAST: return Position{ cur.x + 1, cur.y }; break;
	case Direction::SOUTH: return Position{ cur.x, cur.y + 1 }; break;
	case Direction::WEST: return Position{ cur.x - 1, cur.y }; break;
	default: return Position{ 0,0 };
	}
}


Direction Controller::dfs() {
	/* Could maybe move these static vars into the Controller constructor, but lowkey kinda like this better. Might be some redundancy with existing Controller vars tho */
	static Node start(Position{ 0, 0 });
	static vector<Direction> path; /* A path back to the charger */
	static unordered_set<Position, PositionHasher> mapped{ Position{0,0} }; /* Vector of nodes the robot knows about */
	static unordered_set<Position, PositionHasher> visited{ Position{0,0} }; /* Vector of nodes the robot has visited */
	static unordered_map<Position, vector<Direction>, PositionHasher> returnPath{ {{0,0},  {}} }; /* Map of the most efficient return path from each node. */
	static vector<Direction> returnQ; /* A queue of directions to return to the charger */
	static vector<Direction> resumePath; /* A path back to the previous position */
	static Position curPos{ 0,0 }; /* This is redundant most of the time but helps a LOT with the return algorithm */
	static std::shared_ptr<Node> c = std::make_shared<Node>(start); /* Current node */

	cout << "============================================\n"; printVec(path, "Path Stack: ");

	vector<Direction>& retPath = returnPath[c->coords];
	if (curPos != Position{ 0,0 } && rob->remaining_battery() - 2 < static_cast<int>(retPath.size())) {
		if (returnQ.size() == 0) returnQ = vector<Direction>(retPath);
		printVec(returnQ, "Returning to charger. Path: ");
		Direction d = returnQ.back();
		resumePath.push_back(opposite(d)); returnQ.pop_back();
		curPos = getPos(curPos, d);
		return d;
	}
	if (curPos == Position{ 0,0 } && (rob->remaining_battery() < starting_battery || rob->remaining_battery() < 2)) return Direction::STAY; /* Charge if we need to. */
	if (resumePath.size() > 0) {
		printVec(resumePath, "Resume Path: ");
		Direction d = resumePath.back(); resumePath.pop_back();
		curPos = getPos(curPos, d);
		return d;
	}

	if (rob->get_dirt_underneath() > 0) return Direction::STAY; /* If there's dirt stay still */

	vector<Direction> choice; /* Populate the choice vector */
	vector<Direction>& curPath = returnPath[c->coords];
	for (const auto& dir : { Direction::NORTH, Direction::EAST, Direction::SOUTH, Direction::WEST }) {
		if (rob->is_wall(dir)) continue; /* If there's a wall, don't add it to the choice vector */
		Position p = c->getCoords(dir);
		c->nb.push_back(std::make_shared<Node>(p, c));
		choice.push_back(dir);
		mapped.insert(p);

		vector<Direction>& dirPath = returnPath[p]; /* The following code keeps track of the fastest way back to the charger. */
		if (dirPath.empty() || dirPath.size() > curPath.size() + 1) { /* If the path to the node is longer than what can be reached through the current node */
			dirPath = vector<Direction>(curPath); // Save the current path
			dirPath.push_back(opposite(dir)); // Add the opposite direction to the path
		} else if (curPath.empty() || curPath.size() > dirPath.size() + 1) { /* If the current path is empty or is longer than what can be reached through the other node */
			curPath = vector<Direction>(dirPath); // Save the path to the node
			curPath.push_back(dir); // Add the direction to the path 
		}
	}

	printVec(choice, "Choice: ");
	printVec(curPath, "Return Path: ");

	/* pchoose gets a priority among the choice for whichever node is not already visited. */
	const auto pchoose = std::find_if(choice.begin(), choice.end(), [](Direction d) { return visited.find(c->getCoords(d)) == visited.end(); });
	if (choice.size() == 0 || pchoose == choice.end()) { /* No choice, or if all nodes are visited */
		if (path.empty()) return Direction::STAY; /* Either we're in an enclosed area or we are done. */
		Direction dir = path.back(); path.pop_back(); /* or we've fully explored the branch, so start consuming the path stack. */
		c = c->parent; curPos = c->coords;
		return dir;
	}
	const auto ind = pchoose - choice.begin();
	visited.insert(c->getCoords(choice[ind])); /* Mark the node as visited */
	c = c->nb[ind]; curPos = c->coords; /* Sets the current node to the node we're visiting */
	path.push_back(opposite(choice[ind])); /* Return path */

	return choice[ind];
}
