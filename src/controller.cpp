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
	/*bool operator==(const Position& r) const { return x == r.x && y == r.y; }*/
	auto operator<=>(const Position&) const = default; /* C++20 (VS 2019*/
	friend std::ostream& operator<<(std::ostream& os, const Position& p) {
		os << "(" << p.x << ", " << p.y << ")";
		return os;
	}
};
class Hasher {
public:
	size_t operator()(const Position& p) const {
		return (p.x + p.y) * (p.x + p.y + 1) / 2 + p.x;
		/*return std::hash<int>()(p.x) ^ std::hash<int>()(p.y);*/
	}
};

class Node {
public:
	Position coords;
	vector<Node*> neighbours;
	Direction pre;
	Node* parent;
	Node(const Position coords, Node* parent = nullptr, Direction pre = Direction::NONE) : coords(coords), parent(parent), pre(pre) {}

	bool operator==(const Node& r) const {
		return coords.x == r.coords.x && coords.y == r.coords.y;
	}

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
	static vector<Direction> path/*ToNextBranch*/;
	static unordered_set<Position, Hasher> mapped{ Position{0,0} };
	static unordered_set<Position, Hasher> visited{ Position{0,0} };
	static Node* c = &start;
	/*if (mapped.empty()) mapped.insert(Position{ 0, 0 });*/
	/*******************************************************************************/
	cout << "========================================================\nPath: ";
	for (int i = 0; i < path.size(); i++) {
		cout << (int)path[i] << " ";
	}
	cout << endl;

	/* Algorithm overrides go here, such as when the robot runs out of battery or if there's dirt */
	if (rob->get_dirt_underneath() > 0) return Direction::STAY;

	vector<Direction> choice;
	/*cout << c->nCoords() << " " << c->eCoords() << " " << c->sCoords() << " " << c->wCoords() << endl;*/

	/*Position temp = c->nCoords();*/
	if (!rob->is_wall(Direction::NORTH)/* && mapped.find(temp) == mapped.end()*/) {
		/* I'm unsure if keeping track of the neighbours of each node is even necessary but whatever fix later */
		c->neighbours.push_back(new Node(c->nCoords(), c));
		choice.push_back(Direction::NORTH);
		mapped.insert(c->nCoords());
	}

	/*temp = c->eCoords();*/
	if (!rob->is_wall(Direction::EAST)/* && mapped.find(temp) == mapped.end()*/) {
		c->neighbours.push_back(new Node(c->eCoords(), c));
		choice.push_back(Direction::EAST);
		mapped.insert(c->eCoords());
	}

	/*temp = c->sCoords();*/
	if (!rob->is_wall(Direction::SOUTH)/* && mapped.find(temp) == mapped.end()*/) {
		c->neighbours.push_back(new Node(c->sCoords(), c));
		choice.push_back(Direction::SOUTH);
		mapped.insert(c->sCoords());
	}

	/*temp = c->wCoords();*/
	if (!rob->is_wall(Direction::WEST)/* && mapped.find(temp) == mapped.end()*/) {
		c->neighbours.push_back(new Node(c->wCoords(), c));
		choice.push_back(Direction::WEST);
		mapped.insert(c->wCoords());
	}
	/* Might need to assign priority to nodes that haven't been mapped yet */
	cout << "Choice: ";
	for (int i = 0; i < choice.size(); i++) {
		cout << (int)choice[i] << " ";
	}
	cout << endl;
	if (choice.size() == 0) {
		/* Either we're in an enclosed area, or we have fully explored this branch
		 * If pathToNextBranch is empty, we return stay, since we're blocked in
		 * Otherwise, consume pathToNextBranch until we get to the next node
		 * In standard dfs we just go straight to the next node, but this is a physical roomba
		 * So we need to keep track of the route as we're branching */
		ovrde: 
		if (path.empty()) return Direction::STAY;
		Direction dir = path.back();
		path.pop_back();
		c = c->parent;
		/*Position crds = c->getCoords(dir);
		auto afunc = [crds](Node* n) { return n->coords == crds; };
		auto index = std::find_if(c->neighbours.begin(), c->neighbours.end(), afunc);
		c = c->neighbours[index - c->neighbours.begin()];*/
		return dir;
	}
	else {
		auto pchoose = std::find_if(choice.begin(), choice.end(), [](Direction d) {return visited.find(c->getCoords(d)) == visited.end(); });
		if (pchoose == choice.end()) {
			cout << "No visited node is adjacent, going to ovrde" << endl;
			goto ovrde;
		}
		auto ind = pchoose - choice.begin();
		Direction temp = choice[ind];
		cout << "Chosen index: " << ind << "(" << temp << ")" << endl;
		visited.insert(c->getCoords(choice[ind]));
		/* Adds positions into the path stack for when we backtrack and find new exploration paths */
		//for (int i = 0; i < choice.size(); i++) {
		//	if (i == ind || visited.find(c->getCoords(choice[i])) != visited.end()) continue;
		//	switch (choice[i]) {
		//	case Direction::NORTH: cout << "Unexplored North branch, adding to path";
		//		path.push_back(Direction::SOUTH); path.push_back(Direction::NORTH); break;
		//	case Direction::EAST: cout << "Unexplored East branch, adding to path";
		//		path.push_back(Direction::WEST); path.push_back(Direction::EAST); break;
		//	case Direction::SOUTH: cout << "Unexplored South branch, adding to path";
		//		path.push_back(Direction::NORTH); path.push_back(Direction::SOUTH); break;
		//	case Direction::WEST: cout << "Unexplored West branch, adding to path";
		//		path.push_back(Direction::EAST); path.push_back(Direction::WEST); break;
		//	}
		//}
		c = c->neighbours[ind]; /* Sets the current node to the node we're visiting */
		path.push_back(opposite(choice[ind])); /* Return path */
		
		return choice[ind];
		/* Random thoughts: 
		 * Maybe need to keep track of a "path to next node" for each node?
		 * From the example, after exploring the last 0 of the first branch, it needs to get to the 9 somehow.
		 * Either when we get to that 2 junction, we push a down into that queue, and then as we traverse 1430,
		 * we push west, north, west, north in that order
		 * but the problem might come when there's another junction in the way.
		 * like for example what happens to that first 0 junction
		 * or actually i suppose as your making your way back using the path, check for any neighbours but that might not work
		 * starting from the beginning we have a junction, and east has priority so down + west will be pushed onto that path
		 * -- --
		 * shit wait what happens if theres 3 options, like west south east
		 * no wait you can just do like push WNSW. it might be kinda annoying with different cases tho but idk
		 * -- --
		 * so now path is NSWNSW when we get to 4 so on the way back lets say from 4 it goes W-0, S-0, N-0, W-+, S-3, N-+.
		 * oko continue from 4 to 2, push W on stack (NSWNSWW)
		 * Junction at 2 between east and south, so push NSW to stack (NSWNSWWNSW) and go to 1
		 * go down to 4, push N (NSWNSWWNSWN), go right to 3, push W (NSWNSWWNSWNW), go down to 0, push N (NSWNSWWNSWNWN)
		 * Now at 0, there's no way to go besides already mapped tiles, so we start consuming the stack and on the way back we look
		 * for junctions. Consume N (go to 3) consume W (go to 4) consume N (go to 1) consume W (go to 2)
		 * consume S go to 9 is unexplored, stack is (NSWNSWWN) AAAUUUGGGHHH theres something maybe weirdge here revisit later
		 * then go down 9 branch same way as before.
		 * Once that branch is done, we're gonna return to the stack being NSWNSWWN
		 * consume N -> 2, consume W -> 4, consume W -> 0, consume S -> 0, stack is now NSWN
		 * that south 0 has no continuation so we just clean it
		 * consume N -> 0, consume W -> +, consume S -> 3, stack is now N.
		 * that south 3 has no junction and no continuation so we just clean it, and return to base.
		 * Might have to play around with when it pushes directions into the stack.
		 * */
	}
}
