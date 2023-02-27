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

struct Position { int x, y; };
class Node {
public:
	Position coords;
	vector<Node*> neighbours;
	Direction pre;
	Node(Position coords, Direction pre = Direction::NONE) : coords(coords), pre(pre) {}

	bool operator==(const Node& r) {
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
	/* This stuff should be moved somewhere else, since it's only initialized once */
	Node start(Position{ 0, 0 });
	unordered_set<Position> visited;
	visited.insert(Position{ 0, 0 });
	vector<Direction> pathToNextBranch;
	/*******************************************************************************/


	vector<Direction> choice;
	Node* c = &start;
	if (!rob->is_wall(Direction::NORTH) && visited.find(c->nCoords()) == visited.end()) {
		/* Either don't keep track of neighbours like this, or maybe add a check that the node hasn't been already added. */
		c->neighbours.push_back(new Node(c->nCoords(), Direction::SOUTH));
		choice.push_back(Direction::NORTH);
	}
	if (!rob->is_wall(Direction::EAST) && visited.find(c->eCoords()) == visited.end()) {
		/* Either don't keep track of neighbours like this, or maybe add a check that the node hasn't been already added. */
		c->neighbours.push_back(new Node(c->eCoords(), Direction::WEST));
		choice.push_back(Direction::EAST);
	}
	if (!rob->is_wall(Direction::SOUTH) && visited.find(c->sCoords()) == visited.end()) {
		/* Either don't keep track of neighbours like this, or maybe add a check that the node hasn't been already added. */
		c->neighbours.push_back(new Node(c->sCoords(), Direction::NORTH));
		choice.push_back(Direction::SOUTH);
	}
	if (!rob->is_wall(Direction::WEST) && visited.find(c->wCoords()) == visited.end()) {
		/* Either don't keep track of neighbours like this, or maybe add a check that the node hasn't been already added. */
		c->neighbours.push_back(new Node(c->wCoords(), Direction::EAST));
		choice.push_back(Direction::WEST);
	}
	if (choice.size() == 0) {
		/* Either we're in an enclosed area, or we have fully explored this branch
		 * If pathToNextBranch is empty, we return stay, since we're blocked in
		 * Otherwise, consume pathToNextBranch until we get to the next node
		 * In standard dfs we just go straight to the next node, but this is a physical roomba
		 * So we need to keep track of the route as we're branching */
	} else {
		visited.insert(c->getCoords(choice[0])); /* Makes sure we don't later on remap this node we're visiting. */
		return choice[0];
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
