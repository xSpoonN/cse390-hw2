#ifndef DIRECTIONS
#define DIRECTIONS
//                      0      1      2     3     4     5
enum class Direction { NONE, NORTH, EAST, SOUTH, WEST, STAY };

inline std::string dirstr(Direction dir) {
	switch (dir) {
		case Direction::NONE: return "None";
		case Direction::NORTH: return "North";
		case Direction::EAST: return "East";
		case Direction::SOUTH: return "South";
		case Direction::WEST: return "West";
		case Direction::STAY: return "Stay";
		default: return "?";
	}
}

/* Print out the enum as its name*/
inline std::ostream& operator<<(std::ostream& os, Direction dir) {
	os << dirstr(dir);
	return os;
}

inline Direction opposite(Direction dir) {
	switch (dir) {
		case Direction::NORTH: return Direction::SOUTH;
		case Direction::EAST: return Direction::WEST;
		case Direction::SOUTH: return Direction::NORTH;
		case Direction::WEST: return Direction::EAST;
		default: return Direction::NONE;
	}
}

#endif