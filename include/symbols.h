#ifndef SYMBOLS
#define SYMBOLS

namespace Sym {

	constexpr char NONE = ' ';
	constexpr char WALL = '.';
	constexpr char CHARGER = '+';
	constexpr char ROBOTSYM = 'x';
	constexpr char DIRT0 = '0';
	constexpr char DIRT1 = '1';
	constexpr char DIRT2 = '2';
	constexpr char DIRT3 = '3';
	constexpr char DIRT4 = '4';
	constexpr char DIRT5 = '5';
	constexpr char DIRT6 = '6';
	constexpr char DIRT7 = '7';
	constexpr char DIRT8 = '8';
	constexpr char DIRT9 = '9';

	inline bool is_wall(char ch) {
		return ch == WALL || ch == NONE;
	}

	inline bool is_clean(char ch) {
		return ch == DIRT0;
	}

	inline int get_dirt_level(char ch) {
		if (is_clean(ch)) return 0;
		switch (ch) {
			case DIRT1: return 1;
			case DIRT2: return 2;
			case DIRT3: return 3;
			case DIRT4: return 4;
			case DIRT5: return 5;
			case DIRT6: return 6;
			case DIRT7: return 7;
			case DIRT8: return 8;
			case DIRT9: return 9;
			default: return -1;
		}
	}

	inline bool is_valid(char ch) {
		return ch == NONE
			|| ch == WALL
			|| ch == CHARGER
			|| ch == ROBOTSYM
			|| ch == DIRT0
			|| ch == DIRT1
			|| ch == DIRT2
			|| ch == DIRT3
			|| ch == DIRT4
			|| ch == DIRT5
			|| ch == DIRT6
			|| ch == DIRT7
			|| ch == DIRT8
			|| ch == DIRT9;
	}

	inline void decrement_dirt(char& ch) {
		switch (ch) {
			case DIRT9: ch = DIRT8; break;
			case DIRT8: ch = DIRT7; break;
			case DIRT7: ch = DIRT6; break;
			case DIRT6: ch = DIRT5; break;
			case DIRT5: ch = DIRT4; break;
			case DIRT4: ch = DIRT3; break;
			case DIRT3: ch = DIRT2; break;
			case DIRT2: ch = DIRT1; break;
			case DIRT1: ch = DIRT0; break;
		}
	}

}

#endif