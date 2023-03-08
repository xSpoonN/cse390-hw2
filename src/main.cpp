#include <iostream>
#include <cstring>
#include <fstream>
#include <string>
#include <algorithm>
#include <vector>

#include "symbols.h"
#include "robot.h"

#define err(msg) cout << msg << endl; file.close(); return EXIT_FAILURE;

using std::cout;
using std::endl;
using std::string;
using std::vector;
using house = vector<vector<char>>;

const string MAX_CHARGE_PREFIX = "MAX_CHARGE: ";
const string MAX_STEPS_PREFIX = "MAX_STEPS: ";

static inline void const printarr(const house & model, const std::pair<int, int> p, float currcharge = 0, int currsteps = 0) {
	cout << "Max Charge: " << currcharge << " | Max Steps: " << currsteps << endl;
	for (int i = 0; i < model.size(); i++) {
		for (int j = 0; j < model[0].size(); j++) {
			cout << ((p.first == i && p.second == j) ? 'x' : model[i][j]) << " ";
		}
		cout << endl;
	}
}

int main(int argc, char** argv) {
	/* Verify that a file was passed as an arg. */
	if (argc < 2) {
		cout << "Please specify a file! " << *argv << " <filename>" << endl;
		return EXIT_FAILURE;
	}

	/* Attempt to open the file. */
	std::ifstream file(*(argv+1));
	if (!file.is_open()) { err("Invalid File given"); }

	/* Parse Max Charge */
	string line;
	float charge = 0; int steps = 0; size_t pos = 0;
	if (std::getline(file, line)) {
		if ((pos = line.find(MAX_CHARGE_PREFIX)) == string::npos) { err("Maximum Charge not defined!"); }
		if (line.length() <= MAX_CHARGE_PREFIX.length()) { err("You must specify a value for Max Charge!"); }
		string mctemp = line.substr(pos + MAX_CHARGE_PREFIX.length() - 1);
		string mcin = mctemp.substr(mctemp.find_first_not_of(" "), mctemp.find_last_not_of(" "));
		try {
			charge = std::stof(mcin);
			if (charge <= 0) {
				err("Max Charge must be positive!");
			}
		}
		catch (...) {
			err("Invalid MAX_CHARGE input!");
		}
	} else { err("Maximum Charge not defined!"); }

	/* Parse Max Steps */
	if (std::getline(file, line)) {
		if ((pos = line.find(MAX_STEPS_PREFIX)) == string::npos) { err("Maximum Steps not defined!"); }
		if (line.length() <= MAX_STEPS_PREFIX.length()) { err("You must specify a value for Max Steps!"); }
		string mstemp = line.substr(pos + MAX_STEPS_PREFIX.length() - 1);
		string msin = mstemp.substr(mstemp.find_first_not_of(" "), mstemp.find_last_not_of(" "));
		try {
			steps = std::stoi(msin);
			if (steps <= 0) {
				err("Max Steps must be positive!");
			}
		}
		catch (...) {
			err("Invalid MAX_STEPS input!");
		}
	} else { err("Maximum Steps not defined!"); }

	/* Read input into a "house" */
	house model;
	int row = 0, col = 0;
	std::pair<int, int> start(-1,-1); /* Charger / Start Position */
	vector<int> rowlen; size_t maxlen = 0;
	while (std::getline(file, line)) {
		vector<char> rowvec;
		for (char c : line) {
			if (!Sym::is_valid(c)) { err("Unexpected char in input: " << c); }
			if (c == Sym::NONE) rowvec.push_back(Sym::DIRT0); else rowvec.push_back(c); /* Standardize ' ' to '0' */
			if (c == Sym::CHARGER) {
				if (start.first == -1 && start.second == -1) {
					start = std::make_pair(row, col);
				}
				else { err("Only one start position may be defined!"); }
			}
			++col;
		}
		model.push_back(rowvec);
		rowlen.push_back(static_cast<int>(rowvec.size()));
		if (rowvec.size() > maxlen) maxlen = rowvec.size();
		++row; col = 0;
	}

	/* Verify that there was a charger */
	if (start.first == -1 && start.second == -1) { err("Start position not defined!"); }

	/* Fills out uneven models */
	for (int i = 0; i < rowlen.size(); i++) {
		for (int j = 0; j < maxlen - rowlen[i]; j++) {
			model[i].push_back(Sym::WALL);
		}
	}

	/* Check for solid wall on left/right */
	for (int i = 0; i < model.size(); i++) {
		/* Check left wall */
		if (!Sym::is_wall(model[i][0])) {
			/* Insert missing left wall */
			for (int j = 0; j < model.size(); j++) {
				model[j].insert(model[j].begin(), Sym::WALL);
			}
			++start.second;
		}
		/* Check right wall */
		if (!Sym::is_wall(model[i][model[0].size()-1])) {
			/* Insert missing right wall */
			for (int j = 0; j < model.size(); j++) {
				model[j].push_back(Sym::WALL);
			}
		}
	}

	/* Check for solid wall on top/bottom */
	for (int i = 0; i < model[0].size(); i++) {
		/* Check top wall */
		if (!Sym::is_wall(model[0][i])) {
			/* Insert missing top wall */
			model.insert(model.begin(), vector<char>(model[0].size(), Sym::WALL));
			++start.first;
		}
		/* Check bottom wall */
		if (!Sym::is_wall(model[model.size()-1][i])) {
			/* Insert missing bottom wall */
			model.push_back(vector<char>(model[0].size(), Sym::WALL));
		}
	}

	/* Display model to console */
	printarr(model, start, charge, steps);

	/* Setup output file */
	std::ofstream outputFile("output.txt", std::fstream::trunc);
	if (!outputFile.is_open()) {
		cout << "Failed to open output file" << endl;
		return EXIT_FAILURE;
	}

	/* Check for visual simulation flag */
	int step = -1;
	if (argc >= 3) {
		try {
			step = std::stoi(*(argv+2));
		}
		catch (...) {
			step = -1;
		}
	}

	/* Create robot and clean house */
	Robot robot(model, charge, steps, start.first, start.second, step);
	int success = robot.clean_house(outputFile);

	/* Cleanup */
	outputFile.close();
	if (success) cout << "Exit Success" << endl;
	//printarr(model, start, charge, steps);
	return success ? EXIT_SUCCESS : EXIT_FAILURE;
}