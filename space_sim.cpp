#include <string>
#include <random>
#include <chrono>
#include <iostream>
#include <SDL3/SDL.h>
#include <Eigen/Dense>
#include "space_sim.h"
#include "globals.h"
#include "graphics.hpp"

int main() {
    auto seed = std::chrono::high_resolution_clock::now().time_since_epoch().count(); // seed for entity gen
	simulatorMain();

	return 0;
}
