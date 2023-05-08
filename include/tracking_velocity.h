#pragma once

// TODO WHAT IS THIS FILE? DELETE! NEVER USED - NO CPP ATTACHED
#include "math.h"
#include <cstdio>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

float calculate_distance(std::vector<float>& point1, std::vector<float>& point2);
void get_position_in_line(std::vector<float>& position_in_line,
                          std::vector<float> desired_hitting_position,
                          std::vector<float> end_effector_position,
                          std::vector<float> desired_impact_velocity);
void voila();
