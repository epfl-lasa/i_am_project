#pragma once


#include <cstdio>
#include <iostream>


#include "math.h"

#include <vector>
#include <string>



float calculate_distance(std::vector<float> point1, std::vector<float> point2);
void get_position_in_line(std::vector<float> position_in_line, std::vector<float> desired_hitting_position, std::vector<float> iiwa_end_effector_position);
void voila();
