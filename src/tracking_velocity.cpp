#include "../include/tracking_velocity.h"


float calculate_distance(std::vector<float> point1, std::vector<float> point2){
    float sum = 0;

    for (unsigned int i = 0; i < point1.size(); i++){
        sum += (point1[i] - point2[i])*(point1[i] - point2[i]);
    }
    return sqrt(sum);
}

void get_position_in_line(std::vector<float> &position_in_line, std::vector<float> desired_hitting_position, std::vector<float> end_effector_position, std::vector<float> desired_impact_velocity){
    int length = position_in_line.size();

    float hitting_speed = std::inner_product(desired_impact_velocity.begin(), desired_impact_velocity.end(), desired_impact_velocity.begin(), 0);
    position_in_line = desired_hitting_position - (d/hitting_speed)*desired_impact_velocity;

}

void voila(){
    std::cout << "This file does work" << std::endl;
}