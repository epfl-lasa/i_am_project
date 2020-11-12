#include "../include/tracking_velocity.h"


float calculate_distance(std::vector<float> &point1, std::vector<float> &point2){
    float sum = 0;

    for (unsigned int i = 0; i < point1.size(); i++){
        sum += (point1[i] - point2[i])*(point1[i] - point2[i]);
    }
    //std::cout << sum << std::endl;
    return sqrt(sum);
}

void get_position_in_line(std::vector<float> &position_in_line, std::vector<float> desired_hitting_position, std::vector<float> end_effector_position, std::vector<float> desired_impact_velocity){
    int length = 3;

    float d = calculate_distance(end_effector_position, desired_hitting_position);

    // std::cout << "distance is" << d << std::endl;
    position_in_line.clear();

    float hitting_speed = sqrt(std::inner_product(desired_impact_velocity.begin(), desired_impact_velocity.end(), desired_impact_velocity.begin(), 0));
    
    std::cout << "hitting speed" << hitting_speed << std::endl;

    for (int i = 0; i < length; i++){
        float a = desired_hitting_position[i] - float(d/hitting_speed)*desired_impact_velocity[i];
        position_in_line.push_back(a);
    }

}

void voila(){
    std::cout << "This file does work" << std::endl;
}