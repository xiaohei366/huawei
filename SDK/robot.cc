#include "robot.h"




void Robot::Update(int workstation_id, int carried_item_type, double time_value_coefficient, double collision_value_coefficient, 
        double angular_velocity, double linear_velocity_x, double linear_velocity_y, double direction,
        double location_x, double location_y) {
    this->workstation_id = workstation_id;
    this->carried_item_type = carried_item_type;
    this->time_value_coefficient = time_value_coefficient;
    this->collision_value_coefficient = collision_value_coefficient;
    this->angular_velocity = angular_velocity;
    this->linear_velocity_x = linear_velocity_x;
    this->linear_velocity_y = linear_velocity_y;
    this->direction = direction;
    this->location_x = location_x;
    this->location_y = location_y;
}
