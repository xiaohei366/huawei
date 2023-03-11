#include "robot.h"




void Robot::Updata(int workstation_id, int carried_item_type, float time_value_coefficient, float collision_value_coefficient, 
        float angular_velocity, float linear_velocity_x, float linear_velocity_y, float direction,
        float location_x, float location_y) {
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