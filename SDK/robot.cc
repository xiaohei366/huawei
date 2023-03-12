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






WorkBenchNodeForRobot* Robot::GetTarget() {
    for(int i = 0; i < workbench_for_robot.size(); ++i) {
        //先排序
        auto &vec = workbench_for_robot[i];
        std::sort(vec.begin(), vec.end(),[](const WorkBenchNodeForRobot a, const WorkBenchNodeForRobot b)  {
            return a.dis < b.dis;
        });
    }
    if(workbench_for_robot[1].size() != 0) return &workbench_for_robot[1][0];
    else if (workbench_for_robot[2].size() != 0) return &workbench_for_robot[2][0];
    else return &workbench_for_robot[3][0];
}

void Robot::Clear_vec() {
        for(int i = 0; i < 10; ++i) {
        workbench_for_robot[i].clear();
    }
}



