#include "robot.h"

std::unordered_set<double> Robot::target_set;


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






WorkBenchNodeForRobot Robot::GetTarget() {
    //默认的点--默认工作台类型为1---就是原地打圈
    WorkBenchNodeForRobot default_node =  WorkBenchNodeForRobot(1, this->location_x, this->location_y, 0.5, 0);
    //初始化该机器人所有工作台的候补队列
    std::vector<std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule>>robot_target_queue;
    for(int i = 1; i <= 9; ++i) {
        auto &vec = workbench_for_robot[i];
        std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule> pq;
        for(auto &v: vec) {
            pq.push(v);
        }
        robot_target_queue.push_back(pq);
    }
    //如果是买--买前三个里面离着最近的(从1-3的顺序找)
    if(this->carried_item_type == 0) {
        for(int i = 1; i <= 3; ++i) {
            auto& pq = robot_target_queue[i];
            //再取出来
            while(!pq.empty()) {
                if(target_set.count(pq.top().x*100+pq.top().y)) pq.pop();
                else {
                    target_set.insert(pq.top().x*100+pq.top().y);
                    return pq.top();
                }   
            } 
        }
    }    //如果是买--卖前三个里面离着最近的(从4->6的顺序找)
    else {
        for(int i = 4; i <= 6; ++i) {
            auto& pq = robot_target_queue[i];
            //再取出来
            while(!pq.empty()) {
                if(target_set.count(pq.top().x*100+pq.top().y)) pq.pop();
                else {
                    target_set.insert(pq.top().x*100+pq.top().y);
                    return pq.top();
                }
            }
        }
    } 
    return default_node;
}

void Robot::Clear_vec() {
    for(int i = 0; i < 10; ++i) {
        workbench_for_robot[i].clear();
    }
}



