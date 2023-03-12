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






WorkBenchNodeForRobot* Robot::GetTarget() {
    std::priority_queue<WorkBenchNodeForRobot*>pq;
    //默认的点--默认工作台类型为1
    // 打日志debug
    //std::ofstream logfile;
    //logfile.open("./log.txt", std::ios::app); // 打开日志文件，追加写入
    //if (logfile.is_open()) { // 判断文件是否打开成功
    //for(auto &t: target_set){
    //logfile << t  << std::endl; // 写入日志
    //}
    //logfile.close(); // 关闭文件
    //}
    WorkBenchNodeForRobot* default_node =  new WorkBenchNodeForRobot(1, this->location_x, this->location_y, 0.5);
    //如果是买--买前三个里面离着最近的
    if(this->carried_item_type == 0) {
        for(int i = 1; i <= 3; ++i) {
            auto &vec = workbench_for_robot[i];
            for(auto &v: vec) {
                pq.push(&v);
            }
        }
        //再取出来
        while(!pq.empty()) {
            if(target_set.count(pq.top()->x*100+pq.top()->y)) pq.pop();
            else {
                target_set.insert(pq.top()->x*100+pq.top()->y);
                return pq.top();
            }
        } 
        return default_node; 
        // std::sort(vec.begin(), vec.end(),[](const WorkBenchNodeForRobot a, const WorkBenchNodeForRobot b)  {
        //     return a.dis < b.dis;
        // });
    }
    else {
        for(int i = 4; i <= 6; ++i) {
            auto &vec = workbench_for_robot[i];
            for(auto &v: vec) {
                pq.push(&v);
            }
        }
        //再取出来
        while(!pq.empty()) {
            if(target_set.count(pq.top()->x*100+pq.top()->y)) pq.pop();
            else {
                target_set.insert(pq.top()->x*100+pq.top()->y);
                return pq.top();
            }
        }
        return default_node; 
        } 
    return nullptr;
}

void Robot::Clear_vec() {
    for(int i = 0; i < 10; ++i) {
        workbench_for_robot[i].clear();
    }
}



