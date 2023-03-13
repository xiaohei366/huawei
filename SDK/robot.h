#ifndef Robot_H
#define Robot_H

#include <vector>
#include <algorithm>
#include <unordered_set>
#include <queue>
#include <iostream>
#include <fstream>
#include <string>


struct WorkBenchNodeForRobot {
    int type;
    double x, y;
    double dis;
    WorkBenchNodeForRobot(int T, double X, double Y, double D):type(T), x(X), y(Y), dis(D){};
};

struct cmp_rule {
    bool operator() (const WorkBenchNodeForRobot a, const WorkBenchNodeForRobot b) {
        return a.dis > b.dis; // 按照dis从小到大排序
    }
};




class Robot {
    public:
        explicit Robot(double x, double y):location_x(x), location_y(y) {
            for(int i = 0; i < 10; ++i) {
                std::vector<WorkBenchNodeForRobot> tem;
                workbench_for_robot.push_back(tem);
            }
        }
        ~Robot() = default;


        void Update(int workstation_id, int carried_item_type, double time_value_coefficient, double collision_value_coefficient, 
        double angular_velocity, double linear_velocity_x, double linear_velocity_y, double direction,
        double location_x, double location_y);

       
        WorkBenchNodeForRobot GetTarget();
        void Clear_vec();
        static void Clear_set() {
            target_set.clear();
        };
        //机器人面对不同类型工作台的情况
        std::vector<std::vector<WorkBenchNodeForRobot>> workbench_for_robot;



        
        //下面是机器人的属性
        int workstation_id;
        int carried_item_type;
        double time_value_coefficient;
        double collision_value_coefficient;
        double angular_velocity;
        double linear_velocity_x, linear_velocity_y;
        double direction;
        double location_x, location_y;
    private:
        static std::unordered_set<double> target_set;   
};
















#endif
