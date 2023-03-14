#ifndef Robot_H
#define Robot_H

#include <vector>
#include <algorithm>
#include <unordered_set>
#include <queue>
#include <iostream>
#include <fstream>
#include <string>
#include <stack>
#include <array>

//能够在该类型的工作台上卖何种物品
const std::vector<std::vector<int>>WorkBenchIdForSell{
    {},
    {},
    {},
    {},
    {1, 2},
    {1, 3},
    {2, 3},
    {4, 5, 6},
    {7},
    {1, 2, 3, 4, 5, 6, 7}
};
//该物品能够卖给那种工作台
const std::vector<std::vector<int>>ItemIdForSell{
    {},
    {4, 5, 9},
    {4, 6, 9},
    {5, 6, 9},
    {7, 9},
    {7, 9},
    {7, 9},
    {8, 9},
    {},
    {}
};



struct WorkBenchNodeForRobot {
    int global_id;
    int type;
    double x, y;
    double dis;
    std::unordered_set<int> bag;
    WorkBenchNodeForRobot(int ID, int T, double X, double Y, double D, int ori_material_status):global_id(ID), type(T), x(X), y(Y), dis(D){
        for(int cnt = 0; (ori_material_status >> cnt) != 0; ++cnt) {
            if(((ori_material_status >> cnt) & 1) == 1){
                bag.insert(cnt);
            }
        }
        //std::cerr<<ori_material_status<<std::endl;
    };
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
        
        //存放每一个robot的目标点,买和卖
        std::stack<WorkBenchNodeForRobot> robot_goal_point;

        //得到robot要跑向的目标点 
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
        static std::unordered_set<double> target_set;   
    private:
};
















#endif
