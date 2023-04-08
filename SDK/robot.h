#ifndef Robot_H
#define Robot_H

#include <list>
#include <limits.h>
#include <vector>
#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <iostream>
#include <fstream>
#include <string>
#include <stack>
#include <array>
#include <cmath>
#include <cfloat>
#include <fstream>


//能够在该类型的工作台上卖何种物品
const std::vector<std::vector<int>>WorkBenchIdForSell{
    {},
    {},
    {},
    {},
    {2, 1},
    {3, 1},
    {3, 2},
    {6, 5, 4},
    {7},
    {7, 6, 5, 4}
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
    int real_type;
    double x, y;
    double dis;
    int product_status;
    int remain_production_time;
    std::unordered_set<int> bag;

    bool is_task_over;
    std::vector<std::vector<std::vector<std::array<double,4>>>> workbench_route_workbench;
    //std::vector<std::stack<std::pair<double,double>>> workbench_route_workbench;

    WorkBenchNodeForRobot(int ID, int T, int T_, double X, double Y, double D, int ori_material_status, int ProductStatus, int remain_p_time, bool is_execute_over_, std::vector<std::vector<std::vector<std::array<double,4>>>> workbench_route_workbench_):global_id(ID), type(T), real_type(T_), x(X), y(Y), dis(D), product_status(ProductStatus), remain_production_time(remain_p_time), is_task_over(is_execute_over_), workbench_route_workbench(workbench_route_workbench_){
        for(int cnt = 0; (ori_material_status >> cnt) != 0; ++cnt) {
            if(((ori_material_status >> cnt) & 1) == 1){
                bag.insert(cnt);
            }
        }
        //std::cerr<<ori_material_status<<std::endl;
    };
    WorkBenchNodeForRobot(){};
};



struct cmp_rule {
    bool operator() (const WorkBenchNodeForRobot& a, const WorkBenchNodeForRobot& b) {
        return a.dis > b.dis; // 按照dis从小到大排序
    }
};


struct sNode
{
	bool bObstacle;			// Is the node an obstruction?
	bool bVisited;			// Have we searched this node before?
    char symbol;
    double fGlobalGoal;				// Distance to goal so far
	double fLocalGoal;	
    int node_cost;
    int planning_point_nums;
	int x;							// Nodes position in 2D space
	int y;
    double coordinate_x;
    double coordinate_y;
    
	std::vector<sNode*> vecNeighbours;	// Connections to neighbours
	
    sNode* parent;					// Node connecting to this node that offers shortest parent
    sNode(): bObstacle(false),bVisited(false), symbol('.'), fGlobalGoal(100000),fLocalGoal(100000),node_cost(1),planning_point_nums(1),x(0),y(0),coordinate_x(25.0f),coordinate_y(25.0f),parent(nullptr){
        //vecNeighbours.reserve(9);
    }
};


class Robot {
    public:
        explicit Robot(double x, double y):location_x(x), location_y(y) {
            all_node_object = std::vector<sNode>(10000);   
            fixed_all_node_object = std::vector<sNode>(10000);

            all_node_empty = std::vector<sNode>(10000);
            fixed_all_node_empty = std::vector<sNode>(10000);

            node_tmp_object = std::vector<sNode>(10);
            node_tmp_empty = std::vector<sNode>(10);

            robot_exe_pts_ptr = 0;
            for(int i = 0; i < 10; ++i) {
                std::vector<WorkBenchNodeForRobot> tem;
                workbench_for_robot.push_back(tem);
            }
        }
        ~Robot() = default;
           
        void Update(int workstation_id, int carried_item_type, double time_value_coefficient, double collision_value_coefficient, 
        double angular_velocity, double linear_velocity_x, double linear_velocity_y, double direction,
        double location_x, double location_y);
        
        std::vector<sNode> node_tmp_object;
        sNode node_hwcompet_tmp;
        std::vector<sNode> node_tmp_empty;
        //携带物品
        std::vector<sNode> all_node_object;
        std::vector<sNode> fixed_all_node_object;

        //不携带物品
        std::vector<sNode> all_node_empty;
        std::vector<sNode> fixed_all_node_empty;

        void astar(struct sNode *nodeStart, struct sNode *nodeEnd, bool carry_object);

        //存放每一个robot的目标点,买和卖
        std::stack<WorkBenchNodeForRobot> robot_goal_point;
        
        
        std::vector<WorkBenchNodeForRobot> robot_execute_points;
        int robot_exe_pts_ptr;
        //std::stack<WorkBenchNodeForRobot> greater_level_point;
        //得到robot要跑向的目标点 
        WorkBenchNodeForRobot GetTarget1(int robotID);
        WorkBenchNodeForRobot GetTarget2(int robotID);
        WorkBenchNodeForRobot GetTarget3(int robotID);
        WorkBenchNodeForRobot GetTarget4(int robotID);

        bool remove_middle_points(std::pair<double,double>& point1, std::pair<double,double>& point2, std::pair<double,double>& point3);
        WorkBenchNodeForRobot Num123(int robotID, WorkBenchNodeForRobot &default_node, 
        std::vector<std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule>> robot_target_queue);
        
        WorkBenchNodeForRobot Num456(int robotID, WorkBenchNodeForRobot &default_node, 
        std::vector<std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule>> robot_target_queue);

        WorkBenchNodeForRobot Num789(int robotID, WorkBenchNodeForRobot &default_node, std::vector<std::priority_queue<WorkBenchNodeForRobot, 
        std::vector<WorkBenchNodeForRobot>, cmp_rule>> robot_target_queue,std::vector<std::priority_queue<WorkBenchNodeForRobot, 
        std::vector<WorkBenchNodeForRobot>, cmp_rule>> greater_level_queue
        );
      
        WorkBenchNodeForRobot Num89(int robotID, WorkBenchNodeForRobot &default_node, std::vector<std::priority_queue<WorkBenchNodeForRobot, 
        std::vector<WorkBenchNodeForRobot>, cmp_rule>> robot_target_queue,std::vector<std::priority_queue<WorkBenchNodeForRobot, 
        std::vector<WorkBenchNodeForRobot>, cmp_rule>> greater_level_queue
        );



        void Clear_vec();
        static void Clear_set() {
            target_set.clear();
        };
        //机器人面对不同类型工作台的情况
        std::vector<std::vector<WorkBenchNodeForRobot>> workbench_for_robot;      
        std::unordered_map<int,std::vector<std::vector<std::vector<std::array<double,4>>>>> target_route_points;
        //定义能够存储pair类型的set
        struct PairHash {
            std::size_t operator()(const std::pair<double, int>& p) const {
                return std::hash<double>()(p.first) ^ std::hash<int>()(p.second);
            }
        };
        struct PairEqual {
            bool operator()(const std::pair<double, int>& a, const std::pair<double, int>& b) const {
                return a.first == b.first && a.second == b.second;
            }
        };

        std::vector<std::vector<std::vector<double>>> workbench_tmp;
        
        //下面是机器人的属性
        int ptr_flag = 0;
        int clash_cnt = 0;



        int workstation_id;
        int carried_item_type;
        double time_value_coefficient;
        double collision_value_coefficient;
        double angular_velocity;
        double linear_velocity_x, linear_velocity_y;
        double direction;
        double location_x, location_y;

        static std::unordered_set<std::pair<double, int>, PairHash, PairEqual> target_set;

        const std::vector<int> CircularArray{6, 5, 4};
       
        int CircularArrayPtr = 0;
              
};
















#endif
