#include "robot.h"



std::unordered_set<std::pair<double, int>, Robot::PairHash, Robot::PairEqual> Robot::target_set;
int Robot::CircularArrayPtr = 0;


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




WorkBenchNodeForRobot Robot::GetTarget1() {
    //默认的点--默认工作台类型为1,全局ID为-2---就是原地打圈
    WorkBenchNodeForRobot default_node =  WorkBenchNodeForRobot(-2, 1, this->location_x, this->location_y, 0.5, 0, 0);
    //初始化该机器人所有工作台的候补队列 以及 高优先度生产好的点
    int cnt = 0; //高优先度点的个数
    std::vector<std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule>>robot_target_queue;
    std::vector<std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule>>greater_level_queue;
    for(int i = 0; i <= 9; ++i) {
        auto &vec = workbench_for_robot[i];
        std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule> pq;
        std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule> pq2; //分别对应上面两个有限队列数组的临时队列
        for(auto &v: vec) {
            pq.push(v);
            if(i >= 4 && v.product_status == 1) {
                pq2.push(v);
                ++cnt;
            }
        }
        robot_target_queue.push_back(pq);
        greater_level_queue.push_back(pq2);
    }
    WorkBenchNodeForRobot ans(-2, 1, this->location_x, this->location_y, 0.5, 0, 0);
    if(cnt != 0) ans =  Num789(default_node, robot_target_queue, greater_level_queue);
    if(robot_goal_point.empty()) ans =  Num456(default_node, robot_target_queue);
    
    return ans;
}

WorkBenchNodeForRobot Robot::GetTarget2() {
    //默认的点--默认工作台类型为1,全局ID为-2---就是原地打圈
    WorkBenchNodeForRobot default_node =  WorkBenchNodeForRobot(-2, 1, this->location_x, this->location_y, 0.5, 0, 0);
    //初始化该机器人所有工作台的候补队列 以及 高优先度生产好的点
    int cnt = 0; //高优先度点的个数
    std::vector<std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule>>robot_target_queue;
    std::vector<std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule>>greater_level_queue;
    for(int i = 0; i <= 9; ++i) {
        auto &vec = workbench_for_robot[i];
        std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule> pq;
        std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule> pq2; //分别对应上面两个有限队列数组的临时队列
        for(auto &v: vec) {
            pq.push(v);
            if(i >= 4 && v.product_status == 1) {
                pq2.push(v);
                ++cnt;
            }
        }
        robot_target_queue.push_back(pq);
        greater_level_queue.push_back(pq2);
    }
    //std::cerr<<"start gettarget2"<<std::endl;
    WorkBenchNodeForRobot ans(-2, 1, this->location_x, this->location_y, 0.5, 0, 0);
    if(cnt != 0) ans =  Num789(default_node, robot_target_queue, greater_level_queue);
    if(robot_goal_point.empty()) ans =  Num456(default_node, robot_target_queue);
    
    return ans;
}

WorkBenchNodeForRobot Robot::GetTarget3() {
    //默认的点--默认工作台类型为1,全局ID为-2---就是原地打圈
    WorkBenchNodeForRobot default_node =  WorkBenchNodeForRobot(-2, 1, this->location_x, this->location_y, 0.5, 0, 0);
    //初始化该机器人所有工作台的候补队列 以及 高优先度生产好的点
    int cnt = 0; //高优先度点的个数
    std::vector<std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule>>robot_target_queue;
    std::vector<std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule>>greater_level_queue;
    for(int i = 0; i <= 9; ++i) {
        auto &vec = workbench_for_robot[i];
        std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule> pq;
        std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule> pq2; //分别对应上面两个有限队列数组的临时队列
        for(auto &v: vec) {
            pq.push(v);
            if(i >= 4 && v.product_status == 1) {
                pq2.push(v);
                ++cnt;
            }
        }
        robot_target_queue.push_back(pq);
        greater_level_queue.push_back(pq2);
    }
    WorkBenchNodeForRobot ans(-2, 1, this->location_x, this->location_y, 0.5, 0, 0);
    if(cnt != 0) ans =  Num789(default_node, robot_target_queue, greater_level_queue);
    if(robot_goal_point.empty()) ans =  Num456(default_node, robot_target_queue);
    
    return ans;
}

WorkBenchNodeForRobot Robot::GetTarget4() {
    //默认的点--默认工作台类型为1,全局ID为-2---就是原地打圈
    WorkBenchNodeForRobot default_node =  WorkBenchNodeForRobot(-2, 1, this->location_x, this->location_y, 0.5, 0, 0);
    //初始化该机器人所有工作台的候补队列 以及 高优先度生产好的点
    int cnt = 0; //高优先度点的个数
    std::vector<std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule>>robot_target_queue;
    std::vector<std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule>>greater_level_queue;
    for(int i = 0; i <= 9; ++i) {
        auto &vec = workbench_for_robot[i];
        std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule> pq;
        std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule> pq2; //分别对应上面两个有限队列数组的临时队列
        for(auto &v: vec) {
            pq.push(v);
            if(i >= 4 && v.product_status == 1) {
                pq2.push(v);
                ++cnt;
            }
        }
        robot_target_queue.push_back(pq);
        greater_level_queue.push_back(pq2);
    }
    WorkBenchNodeForRobot ans(-2, 1, this->location_x, this->location_y, 0.5, 0, 0);
    if(cnt != 0) ans =  Num789(default_node, robot_target_queue, greater_level_queue);
    if(robot_goal_point.empty()) ans =  Num456(default_node, robot_target_queue);
    
    return ans;
}


void Robot::Clear_vec() {
    for(int i = 0; i < 10; ++i) {
        workbench_for_robot[i].clear();
    }
}


WorkBenchNodeForRobot Robot::Num789(WorkBenchNodeForRobot &default_node, std::vector<std::priority_queue<WorkBenchNodeForRobot, 
        std::vector<WorkBenchNodeForRobot>, cmp_rule>> robot_target_queue, std::vector<std::priority_queue<WorkBenchNodeForRobot, 
        std::vector<WorkBenchNodeForRobot>, cmp_rule>> greater_level_queue
) {
    for(int i=7;i<=9;i++)
    {
        //创建一个二级优先队列，存放堆栈内
        while(!robot_target_queue[i].empty() && robot_goal_point.size() != 2)
        {
            auto m = robot_target_queue[i].top();
            int type = m.type;
            double coordinate_x = m.x;
            double coordinate_y = m.y;
            std::unordered_set<int> us = m.bag;
            
            // if(target_set.count({coordinate_x*100+coordinate_y, type}) != 0)
            // {
            //     robot_target_queue[i].pop();
            //     continue;
            // }

            //当其为0的时候，表示，工作台缺少原材料
            for(int j=0;j<WorkBenchIdForSell[type].size();j++)
            {
                int raw_material_type = WorkBenchIdForSell[type][j];
                //如果当前装原材料的格子是空的，然后需要去判断有没有别的robot在执行该任务
                //double date_tmp=coordinate_x*100+coordinate_y;
                //std::cerr<<"data tmp = "<<date_tmp<<std::endl;
                if(us.count(raw_material_type) == 0 && target_set.count({coordinate_x*100+coordinate_y, raw_material_type}) == 0)
                {
                    robot_goal_point.push(m);
                    //下面开始在最近的1--3号工作台
                    double new_idential = coordinate_x*100+coordinate_y;
                    target_set.insert({new_idential, raw_material_type});


                    auto que = greater_level_queue[raw_material_type];
                    while(!que.empty() && robot_goal_point.size() == 1)
                    {
                        
                        double unique_idential = que.top().x*100+que.top().y;
                        if(target_set.count({unique_idential,raw_material_type}))
                        {
                            que.pop();
                        }
                        else
                        {
                            //若寻找成功，则将售卖工作台的类型改为缺少的原材料再返回
                            robot_goal_point.top().type = raw_material_type;
                            target_set.insert({unique_idential,raw_material_type});
                            robot_goal_point.push(que.top());
                        }
                    }
                    if(robot_goal_point.size() == 1)
                    {
                        target_set.erase({robot_goal_point.top().x*100+robot_goal_point.top().y, raw_material_type});
                        robot_goal_point.pop();
                    }
                    else break;
                }
            }
            if(robot_goal_point.size() != 2)
            {
                //将该工作台从优先队列删除
                robot_target_queue[i].pop();
            }
        }
        
    }
    return default_node;
}





WorkBenchNodeForRobot Robot::Num456(WorkBenchNodeForRobot &default_node, std::vector<std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule>> robot_target_queue) { 
    //具体的策略
    for(int k=0 ;k <= 2;++k)
    {
        int num_circular_array = CircularArray.size();
        int i = CircularArray[(k + CircularArrayPtr + num_circular_array) % num_circular_array];
        while(!robot_target_queue[i].empty() && robot_goal_point.size() != 2)
        {
            auto m = robot_target_queue[i].top();
            int type = m.type;
            double coordinate_x = m.x;
            double coordinate_y = m.y;
            std::unordered_set<int> us = m.bag;

            //当其为0的时候，表示，工作台缺少原材料
            for(int j=0;j<WorkBenchIdForSell[type].size();j++)
            {
                int raw_material_type = WorkBenchIdForSell[type][j];
                
                //这个材料为空并且这个材料没有有机器人处理才能去判断找合适的购买工作台
                if(us.count(raw_material_type) == 0 && target_set.count({coordinate_x*100+coordinate_y, raw_material_type}) == 0)
                {
                    
                    robot_goal_point.push(m);
                    //由于4-6里面有工作台缺少材料，
                    double new_idential = coordinate_x*100+coordinate_y;
                    target_set.insert({new_idential, raw_material_type});

                    //下面开始找4-6相应队列里最近的1--3号工作台
                    auto que = robot_target_queue[raw_material_type];
                    while(!que.empty() && robot_goal_point.size() == 1)
                    {
                        
                        double unique_idential = que.top().x*100+que.top().y;
                        if(target_set.count({unique_idential, raw_material_type}))
                        {
                            que.pop();
                        }
                        else
                        {
                            //若寻找成功，则将售卖工作台的类型改为缺少的原材料再返回
                            robot_goal_point.top().type = raw_material_type;
                            target_set.insert({unique_idential, raw_material_type});
                            robot_goal_point.push(que.top());
                            //同时修改循环数组的指针
                            CircularArrayPtr++;
                            if(CircularArrayPtr >= CircularArray.size()) CircularArrayPtr = 0;
                        }
                    }
                    if(robot_goal_point.size() == 1)
                    {
                        target_set.erase({robot_goal_point.top().x*100+robot_goal_point.top().y, raw_material_type});
                        robot_goal_point.pop();
                    }
                    else break;
                }
            }
            if(robot_goal_point.size() != 2)
            {
                //将该工作台从优先队列删除
                robot_target_queue[i].pop();
            }
        }
          
    }
    return default_node;
}