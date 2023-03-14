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
    //默认的点--默认工作台类型为1,全局ID为-2---就是原地打圈
    WorkBenchNodeForRobot default_node =  WorkBenchNodeForRobot(-2, 1, this->location_x, this->location_y, 0.5, 0);
    //初始化该机器人所有工作台的候补队列
    std::vector<std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule>>robot_target_queue;
    for(int i = 0; i <= 9; ++i) {
        auto &vec = workbench_for_robot[i];
        std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule> pq;
        for(auto &v: vec) {
            pq.push(v);
        }
        robot_target_queue.push_back(pq);
        
    }
    //具体的策略
    for(int i=4;i<=9;i++)
    {
        while(!robot_target_queue[i].empty() && robot_goal_point.size() != 2)
        {
            auto m = robot_target_queue[i].top();
            int type = m.type;
            double coordinate_x = m.x;
            double coordinate_y = m.y;
            std::unordered_set<int> us = m.bag;
            //for(auto &n : us)
            //    std::cerr<<n<<std::endl;
            if(target_set.count(coordinate_x*100+coordinate_y) != 0)
            {
                robot_target_queue[i].pop();
                continue;
            }

            /*if(i == 7)
            {
                std::cerr<<<<std::endl;
            } */

            //当其为0的时候，表示，工作台缺少原材料
            for(int j=0;j<WorkBenchIdForSell[type].size();j++)
            {
                int raw_material_type = WorkBenchIdForSell[type][j];
                //如果当前装原材料的格子是空的，然后需要去判断有没有别的robot在执行该任务
                //double date_tmp=coordinate_x*100+coordinate_y;
                //std::cerr<<"data tmp = "<<date_tmp<<std::endl;
                if(us.count(raw_material_type) == 0)
                {
                    
                    //std::cerr<<coordinate_x*100+coordinate_y<<std::endl;
                    //robot_goal_point.push({coordinate_x,coordinate_y});
                    robot_goal_point.push(m);
                    //下面开始在最近的1--3号工作台
                    double new_idential = coordinate_x*100+coordinate_y;
                    target_set.insert(new_idential);


                    auto que = robot_target_queue[raw_material_type];
                    while(!que.empty() && robot_goal_point.size() == 1)
                    {
                        
                        double unique_idential = que.top().x*100+que.top().y;
                        if(target_set.count(unique_idential))
                        {
                            que.pop();
                        }
                        else
                        {
                            target_set.insert(unique_idential);
                            /*double raw_material_coordinate_x = que.top().x;
                            double raw_material_coordinate_y = que.top().y;
                            WorkBenchNodeForRobot raw_material_workbench;
                            raw_material_workbench.type = raw_material_type;
                            raw_material_workbench.x = raw_material_coordinate_x;
                            raw_material_workbench.y = raw_material_coordinate_y;
                            raw_material_workbench.dis = */
                            robot_goal_point.push(que.top());
                        }
                    }
                    break;
                }
            }
            //当robot没有找到了新的目标点，说明上面的这个工作台是满的
            if(robot_goal_point.size() == 1)
            {
                //将该工作台从优先队列删除
                robot_target_queue[i].pop();
                target_set.erase(robot_goal_point.top().x*100+robot_goal_point.top().y);
                robot_goal_point.pop();
            }
            else if(robot_goal_point.size() == 0)
            {
                robot_target_queue[i].pop();
            }
        }
          
    }
    //robot_goal_point.push({default_node.x,default_node.y});
    return default_node;


}


void Robot::Clear_vec() {
    for(int i = 0; i < 10; ++i) {
        workbench_for_robot[i].clear();
    }
}



