#include "robot.h"



std::unordered_set<std::pair<double, int>, Robot::PairHash, Robot::PairEqual> Robot::target_set;
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


void Robot::Clear_vec() {
    for(int i = 0; i < 10; ++i) {
        workbench_for_robot[i].clear();
    }
}

WorkBenchNodeForRobot Robot::GetTarget1(int robotID) {
    //默认的点--默认工作台类型为1,全局ID为-2---就是原地打圈
    WorkBenchNodeForRobot default_node =  WorkBenchNodeForRobot(-2, 1, this->location_x, this->location_y, 0.5, 0, 0, INT_MAX);
    //初始化该机器人所有工作台的候补队列 以及 高优先度生产好的点
    int cnt = 0; //高优先度点的个数
    std::vector<std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule>>robot_target_queue;
    std::vector<std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule>>greater_level_queue;
    for(int i = 0; i <= 9; ++i) {
        //机器人面对1-9类型工作台的参数
        auto &vec = workbench_for_robot[i];
        std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule> pq;
        std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule> pq2; //分别对应上面两个有限队列数组的临时队列
        for(auto &v: vec) {
            pq.push(v);
            if(i >= 4 && v.product_status == 1) {
                //只有当小车距离7的距离小于2的时候，才会来取7，防止出现空车跑来取7的情况
                if(i == 7 && v.dis > 2) continue; 
                pq2.push(v);
                ++cnt;
            }
        }
        robot_target_queue.push_back(pq);
        greater_level_queue.push_back(pq2);
    }
    WorkBenchNodeForRobot ans(-2, 1, this->location_x, this->location_y, 0.5, 0, 0, INT_MAX);
    if(cnt != 0) ans =  Num89(robotID, default_node, robot_target_queue, greater_level_queue);
    if(cnt != 0 && robot_goal_point.empty()) ans =  Num789(robotID, default_node, robot_target_queue, greater_level_queue);
    if(robot_goal_point.empty()) ans =  Num456(robotID, default_node, robot_target_queue);
    
    return ans;
}

WorkBenchNodeForRobot Robot::GetTarget2(int robotID) {
    //默认的点--默认工作台类型为1,全局ID为-2---就是原地打圈
    WorkBenchNodeForRobot default_node =  WorkBenchNodeForRobot(-2, 1, this->location_x, this->location_y, 0.5, 0, 0, INT_MAX);
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
                if(v.dis > 2) continue; 
                pq2.push(v);
                ++cnt;
            }
        }
        robot_target_queue.push_back(pq);
        greater_level_queue.push_back(pq2);
    }
    WorkBenchNodeForRobot ans(-2, 1, this->location_x, this->location_y, 0.5, 0, 0, INT_MAX);
    if(cnt != 0) ans =  Num89(robotID, default_node, robot_target_queue, greater_level_queue);
    if(cnt != 0 && robot_goal_point.empty()) ans =  Num789(robotID, default_node, robot_target_queue, greater_level_queue);
    if(robot_goal_point.empty()) ans =  Num456(robotID, default_node, robot_target_queue);
    return ans;
}

WorkBenchNodeForRobot Robot::GetTarget3(int robotID) {
    //默认的点--默认工作台类型为1,全局ID为-2---就是原地打圈
    WorkBenchNodeForRobot default_node =  WorkBenchNodeForRobot(-2, 1, this->location_x, this->location_y, 0.5, 0, 0, INT_MAX);
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
                if(v.dis > 2) continue;
                pq2.push(v);
                ++cnt;
            }
        }
        robot_target_queue.push_back(pq);
        greater_level_queue.push_back(pq2);
    }
    WorkBenchNodeForRobot ans(-2, 1, this->location_x, this->location_y, 0.5, 0, 0, INT_MAX);
    if(cnt != 0) ans =  Num89(robotID, default_node, robot_target_queue, greater_level_queue);
    if(cnt != 0 && robot_goal_point.empty()) ans =  Num789(robotID, default_node, robot_target_queue, greater_level_queue);
    if(robot_goal_point.empty()) ans =  Num456(robotID, default_node, robot_target_queue);
    
    return ans;
}

WorkBenchNodeForRobot Robot::GetTarget4(int robotID) {
    //默认的点--默认工作台类型为1,全局ID为-2---就是原地打圈
    WorkBenchNodeForRobot default_node =  WorkBenchNodeForRobot(-2, 1, this->location_x, this->location_y, 0.5, 0, 0, INT_MAX);
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
                if(v.dis > 2) continue;
                pq2.push(v);
                ++cnt;
            }
        }
        robot_target_queue.push_back(pq);
        greater_level_queue.push_back(pq2);
    }
    WorkBenchNodeForRobot ans(-2, 1, this->location_x, this->location_y, 0.5, 0, 0, INT_MAX);
    if(cnt != 0) ans =  Num89(robotID, default_node, robot_target_queue, greater_level_queue);
    if(cnt != 0) ans =  Num789(robotID, default_node, robot_target_queue, greater_level_queue);
    if(robot_goal_point.empty()) ans =  Num456(robotID, default_node, robot_target_queue);
    
    return ans;
}


WorkBenchNodeForRobot Robot::Num789(int robotID, WorkBenchNodeForRobot &default_node, std::vector<std::priority_queue<WorkBenchNodeForRobot, 
        std::vector<WorkBenchNodeForRobot>, cmp_rule>> robot_target_queue, std::vector<std::priority_queue<WorkBenchNodeForRobot, 
        std::vector<WorkBenchNodeForRobot>, cmp_rule>> greater_level_queue
) {
    //具体的策略
    //将4-7放在总的候选队列里
    std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule> target456_queue;
    for(int i = 4; i <= 6; ++i) {
        while(!greater_level_queue[i].empty()) {
            target456_queue.push(greater_level_queue[i].top());
            greater_level_queue[i].pop();
        }
    }
    //随后把456能卖的工作台整体加入一个候选队列，这样是为了同一类型的工作台遍历方便
    std::vector<std::vector<WorkBenchNodeForRobot>> target_for_sell{{},{},{},{},{},{},{},{},{},{}};
    for(int i = 7; i <= 9; ++i) {
        while(!robot_target_queue[i].empty()) {
            for(auto &&j: WorkBenchIdForSell[i]) {
                target_for_sell[j].push_back(robot_target_queue[i].top());
            }
            robot_target_queue[i].pop();  
        }
    }
    //然后先从大候选队列里面找到一个可以的买和卖的目标点
    while(!target456_queue.empty()) {
        auto m = target456_queue.top();
        target456_queue.pop();
        double coordinate_x = m.x;
        double coordinate_y = m.y;
        int type = m.type;
        //买的点不准重复
        if(target_set.count({coordinate_x*100+coordinate_y, type})) continue;
        //找到这个买的点能卖的离着最近的点（这个卖的候选，肯定要和买的位置想联动,直接遍历找那个离着最近的点）
        double min_dis = DBL_MAX;
        int time = INT_MAX;
        WorkBenchNodeForRobot ans_for_sell;
        //先找能买的工作台，然后将其挨个加入优先队列
        for(auto &workbench: target_for_sell[type]) {
            double workbench_location_x = workbench.x;
            double workbench_location_y = workbench.y;
            int p_time = workbench.remain_production_time;
            //卖的能卖且不可以被重复选择
            if(workbench.bag.count(type) || target_set.count({workbench_location_x*100+workbench_location_y, type})) continue;
            double distance = sqrt(pow(coordinate_x - workbench_location_x,2)+pow(coordinate_y - workbench_location_y,2));
            if(time > p_time) {
                time = p_time;
                min_dis = distance;
                ans_for_sell = workbench;
                continue;
            }
            if(distance > min_dis) continue;
            time = p_time;
            min_dis = distance;
            ans_for_sell = workbench;
        }
        //是否找到能卖并且离着买最近的工作台？
        if(min_dis == DBL_MAX) continue;
        if(ans_for_sell.type != 9) {
            ans_for_sell.type = type;
            target_set.insert({ans_for_sell.x*100+ans_for_sell.y, type});
        }
        robot_goal_point.push(ans_for_sell);
        robot_goal_point.push(m);
        target_set.insert({coordinate_x*100+coordinate_y, type});
        //std::cerr << ans_for_sell.type << "&" << m.type << std::endl;
        break;
    } 
    return default_node;
}


WorkBenchNodeForRobot Robot::Num89(int robotID, WorkBenchNodeForRobot &default_node, std::vector<std::priority_queue<WorkBenchNodeForRobot, 
        std::vector<WorkBenchNodeForRobot>, cmp_rule>> robot_target_queue, std::vector<std::priority_queue<WorkBenchNodeForRobot, 
        std::vector<WorkBenchNodeForRobot>, cmp_rule>> greater_level_queue
) {
    //具体的策略
    //将4-7放在总的候选队列里
    std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule> target7_queue;
    for(int i = 7; i <= 7; ++i) {
        while(!greater_level_queue[i].empty()) {
            target7_queue.push(greater_level_queue[i].top());
            greater_level_queue[i].pop();
        }
    }
    //随后把4567能卖的工作台整体加入一个候选队列，这样是为了同一类型的工作台遍历方便
    std::vector<std::vector<WorkBenchNodeForRobot>> target_for_sell{{},{},{},{},{},{},{},{},{},{}};
    for(int i = 7; i <= 9; ++i) {
        while(!robot_target_queue[i].empty()) {
            for(auto &&j: WorkBenchIdForSell[i]) {
                target_for_sell[j].push_back(robot_target_queue[i].top());
            }
            robot_target_queue[i].pop();  
        }
    }
    //然后先从大候选队列里面找到一个可以的买和卖的目标点
    while(!target7_queue.empty()) {
        auto m = target7_queue.top();
        target7_queue.pop();
        double coordinate_x = m.x;
        double coordinate_y = m.y;
        int type = m.type;
        //买的点不准重复
        if(target_set.count({coordinate_x*100+coordinate_y, type})) continue;
        //找到这个买的点能卖的离着最近的点（这个卖的候选，肯定要和买的位置想联动,直接遍历找那个离着最近的点）
        double min_dis = DBL_MAX;
        int time = INT_MAX;
        WorkBenchNodeForRobot ans_for_sell;
        //先找能买的工作台，然后将其挨个加入优先队列
        for(auto &workbench: target_for_sell[type]) {
            double workbench_location_x = workbench.x;
            double workbench_location_y = workbench.y;
            int p_time = workbench.remain_production_time;
            //卖的能卖且不可以被重复选择
            if(workbench.bag.count(type) || target_set.count({workbench_location_x*100+workbench_location_y, type})) continue;
            double distance = sqrt(pow(coordinate_x - workbench_location_x,2)+pow(coordinate_y - workbench_location_y,2));

            if(distance > min_dis) continue;
            time = p_time;
            min_dis = distance;
            ans_for_sell = workbench;
        }
        //是否找到能卖并且离着买最近的工作台？
        if(min_dis == DBL_MAX) continue;
        ans_for_sell.type = type;
        robot_goal_point.push(ans_for_sell);
        target_set.insert({ans_for_sell.x*100+ans_for_sell.y, type});
        robot_goal_point.push(m);
        target_set.insert({coordinate_x*100+coordinate_y, type});
        //std::cerr << ans_for_sell.type << "&" << m.type << std::endl;
        break;
    } 
    return default_node;
}


WorkBenchNodeForRobot Robot::Num123(int robotID, WorkBenchNodeForRobot &default_node, std::vector<std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule>> robot_target_queue) { 
    //具体的策略
    //先去将123都放在优先队列里
    std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule> target123_queue;
    for(int i = 1; i <= 3; ++i) {
        while(!robot_target_queue[i].empty()) {
            target123_queue.push(robot_target_queue[i].top());
            robot_target_queue[i].pop();
        }
    }
    //随后把123能卖的工作台整体加入一个候选队列，这样是为了同一类型的工作台遍历方便
    std::vector<std::vector<WorkBenchNodeForRobot>> target_for_sell{{},{},{},{}};
    for(int i = 4; i <= 6; ++i) {
        while(!robot_target_queue[i].empty()) {
            for(auto &&j: WorkBenchIdForSell[i]) {
                target_for_sell[j].push_back(robot_target_queue[i].top());
            }
            robot_target_queue[i].pop();  
        }
    }
    //然后先从大候选队列里面找到一个可以的买和卖的目标点
    while(!target123_queue.empty()) {
        auto m = target123_queue.top();
        target123_queue.pop();
        double coordinate_x = m.x;
        double coordinate_y = m.y;
        int type = m.type;
        //买的点不准重复
        if(target_set.count({coordinate_x*100+coordinate_y, type})) continue;
        //找到这个买的点能卖的离着最近的点（这个卖的候选，肯定要和买的位置想联动,直接遍历找那个离着最近的点）
        double min_dis = DBL_MAX;
        WorkBenchNodeForRobot ans_for_sell;
        //先找能买的工作台，然后将其挨个加入优先队列
        for(auto &workbench: target_for_sell[type]) {
            double workbench_location_x = workbench.x;
            double workbench_location_y = workbench.y;
            //卖的能卖且不可以被重复选择
            if(workbench.bag.count(type) || target_set.count({workbench_location_x*100+workbench_location_y, type})) continue;
            double distance = sqrt(pow(coordinate_x - workbench_location_x,2)+pow(coordinate_y - workbench_location_y,2));
            if(distance > min_dis) continue;
            min_dis = distance;
            ans_for_sell = workbench;
        }
        //是否找到能卖并且离着买最近的工作台？
        if(min_dis == DBL_MAX) continue;
        ans_for_sell.type = type;
        robot_goal_point.push(ans_for_sell);
        target_set.insert({ans_for_sell.x*100+ans_for_sell.y, type});
        robot_goal_point.push(m);
        target_set.insert({coordinate_x*100+coordinate_y, type});
        //std::cerr << ans_for_sell.type << "&" << m.type << std::endl;
        break;
    } 
    return default_node;
}



WorkBenchNodeForRobot Robot::Num456(int robotID, WorkBenchNodeForRobot &default_node, std::vector<std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule>> robot_target_queue) { 
     //具体的策略
    //先去将123都放在优先队列里
    std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule> target123_queue;
    for(int i = 1; i <= 3; ++i) {
        while(!robot_target_queue[i].empty()) {
            target123_queue.push(robot_target_queue[i].top());
            robot_target_queue[i].pop();
        }
    }
    //随后把123能卖的工作台整体加入一个候选队列，这样是为了同一类型的工作台遍历方便
    //但是候选队列只能选当前的循环里面
    int idx = 0;//当前应该选哪个了？

    for(int k = 0 ;k <= 2;++k)
    {
        std::priority_queue<WorkBenchNodeForRobot, std::vector<WorkBenchNodeForRobot>, cmp_rule> target123_queue_tem(target123_queue);
        int num_circular_array = CircularArray.size();
        idx = CircularArray[(k + CircularArrayPtr + num_circular_array) % num_circular_array];
        std::vector<std::vector<WorkBenchNodeForRobot>> target_for_sell{{},{},{},{}};
        while(!robot_target_queue[idx].empty()) {
            for(auto &&j: WorkBenchIdForSell[idx]) {
                target_for_sell[j].push_back(robot_target_queue[idx].top());
            }
            robot_target_queue[idx].pop();  
        }
        //std::cerr<<target123_queue_tem.size()<<std::endl;
        //然后先从大候选队列里面找到一个可以的买和卖的目标点
        while(!target123_queue_tem.empty()) {
            auto m = target123_queue_tem.top();
            target123_queue_tem.pop();
            double coordinate_x = m.x;
            double coordinate_y = m.y;
            int type = m.type;
            //买的点不准重复
            if(target_set.count({coordinate_x*100+coordinate_y, type})) continue;
            //找到这个买的点能卖的离着最近的点（这个卖的候选，肯定要和买的位置相联动,直接遍历找那个离着最近的点）
            double min_dis = DBL_MAX;
            WorkBenchNodeForRobot ans_for_sell;
            //先找能买的工作台，然后将其挨个加入优先队列
            for(auto &workbench: target_for_sell[type]) {
                double workbench_location_x = workbench.x;
                double workbench_location_y = workbench.y;
                //卖的能卖且不可以被重复选择
                if(workbench.bag.count(type) || target_set.count({workbench_location_x*100+workbench_location_y, type})) continue;
                double distance = sqrt(pow(coordinate_x - workbench_location_x,2)+pow(coordinate_y - workbench_location_y,2));
                if(distance > min_dis) continue;
                min_dis = distance;
                ans_for_sell = workbench;
            }
            //是否找到能卖并且离着买最近的工作台？
            if(min_dis == DBL_MAX) continue;
            //同时修改循环数组的指针
            CircularArrayPtr++;
            if(CircularArrayPtr >= CircularArray.size()) CircularArrayPtr = 0;

            ans_for_sell.type = type;
            robot_goal_point.push(ans_for_sell);
            target_set.insert({ans_for_sell.x*100+ans_for_sell.y, type});
            robot_goal_point.push(m);
            target_set.insert({coordinate_x*100+coordinate_y, type});
            //std::cerr << ans_for_sell.type << "&" << m.type << std::endl;
            break;
        }
        if(robot_goal_point.size() == 2) break;
    } 
    return default_node;
}



bool Robot::astar(struct sNode *nodeStart, struct sNode *nodeEnd, bool carry_object)
{	
    all_node_object = fixed_all_node_object;
    //else all_node_empty = fixed_all_node_empty;
    auto distance = [](sNode* a, sNode* b) // For convenience
	{
        //return abs((a->x - b->x))+abs((a->y - b->y));
		return sqrtf((a->coordinate_x - b->coordinate_x)*(a->coordinate_x - b->coordinate_x) + (a->coordinate_y - b->coordinate_y)*(a->coordinate_y - b->coordinate_y));
	};

    auto heuristic = [distance](sNode* a, sNode* b) // So we can experiment with heuristic
	{
		return distance(a, b);
	};


    sNode *nodeCurrent = nodeStart;
	nodeStart->fLocalGoal = 0.0f;
	nodeStart->fGlobalGoal = heuristic(nodeStart, nodeEnd);

	std::list<sNode*> listNotTestedNodes;
	listNotTestedNodes.push_back(nodeStart);

	while(!listNotTestedNodes.empty() && nodeCurrent != nodeEnd)
	{
		//排序
		listNotTestedNodes.sort([](const sNode* lhs, const sNode* rhs){ return lhs->fGlobalGoal < rhs->fGlobalGoal; } );

		//pop被check过的
		while(!listNotTestedNodes.empty() && listNotTestedNodes.front()->bVisited)
				listNotTestedNodes.pop_front();

        if (listNotTestedNodes.empty())
				break;

		//取代价最小
		nodeCurrent = listNotTestedNodes.front();
		nodeCurrent->bVisited = true;
        
        

		//遍历neighbor
		for (auto nodeNeighbour : nodeCurrent->vecNeighbours)
		{
            //std::cerr << "nodeNeighbour is "<< nodeNeighbour->x<< ","<< nodeNeighbour->y<< std::endl;
			//如果没被遍历且不是墙 加入待测
			if (!nodeNeighbour->bVisited && nodeNeighbour->bObstacle == 0)
			{
				listNotTestedNodes.push_back(nodeNeighbour);
			}

			//计算当前已耗费+估计还会耗费
			double fPossiblyLowerGoal = nodeCurrent->fLocalGoal + nodeNeighbour -> node_cost;// * distance(nodeCurrent, nodeNeighbour);

			//替换s
            //std::cerr << "fPossiblyLowerGoal"<< fPossiblyLowerGoal << std::endl;
            //std::cerr << "nodeNeighbour->fLocalGoal"<< nodeNeighbour->fLocalGoal << std::endl;
			if (fPossiblyLowerGoal < nodeNeighbour->fLocalGoal)
			{
				nodeNeighbour->parent = nodeCurrent;
                //std::cerr << "nodeCurrent is "<< nodeCurrent->x<< ","<< nodeCurrent->y<< std::endl;
				nodeNeighbour->fLocalGoal = fPossiblyLowerGoal;

				nodeNeighbour->fGlobalGoal = nodeNeighbour->fLocalGoal + heuristic(nodeNeighbour, nodeEnd);
			}
		}
	}

    WorkBenchNodeForRobot point;
    WorkBenchNodeForRobot pre_point;
    WorkBenchNodeForRobot next_point;
    //std::ofstream outfile("./log.txt",std::ios::out);
    //std::ofstream outfile2("./log_del.txt",std::ios::out);
    if (nodeEnd != nullptr)
	{
	    sNode *p = nodeEnd;
        point.x = p -> coordinate_x;
        point.y = p -> coordinate_y;
        //std::cerr<<point.x<<"  "<<point.y<<std::endl;
        robot_execute_points.push(point);
        //outfile<<point.x<<"  "<<point.y<<std::endl;
        //outfile2<<point.x<<"  "<<point.y<<std::endl;
		while (p->parent != nullptr && p -> parent -> parent != nullptr)
		{
            std::string str;
            //pre_pont存放上一个点
            pre_point.x = p -> coordinate_x;
            pre_point.y = p -> coordinate_y;
            std::pair<double,double> point1 = {pre_point.x, pre_point.y};

            p = p -> parent;
            ////point存放当前点
            point.x = p -> coordinate_x;
            point.y = p -> coordinate_y;
            //outfile<<point.x<<"  "<<point.y<<std::endl;
            std::pair<double,double> point2 = {point.x, point.y};
            //next_pont存放下一个点
            next_point.x = p -> parent -> coordinate_x;
            next_point.y = p -> parent -> coordinate_y;
            std::pair<double,double> point3 = {next_point.x, next_point.y};

            if(remove_middle_points(point1, point2, point3) == true) {
                //outfile2<<point.x<<"  "<<point.y<<std::endl;
                robot_execute_points.push(point);
            }
		}
        /*if(p -> parent != nullptr){
            point.x = p -> parent -> coordinate_x;
            point.y = p -> parent -> coordinate_y;
            robot_execute_points.push(point);
            outfile2<<point.x<<"  "<<point.y<<std::endl;
            outfile<<point.x<<"  "<<point.y<<std::endl;
        }   */
    }
    return true;
}



bool Robot::remove_middle_points(std::pair<double,double>& point1, std::pair<double,double>& point2, std::pair<double,double>& point3) {
    double x1 = point1.first;
    double y1 = point1.second;
    double x2 = point2.first;
    double y2 = point2.second;
    double x3 = point3.first;
    double y3 = point3.second;

    auto distance = [] (double x1, double y1, double x2, double y2){
        return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    };
    
    double d1 = distance(x1, y1, x2, y2);
    double d2 = distance(x2, y2, x3, y3);
    double d3 = distance(x1, y1, x3, y3);
    if (d1 + d2 == d3) return false;
    //std::cerr<<d1<<" "<<d2<<"  "<<d3<<std::endl;
    return true;
}