#include "hwcompet.h"

void hw_compet::get_yaw_angle(double& distance_target,double& yaw ,double& vector_angle ,double workbench_x ,double workbench_y ,const Robot& robot_cluster_get_angle)
{
    //得到小车的朝向
    yaw = robot_cluster_get_angle.direction;
    double car_x = robot_cluster_get_angle.location_x;
    double car_y = robot_cluster_get_angle.location_y;

	double delta_x = workbench_x - car_x;
	double delta_y = workbench_y - car_y;
    distance_target = sqrt(pow(delta_x,2)+pow(delta_y,2)); 
	vector<double> vec1 = { delta_x,delta_y };
	vector<double> vec2 = { 1,0 };

	double dot_product = 0.0;
	for (size_t i = 0; i < 2; i++) {
		dot_product += vec1[i] * vec2[i];
	}

	// 计算两个向量的长度
	double vec1_length = 0.0;
	double vec2_length = 0.0;
	for (size_t i = 0; i < 2; i++) {
		vec1_length += vec1[i] * vec1[i];
		vec2_length += vec2[i] * vec2[i];
	}
	vec1_length = sqrt(vec1_length);
	vec2_length = sqrt(vec2_length);

	// 计算两个向量之间的夹角
	vector_angle = acos(dot_product / (vec1_length * vec2_length));
	if (delta_y < 0)
		vector_angle = -vector_angle;
}

//更新单独一个机器人到各个工作台的距离(为了便于多线程)
//也可以修改成直接求解四个机器到工作台的距离
double hw_compet::update_distance(const WorkBenchNode& workbench,const Robot& robot)
{

    double car_location_x = robot.location_x;
    double car_location_y = robot.location_y;
    double workbench_location_x = workbench.x;
    double workbench_location_y = workbench.y;
    double distance=sqrt(pow(car_location_x-workbench_location_x,2)+pow(car_location_y-workbench_location_y,2));
	return distance;
}


//速度计算
void hw_compet::vel_cmd_out(pid_controller **pid, double &aS, double &lS, double yaw, double angle, double distance, int map_id)
{
		double err = yaw - angle;

		aS = (pid_update(*pid, pid_param[map_id][0], yaw - angle));
        err = yaw - angle;
        if(err < -3.15)
        {
            err = (2*3.1415926+err);
        }
        if(err > 3.15)
        {
            err =-(2*3.1415926-err);
        }
		if(abs(err) < pid_param[map_id][1])
		{
			lS = 6;
			if(distance<pid_param[map_id][3])
			{
				lS = pid_param[map_id][4];
			}
		}
		else
			lS = pid_param[map_id][5];
        //std::cerr<<lS<<std::endl;
		//lS = 1/abs(aS)+0.3;
		//lS = 0;
}
//pid初始化
void hw_compet::pid_init(pid_controller *pid, int map_id)
{
    
	pid->integrator = 0.0f;
	pid->differentiator = 0.0f;
	pid->pre_err = 0.0f;
	pid->pre_measure = 0.0f;
	pid->out = 0.0f;
	pid->T = 0.020;
	pid->lim_min = -3.1415926;
	pid->lim_max = 3.1415926;
	pid->Kp = pid_param[map_id][6];
	pid->Ki = pid_param[map_id][7];
	pid->Kd = pid_param[map_id][8];
	pid->tau = pid_param[map_id][9];
}

double hw_compet::pid_update(pid_controller *pid, double setpoint, double measure)
{
	//Error
	double err = setpoint - measure;
    if(err < -3.15)
	{
		err = (2*3.1415926+err);
	}
	if(err > 3.15)
	{
		err =-(2*3.1415926-err);
	}

	
	//P
	double proportional = pid->Kp * err;

	//I
	pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (err+pid->pre_err);

	double limMinInt, limMaxInt;

	if(pid->lim_max > proportional)
	{
		limMaxInt = pid->lim_max - proportional;
	}
	else
	{
		limMaxInt = 0.0f;
	}

	if(pid->lim_min < proportional)
	{
		limMinInt = pid->lim_min - proportional;
	}
	else
	{
		limMinInt = 0.0f;
	}

	if(pid->integrator > limMaxInt)
	{
		pid->integrator = limMaxInt;
	}
	else if(pid->integrator < limMinInt)
	{
		pid->integrator = limMinInt;
	}

	//D
	pid->differentiator = (2.0f * pid->Kd * (measure-pid->pre_measure)
											+	(2.0f * pid->tau - pid->T) * pid->differentiator)
											/ (2.0f * pid->tau + pid->T);

	//SUM
	pid->out = proportional + pid->integrator + pid->differentiator;
	
	if(pid->out > pid->lim_max)
	{
		pid->out = pid->lim_max;
	}
	else if(pid->out < pid->lim_min)
	{
		pid->out = pid->lim_min;
	}

	pid->pre_err = err;
	pid->pre_measure = measure;

	return pid->out;
}



//与判题器进行信息交互
bool hw_compet::readUntilOK() {
	char line[1024];
	int workbenchId = 1;
	int robotId = 0;
	int flag_event = 1;
    std::vector<std::vector<int>> pre_sum(4,vector<int>(51,0));
	std::vector<int> del_sum(4,0);
	while (fgets(line, sizeof(line), stdin))
	{
		if (line[0] == 'O' && line[1] == 'K') {
			return true;
		}
		else
		{
			vector<string> data_each_frame;
			string input(line);
			std::stringstream ss(input);
			string one_str;
			while (std::getline(ss, one_str, ' ')) 
			{
				data_each_frame.push_back(one_str);
			}
			switch (flag_event) {
				case 1:
                {
                    initial_money = stoi(data_each_frame[1]);
                    break;
                }
                case 2:
                {
                    workbench_num = stoi(data_each_frame[0]);
                    break;
                }
                case 3:
                {
                    while (workbenchId != workbench_num)
                    {
                        flag_event--;
                        break;
                    }
					//得到横纵坐标
					double workbench_row = stod(data_each_frame[1]);
                    double workbench_col = stod(data_each_frame[2]);
					for(int i = 0; i < 4; i++)
					{
						//不在删除的数组里面
						if(del_workbench_map[map_id][i].count(workbenchId) == 0)
                    	{
                        	pre_sum[i][workbenchId] = del_sum[i];
                        	//工作台的编号1--9
                        	work_bench_cluster[i][stoi(data_each_frame[0])].Update(workbench_row,workbench_col,stoi(data_each_frame[3]),stoi(data_each_frame[4]),stoi(data_each_frame[5])); 
                    	}
                    	else
                    	{
							//std::cerr<<"del workbench "<<workbenchId<<"    "<<workbench_row<<"      "<<workbench_col<<std::endl;
                        	del_sum[i]++;
							pre_sum[i][workbenchId] = 53;
                    	}
					}
                    workbenchId++;
                    break;
                }
                case 4:
                {
				    flag_event--;
                    int workstation_id = stoi(data_each_frame[0]);
					if(workstation_id != -1) workstation_id -= pre_sum[robotId][workstation_id + 1];	
					if(workstation_id < -1) workstation_id = -1;					
                    robot_cluster[robotId].Update(workstation_id, stoi(data_each_frame[1]), stod(data_each_frame[2]), stod(data_each_frame[3]), stod(data_each_frame[4]), stod(data_each_frame[5]), stod(data_each_frame[6]), stod(data_each_frame[7]), stod(data_each_frame[8]), stod(data_each_frame[9]));
                    robotId++;
                    break;
                }
                default:
                    break;
            }
            flag_event++;
		}
	}
	return false;
}



//读取地图
bool hw_compet::init() {
	char line[1024];
	//地图行 i表示坐标y
	//循环中j表示坐标x
	int i = 99;
    std::vector<int> cnt(4,0);
    int wait_del_workbench = 1;
	struct workbench *wb;
	struct workbench *wb_tmp;
	int robot_nums = 0;
	int cnt_cx = 0;
	std::vector<std::vector<int>> wb_to_wb = {
		{},
		{4,5,9},
		{4,6,9},
		{5,6,9},
		{7,9},
		{7,9},
		{7,9},
		{8,9},
		{},
		{}
	};
	
	//先读取地图
	while (fgets(line, sizeof line, stdin)) {
		if (line[0] == 'O' && line[1] == 'K') {
			for(int robotId = 0; robotId < 4; robotId++){
				for(int i=0; i<10000; i++)
				{
					if(robot_cluster[robotId].all_node_empty[i].bObstacle == true && robot_cluster[robotId].all_node_empty[i].symbol == '#' 
					&& robot_cluster[robotId].all_node_empty[i+100].bObstacle == true && robot_cluster[robotId].all_node_empty[i+100].symbol == '.' 
					&& robot_cluster[robotId].all_node_empty[i+200].bObstacle == true && robot_cluster[robotId].all_node_empty[i+200].symbol == '.' 
					&& robot_cluster[robotId].all_node_empty[i+300].bObstacle == true && robot_cluster[robotId].all_node_empty[i+300].symbol == '#')
					{
						//std::cerr<<"***************"<<std::endl;
						robot_cluster[robotId].all_node_empty[i+100].bObstacle  = false;
						robot_cluster[robotId].all_node_empty[i+200].bObstacle = false; 
						robot_cluster[robotId].all_node_empty[i+100].coordinate_x = (robot_cluster[robotId].all_node_empty[i+100].coordinate_x + robot_cluster[robotId].all_node_empty[i+200].coordinate_x)/2;
						robot_cluster[robotId].all_node_empty[i+200].coordinate_x = (robot_cluster[robotId].all_node_empty[i+100].coordinate_x + robot_cluster[robotId].all_node_empty[i+200].coordinate_x)/2;
						robot_cluster[robotId].all_node_empty[i+100].coordinate_y = (robot_cluster[robotId].all_node_empty[i+100].coordinate_y + robot_cluster[robotId].all_node_empty[i+200].coordinate_y)/2;
						robot_cluster[robotId].all_node_empty[i+200].coordinate_y = (robot_cluster[robotId].all_node_empty[i+100].coordinate_y + robot_cluster[robotId].all_node_empty[i+200].coordinate_y)/2;

					}


					if(robot_cluster[robotId].all_node_empty[i].bObstacle == true && robot_cluster[robotId].all_node_empty[i].symbol == '#' 
					&& robot_cluster[robotId].all_node_empty[i-1].bObstacle == true && robot_cluster[robotId].all_node_empty[i-1].symbol == '.' 
					&& robot_cluster[robotId].all_node_empty[i-2].bObstacle == true && robot_cluster[robotId].all_node_empty[i-2].symbol == '.' 
					&& robot_cluster[robotId].all_node_empty[i-3].bObstacle == true && robot_cluster[robotId].all_node_empty[i-3].symbol == '#')
					{
						robot_cluster[robotId].all_node_empty[i-1].bObstacle = false;
					    robot_cluster[robotId].all_node_empty[i-2].bObstacle = false;
						robot_cluster[robotId].all_node_empty[i-1].coordinate_x = (robot_cluster[robotId].all_node_empty[i-1].coordinate_x + robot_cluster[robotId].all_node_empty[i-2].coordinate_x)/2;
						robot_cluster[robotId].all_node_empty[i-2].coordinate_x = (robot_cluster[robotId].all_node_empty[i-1].coordinate_x + robot_cluster[robotId].all_node_empty[i-2].coordinate_x)/2;
						robot_cluster[robotId].all_node_empty[i-1].coordinate_y = (robot_cluster[robotId].all_node_empty[i-1].coordinate_y + robot_cluster[robotId].all_node_empty[i-2].coordinate_y)/2;
						robot_cluster[robotId].all_node_empty[i-2].coordinate_y = (robot_cluster[robotId].all_node_empty[i-1].coordinate_y + robot_cluster[robotId].all_node_empty[i-2].coordinate_y)/2;

					}
				}
				robot_cluster[robotId].fixed_all_node_empty = robot_cluster[robotId].all_node_empty;
				robot_cluster[robotId].fixed_all_node_object = robot_cluster[robotId].all_node_object;
			}

			int cnt = 0;
			for(int n=9999; n>=0;n--)
			{
				
				if(cnt == 100)
				{
					cnt = 0;
					std::cerr << " "<< std::endl;
				}
				if(robot_cluster[0].all_node_empty[n].bObstacle==true)
					std::cerr << "#";
				else std::cerr << robot_cluster[0].all_node_object[n].symbol;
				cnt++;
			}
/*
			//遍历机器人
			for(int robotId = 0; robotId < 1; robotId++){
				//遍历1到9号工作台
				for(int wb_num = 1; wb_num < work_bench_cluster[robotId].size(); wb_num++){
					//std::cerr<<"task numbers is "<< work_bench_cluster[robotId].size() <<std::endl;
					//遍历每一种类型工作台中的具体每一个
					for(int m = 0; m < work_bench_cluster[robotId][wb_num].WorkBenchVec.size(); m++){
					//for(auto &m : work_bench_cluster[robotId][wb_num].WorkBenchVec){
						//1号工作台坐标
						int row_start = (int)((work_bench_cluster[robotId][wb_num].WorkBenchVec[m].y + 0.25)/0.5 - 1);
                		int col_start = (int)((work_bench_cluster[robotId][wb_num].WorkBenchVec[m].x + 0.25)/0.5 - 1);
						//std::cerr<<m.x<<"   "<<m.y<<std::endl;
						int start_node_num = row_start * 100 + col_start;
						//遍历wb_to_wb,得到一种类型的工作台，可以作为哪些高等级工作台的原料
						for(auto &n : wb_to_wb[wb_num]){
							//遍历所有的高等级工作台，并绑定A*任务，等待线程池执行
							//for(int t = 0; t < )
							for(auto &t : work_bench_cluster[robotId][n].WorkBenchVec){
								if(t.x == 0 && t.y == 0) continue;
								int row_end = (int)((t.y + 0.25)/0.5 - 1);
                				int col_end = (int)((t.x + 0.25)/0.5 - 1);
								int end_node_num = row_end * 100 + col_end;
								int global_id = t.global_id;
								//绑定A*任务
								//std::cerr<<"x = "<<t.x<<"  y = "<<t.y<<std::endl;
								auto task = std::bind(&hw_compet::astar_init_using, this, &robot_cluster[robotId].all_node_object[start_node_num], &robot_cluster[robotId].all_node_object[end_node_num], robotId, global_id, t.type, &work_bench_cluster[robotId][wb_num].WorkBenchVec[m], &t);
								deal_planning_route_pool.AddTask(task);
							}
						}
					}
				}
			}
	*/		
			for(int robotId = 0; robotId < 1;robotId++){
				int cnt_tmp=0;
				robot_cluster[robotId].node_tmp_object[0].x = (robot_cluster[robotId].location_x - 0.25)/0.5;
				robot_cluster[robotId].node_tmp_object[0].y = (robot_cluster[robotId].location_y - 0.25)/0.5;
				int start_node_num = robot_cluster[robotId].node_tmp_object[0].y * 100 + robot_cluster[robotId].node_tmp_object[0].x;
				for(auto &workbench_tmp : usable_workbench_for_robot[robotId]){
					cnt_tmp++;
					robot_cluster[robotId].node_tmp_object[1].x = workbench_tmp.x;
					robot_cluster[robotId].node_tmp_object[1].y = workbench_tmp.y;
					int end_node_num = robot_cluster[robotId].node_tmp_object[1].y * 100 + robot_cluster[robotId].node_tmp_object[1].x;
					int point_nums = astar(&robot_cluster[robotId].all_node_object[end_node_num], &robot_cluster[robotId].all_node_object[start_node_num], robotId, 1);
					if(point_nums <= 1){
						if(workbench_tmp.type == 1 || workbench_tmp.type == 2 || workbench_tmp.type == 3){
							Robot::target_set.insert({workbench_tmp.coordinate_x * 100 + workbench_tmp.coordinate_y, workbench_tmp.type});
						}
						else{
							for(auto &h : WorkBenchIdForSell[workbench_tmp.type]){
								Robot::target_set.insert({workbench_tmp.coordinate_x * 100 + workbench_tmp.coordinate_y, h});
							}
						}
					}
					std::vector<WorkBenchNodeForRobot> st_tmp_empty;
					robot_cluster[robotId].robot_execute_points = st_tmp_empty;
				}
			}		



			//for(auto &m : del_workbench_map[0][0]){
			//	std::cerr<<"del global_id "<<m<<std::endl;
			//}
			
			//同时进行pid的初始化
			for(int n=0; n<4; n++)
			{
				control_ptr[n] = &controller[n];
				pid_init(control_ptr[n], map_id);
			}
			return true;
		}
		else
		{
			std::string input(line);
			for(int j = 0; j <= 99; ++j)
			{
				if (input[j] == 'A')
				{
					//更新机器人位置
					robot_cluster[robot_nums].location_x = j*0.5+0.25;
					robot_cluster[robot_nums].location_y = i*0.5+0.25;
					//std::cerr<<robot_cluster[robot_nums].location_x<<"  "<<j<<"    "<<robot_cluster[robot_nums].location_y<<"    "<<i<<std::endl;
					robot_nums++;	
				}
				else if (input[j] >= '1' && input[j] <= '9')
				{
					int type = input[j] - '0';
					usable_workbench workbench_tmp;
					//判断地图的id
					check_map_id(input, i, j, type);
					for(int del_robot_wb = 0; del_robot_wb < 4; del_robot_wb++)
					{
						if(del_workbench_map[map_id][del_robot_wb].count(wait_del_workbench) == 0)
                    	{
							workbench_tmp.global_id = cnt[del_robot_wb];
							workbench_tmp.coordinate_x = (double)j*0.5+0.25;
							workbench_tmp.coordinate_y = (double)i*0.5+0.25;
							workbench_tmp.x = j;
							workbench_tmp.y = i;
							workbench_tmp.type = type;
                        	work_bench_cluster[del_robot_wb][type].Add(cnt[del_robot_wb]++,(double)j*0.5+0.25, (double)i*0.5+0.25);
							usable_workbench_for_robot[del_robot_wb].push_back(workbench_tmp);
                    	}
					}
					wait_del_workbench++;
				}
				else if(input[j] == '#')
				{
					int type = input[j] - '0';
					//判断地图的id
					check_map_id(input, i, j, type);
					cost_map[j][i] = INT_MAX;
					for(int m = 0; m < 4; m++) cost_map_for_robot[m][j][i] = INT_MAX;
				}


				for(int robotId = 0; robotId < 4; robotId++){
					int num = i*100 + j;
					if(input[j] == '#'){
						robot_cluster[robotId].all_node_empty[num].bObstacle = true;
						//robot_cluster[robotId].all_node_empty[num].node_cost = 10000;
						robot_cluster[robotId].all_node_object[num].bObstacle = true;
						//robot_cluster[robotId].all_node_object[num].node_cost = 10000;
						robot_cluster[robotId].all_node_empty[num].symbol = '#';
						robot_cluster[robotId].all_node_object[num].symbol = '#';

						if(j-1>=0){
							robot_cluster[robotId].all_node_empty[num - 1].bObstacle = true;
							robot_cluster[robotId].all_node_object[num - 1].bObstacle = true;
							//robot_cluster[robotId].all_node_object[num - 1].node_cost = 10000;
						}
						if(j+1<=99){
							robot_cluster[robotId].all_node_empty[num + 1].bObstacle = true;
							robot_cluster[robotId].all_node_object[num + 1].bObstacle = true;
							//robot_cluster[robotId].all_node_object[num + 1].node_cost = 10000;
						}
						if(i-1>=0){
							robot_cluster[robotId].all_node_empty[num - 100].bObstacle = true;
							robot_cluster[robotId].all_node_object[num - 100].bObstacle = true;
							//robot_cluster[robotId].all_node_object[num - 100].node_cost = 10000;
						}
						if(i+1<=99){
							robot_cluster[robotId].all_node_empty[num + 100].bObstacle = true;		
							robot_cluster[robotId].all_node_object[num + 100].bObstacle = true;
							//robot_cluster[robotId].all_node_object[num + 100].node_cost = 10000;
						}
						//left_up
						if(i+1<=99 && j-1>=0){
							robot_cluster[robotId].all_node_empty[num + 100 - 1].node_cost = 4;
							robot_cluster[robotId].all_node_object[num + 100 - 1].node_cost = 500;
						}
						//right_up
                        if(i+1<=99 && j+1<=99){
							robot_cluster[robotId].all_node_empty[num + 100 + 1].node_cost = 4;
							robot_cluster[robotId].all_node_object[num + 100 + 1].bObstacle = true;
						}
						//left_down
                        if(i-1>=0 && j-1>=0){
							robot_cluster[robotId].all_node_empty[num - 100 - 1].node_cost = 4;
							//robot_cluster[robotId].all_node_object[num - 100 - 1].bObstacle = true;
							//robot_cluster[robotId].all_node_object[num - 100 - 1].node_cost = 10000;
							robot_cluster[robotId].all_node_object[num - 100 - 1].node_cost = 500;
						}
						//right_down
                        if(i-1>=0 && j+1<=99){
							robot_cluster[robotId].all_node_empty[num - 100 + 1].node_cost = 4;
							//robot_cluster[robotId].all_node_object[num - 100 + 1].bObstacle = true;
							robot_cluster[robotId].all_node_object[num - 100 + 1].bObstacle = true;
						}
					}
					else{
						if(input[j] <= '9' && input[j] >= '0')
						{
							int type = input[j] - '0';
							robot_cluster[robotId].all_node_empty[num].symbol = input[j];
							robot_cluster[robotId].all_node_object[num].symbol = input[j];
							if(robot_cluster[robotId].all_node_empty[num].bObstacle == true){
								if(input[j] == '1' || input[j] == '2' || input[j] == '3'){
									Robot::target_set.insert({(j*0.5+0.25) * 100 +i*0.5+0.25, type});
								}
								else{
									for(auto &h : WorkBenchIdForSell[type]){
									Robot::target_set.insert({(j*0.5+0.25) * 100 +i*0.5+0.25, h});
									}
								}		
							}
						}

						int x = j;
						int y = i;
						robot_cluster[robotId].all_node_empty[num].x = x;
						robot_cluster[robotId].all_node_empty[num].y = y;
						robot_cluster[robotId].all_node_empty[num].coordinate_x = j*0.5+0.25;
						robot_cluster[robotId].all_node_empty[num].coordinate_y = i*0.5+0.25;
						robot_cluster[robotId].all_node_empty[num].parent = nullptr;

						if(i > 0) robot_cluster[robotId].all_node_empty[num].vecNeighbours.push_back(&robot_cluster[robotId].all_node_empty[(i - 1) * 100 + (j + 0)]);
						if(i < 99) robot_cluster[robotId].all_node_empty[num].vecNeighbours.push_back(&robot_cluster[robotId].all_node_empty[(i + 1) * 100 + (j + 0)]);
						if (j > 0) robot_cluster[robotId].all_node_empty[num].vecNeighbours.push_back(&robot_cluster[robotId].all_node_empty[(i + 0) * 100 + (j - 1)]);
						if(j < 99) robot_cluster[robotId].all_node_empty[num].vecNeighbours.push_back(&robot_cluster[robotId].all_node_empty[(i + 0) * 100 + (j + 1)]);
						
						// We can also connect diagonally
						if (i > 0 && j > 0) robot_cluster[robotId].all_node_empty[num].vecNeighbours.push_back(&robot_cluster[robotId].all_node_empty[(i - 1) * 100 + (j - 1)]);
						if (i < 99 && j > 0) robot_cluster[robotId].all_node_empty[num].vecNeighbours.push_back(&robot_cluster[robotId].all_node_empty[(i + 1) * 100 + (j - 1)]);
						if (i > 0 && j < 99) robot_cluster[robotId].all_node_empty[num].vecNeighbours.push_back(&robot_cluster[robotId].all_node_empty[(i - 1) * 100 + (j + 1)]);
						if (i < 99 && j < 99) robot_cluster[robotId].all_node_empty[num].vecNeighbours.push_back(&robot_cluster[robotId].all_node_empty[(i + 1) * 100 + (j + 1)]);


						if(robot_cluster[robotId].all_node_object[num].bObstacle == true) continue;
						//if(robot_cluster[robotId].all_node_object[num].node_cost == 200 && input[j] != '.') robot_cluster[robotId].all_node_object[num].node_cost = 3;
						if(input[j] != '.' && input[j] != 'A') robot_cluster[robotId].all_node_object[num].node_cost = 3;
						robot_cluster[robotId].all_node_object[num].x = x;
						robot_cluster[robotId].all_node_object[num].y = y;
						robot_cluster[robotId].all_node_object[num].coordinate_x = j*0.5+0.25;
						robot_cluster[robotId].all_node_object[num].coordinate_y = i*0.5+0.25;
						robot_cluster[robotId].all_node_object[num].parent = nullptr;

						if(i > 0) robot_cluster[robotId].all_node_object[num].vecNeighbours.push_back(&robot_cluster[robotId].all_node_object[(i - 1) * 100 + (j + 0)]);
						if(i < 99) robot_cluster[robotId].all_node_object[num].vecNeighbours.push_back(&robot_cluster[robotId].all_node_object[(i + 1) * 100 + (j + 0)]);
						if (j > 0) robot_cluster[robotId].all_node_object[num].vecNeighbours.push_back(&robot_cluster[robotId].all_node_object[(i + 0) * 100 + (j - 1)]);
						if(j < 99) robot_cluster[robotId].all_node_object[num].vecNeighbours.push_back(&robot_cluster[robotId].all_node_object[(i + 0) * 100 + (j + 1)]);
						
						// We can also connect diagonally
						if (i > 0 && j > 0) robot_cluster[robotId].all_node_object[num].vecNeighbours.push_back(&robot_cluster[robotId].all_node_object[(i - 1) * 100 + (j - 1)]);
						if (i < 99 && j > 0) robot_cluster[robotId].all_node_object[num].vecNeighbours.push_back(&robot_cluster[robotId].all_node_object[(i + 1) * 100 + (j - 1)]);
						if (i > 0 && j < 99) robot_cluster[robotId].all_node_object[num].vecNeighbours.push_back(&robot_cluster[robotId].all_node_object[(i - 1) * 100 + (j + 1)]);
						if (i < 99 && j < 99) robot_cluster[robotId].all_node_object[num].vecNeighbours.push_back(&robot_cluster[robotId].all_node_object[(i + 1) * 100 + (j + 1)]);	
					}
				}

			}
			i--;
		}
	}
	return false;
}
 
 /*
void hw_compet::Deal_Clash(int RobotID) {
	auto &cur_robot = robot_cluster[RobotID];
    for(int i = 0; i < 4; ++i) {
		if(i == RobotID) continue;
		auto &robot_neigh = robot_cluster[i];
		auto dis = CalDis(cur_robot, robot_neigh);
		if (dis <= 1.5 && ((cur_robot.direction >= 0 && robot_neigh.direction < 0) || (cur_robot.direction <= 0 && robot_neigh.direction > 0))) {
			cur_robot.direction = robot_neigh.direction;
			linear_speed = -2;//以最大速度回退
			angular_speed = ((rand() & 1) ? 1 : -1) * 3.1415926;
		}
	}
}*/

bool hw_compet::Deal_Clash(int RobotID, int ptr_flag) {
    auto &cur_robot = robot_cluster[RobotID];
    for(int i = RobotID; i < 4; ++i) {
        if(i == RobotID) continue;
        auto &robot_neigh = robot_cluster[i];
        auto dis = CalDis(cur_robot, robot_neigh);
        if (ptr_flag == 0 && dis <= 1.3){ //&& ((cur_robot.direction >= 0 && robot_neigh.direction <= 0) || (cur_robot.direction <= 0 && robot_neigh.direction >= 0))) {
            return true;
            //cur_robot.direction = robot_neigh.direction;
            //linear_speed = -2;//以最大速度回退
            //angular_speed = ((rand() & 1) ? 1 : -1) * 3.1415926;
        }
        else if(ptr_flag == 1 && dis <= 1.2){
        	return true;
        }
    }
    return false;
}



int hw_compet::astar(sNode *nodeStart, sNode *nodeEnd, int robotId, int carry_empty_type)
{
	int point_nums = 1;
	//std::cerr<<" ^^^^^^^^ "<< &robot_cluster[robotId].all_node_object<<std::endl;
    if(carry_empty_type == 0) robot_cluster[robotId].all_node_empty = robot_cluster[robotId].fixed_all_node_empty;
    else robot_cluster[robotId].all_node_object = robot_cluster[robotId].fixed_all_node_object;
    auto distance = [](sNode* a, sNode* b) // For convenience
	{
        return abs((a->x - b->x))+abs((a->y - b->y));
		//return sqrtf((a->coordinate_x - b->coordinate_x)*(a->coordinate_x - b->coordinate_x) + (a->coordinate_y - b->coordinate_y)*(a->coordinate_y - b->coordinate_y));
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
			double fPossiblyLowerGoal = nodeCurrent->fLocalGoal + distance(nodeCurrent, nodeNeighbour) * nodeNeighbour -> node_cost;// * distance(nodeCurrent, nodeNeighbour);

			//替换s
            //std::cerr << "fPossiblyLowerGoal"<< fPossiblyLowerGoal << std::endl;
            //std::cerr << "nodeNeighbour->fLocalGoal"<< nodeNeighbour->fLocalGoal << std::endl;
			if (fPossiblyLowerGoal < nodeNeighbour->fLocalGoal)
			{
				nodeNeighbour->parent = nodeCurrent;
                //std::cerr << "nodeCurrent is "<< nodeCurrent->x<< ","<< nodeCurrent->y<< std::endl;
				nodeNeighbour->fLocalGoal = fPossiblyLowerGoal;
                double distance_left = heuristic(nodeNeighbour, nodeEnd);
                //if(distance_left > 8) nodeNeighbour->fGlobalGoal = nodeNeighbour->fLocalGoal + heuristic(nodeNeighbour, nodeEnd);
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
        nodeStart->planning_point_nums++;
	    sNode *p = nodeEnd;
        point.x = p -> coordinate_x;
        point.y = p -> coordinate_y;
        //std::cerr<<point.x<<"  "<<point.y<<std::endl;
        robot_cluster[robotId].robot_execute_points.push_back(point);
        //outfile<<point.x<<"  "<<point.y<<std::endl;
        //outfile2<<point.x<<"  "<<point.y<<std::endl;
		while (p->parent != nullptr && p -> parent -> parent != nullptr)
		{
			point_nums++;
            //std::string str;
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

            //robot_cluster[robotId].robot_execute_points.push(point);
            if(remove_middle_points(point1, point2, point3) == true) {
                //outfile2<<point.x<<"  "<<point.y<<std::endl;
                robot_cluster[robotId].robot_execute_points.push_back(point);
            }
		}
    }
	std::reverse(robot_cluster[robotId].robot_execute_points.begin(),robot_cluster[robotId].robot_execute_points.end());
	return point_nums;
}









bool hw_compet::remove_middle_points(std::pair<double,double>& point1, std::pair<double,double>& point2, std::pair<double,double>& point3) {
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





void hw_compet::check_map_id(string s, int i, int j, int type) {
	if(i == 97 && j == 3 && type == 6) this->map_id = 3;
	if(i == 60 && j == 43 && type == 1) this->map_id = 2;
	if(i == 94 && j == 47 && type == 5) this->map_id = 0;
	if(i == 74 && j == 50 && type == 6) this->map_id = 1;
	//this->map_id = 0;
}
//初始化一个机器人的工作台位置数组并获得机器人的目标
WorkBenchNodeForRobot hw_compet::GetRobotTarget(Robot& robot,int robotId) {
	//先清空机器人的工作台数组
	robot.Clear_vec();
	//先来给它初始化---注意工作台从1开始重新加
	for(int i = 1; i < work_bench_cluster[robotId].size(); ++i) {
		for(auto wb: work_bench_cluster[robotId][i].WorkBenchVec) {
            double dis = update_distance(wb, robot);
			//std::sort(wb.workbench_route_workbench.begin(),wb.workbench_route_workbench.end());
			//if(i == 1 && wb.workbench_route_workbench.size() > 1) std::cerr<<"&&&&&&&&&&  "<<wb.workbench_route_workbench[0][0][0]<<"   "<<wb.workbench_route_workbench[1][0][0]<<std::endl;
			robot.workbench_for_robot[i].push_back(WorkBenchNodeForRobot(wb.global_id, i, i, wb.x,wb.y, dis, wb.ori_material_status, wb.product_status, wb.remain_production_time, wb.workbench_route_workbench));
			//std::cerr<<"^^^^^^^^^^^^^^  "<<wb.workbench_route_workbench.size()<<std::endl;
			/*for(int m = 1; m <=9; m++)
			{
				if(wb.workbench_route_workbench[m].size() != 0){
					//std::cerr<<wb.global_id<<"    "<<wb.workbench_route_workbench.size()<<std::endl;
					robot_cluster[robotId].target_route_points[wb.global_id] = wb.workbench_route_workbench;
					break;
				}
			}*/
		}
		
	}
	//随后得到改机器人的目标
    WorkBenchNodeForRobot target;
	target = robot.GetTarget2(robotId);
	//if(this->map_id == 0) target = robot.GetTarget1(robotId);
	//else if(this->map_id == 1) target = robot.GetTarget2(robotId);
	//else if(this->map_id == 2) target = robot.GetTarget3(robotId);
	//else target = robot.GetTarget4(robotId);
	return target;
}




void hw_compet::astar_init_using(sNode *nodeStart, sNode *nodeEnd, int robotId, int global_id_end_point, int workbench_type, WorkBenchNode* workbench_route_start, WorkBenchNode* workbench_route_end)
{
	//std::ofstream outfile("./log.txt",std::ios_base::app);
	//outfile<<nodeStart->coordinate_x<<" "<<nodeStart->coordinate_y<<"  "<<nodeEnd->coordinate_x<<"  "<<nodeEnd->coordinate_y<<std::endl;
	int cnt_points = 1;
	//std::cerr<<"@@@@@@@@@@@@@@@@@@@@@   "<<std::endl;
	std::vector<std::array<double,4>> startwb_to_endwb_route; 	
    robot_cluster[robotId].all_node_object = robot_cluster[robotId].fixed_all_node_object;
    auto distance = [](sNode* a, sNode* b) // For convenience
	{
        return abs((a->x - b->x))+abs((a->y - b->y));
		//return sqrtf((a->coordinate_x - b->coordinate_x)*(a->coordinate_x - b->coordinate_x) + (a->coordinate_y - b->coordinate_y)*(a->coordinate_y - b->coordinate_y));
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
			double fPossiblyLowerGoal = nodeCurrent->fLocalGoal + distance(nodeCurrent, nodeNeighbour) * nodeNeighbour -> node_cost;
			//替换s
			if (fPossiblyLowerGoal < nodeNeighbour->fLocalGoal)
			{
				nodeNeighbour->parent = nodeCurrent;
				nodeNeighbour->fLocalGoal = fPossiblyLowerGoal;
                double distance_left = heuristic(nodeNeighbour, nodeEnd);
                nodeNeighbour->fGlobalGoal = nodeNeighbour->fLocalGoal + heuristic(nodeNeighbour, nodeEnd);
			}
		}
	}

    WorkBenchNodeForRobot point;
    WorkBenchNodeForRobot pre_point;
    WorkBenchNodeForRobot next_point;
    if (nodeEnd != nullptr)
	{
        //nodeStart->planning_point_nums++;
	    sNode *p = nodeEnd;
        point.x = p -> coordinate_x;
        point.y = p -> coordinate_y;
		startwb_to_endwb_route.push_back({double(cnt_points), double(global_id_end_point), p -> coordinate_x, p -> coordinate_y});
		while (p->parent != nullptr && p -> parent -> parent != nullptr)
		{
			cnt_points++;
			//if(cnt == 1)std::cerr<<"@@@@@@@@@@@@@@@@@@@@@   "<<std::endl;
            std::string str;
            //pre_pont存放上一个点
            pre_point.x = p -> coordinate_x;
            pre_point.y = p -> coordinate_y;
            std::pair<double,double> point1 = {pre_point.x, pre_point.y};

            p = p -> parent;
            ////point存放当前点
            point.x = p -> coordinate_x;
            point.y = p -> coordinate_y;
            std::pair<double,double> point2 = {point.x, point.y};
            //next_pont存放下一个点
            next_point.x = p -> parent -> coordinate_x;
            next_point.y = p -> parent -> coordinate_y;
            std::pair<double,double> point3 = {next_point.x, next_point.y};
            if(remove_middle_points(point1, point2, point3) == true) {
				startwb_to_endwb_route.push_back({double(cnt_points), double(global_id_end_point), p -> coordinate_x, p -> coordinate_y});
            }
			//std::cerr<<"**********"<<cnt_points<<std::endl;
		}
    }
	//std::cerr<<"*&&&&&&&&&&&&&&&&&&&&&&  "<<startwb_to_endwb_route.size()<<std::endl;
    startwb_to_endwb_route[0][0] = cnt_points;
	//std::cerr<<nodeStart->x<<"  "<<nodeStart->y<<" **************** "<<global_id_end_point<<" ^^^^^^^^^^^^^^^^^ "<<workbench_type<<" ^^^^^^^^^^^^^ "<<cnt_points<<" %%%% "<<workbench_route_start->workbench_route_workbench.size()<<std::endl;
	if(cnt_points > 2) workbench_route_start->workbench_route_workbench[workbench_type].push_back(startwb_to_endwb_route);
	//std::cerr<<"******************  "<<workbench_route_start->workbench_route_workbench.size()<<std::endl;
	//if(startwb_to_endwb_route.size() == 1) outfile<<nodeStart->coordinate_x<<" "<<nodeStart->coordinate_y<<"  "<<nodeEnd->coordinate_x<<"  "<<nodeEnd->coordinate_y<<std::endl;

		//std::cerr<<"^^^^^^^^^^^^^^^  "<<workbench_route_start.workbench_route_workbench.size()<<std::endl;
		//std::sort(workbench_route_start.workbench_route_workbench.begin(), workbench_route_start.workbench_route_workbench.end());
		//for(int n = 0; n < workbench_route_start.workbench_route_workbench.size(); n++) std::cerr<<workbench_route_start.workbench_route_workbench.size()<<std::endl;
	robot_cluster[robotId].all_node_object = robot_cluster[robotId].fixed_all_node_object;

}


