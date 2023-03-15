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
double hw_compet::update_distance(const WorkBenchNode& workbench,const Robot robot)
{

    double car_location_x = robot.location_x;
    double car_location_y = robot.location_y;
    double workbench_location_x = workbench.x;
    double workbench_location_y = workbench.y;
    double distance=sqrt(pow(car_location_x-workbench_location_x,2)+pow(car_location_y-workbench_location_y,2));
	return distance;
}


//速度计算
void hw_compet::vel_cmd_out(pid_controller **pid, double &aS, double &lS, double yaw, double angle, double distance)
{
		double a = yaw - angle;
		aS = (pid_update(*pid, 0.0f, yaw - angle));
		if(abs(a) < 0.7)
		{
			lS = (abs(100/a));
			if(distance<1)
			{
				lS = 3;
			}
		}
		else
			lS = 3;
		//lS = 1/abs(aS)+0.3;
		//lS = 0;
}
//pid初始化
void hw_compet::pid_init(pid_controller *pid)
{
	pid->integrator = 0.0f;
	pid->differentiator = 0.0f;
	pid->pre_err = 0.0f;
	pid->pre_measure = 0.0f;
	pid->out = 0.0f;
	pid->T = 0.020;
	pid->lim_min = -3.1415926;
	pid->lim_max = 3.1415926;
	pid->Kp = 10;
	pid->Ki = 0;
	pid->Kd = 0.0000;
	pid->tau = 0;
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




bool hw_compet::readUntilOK() {
	char line[1024];
	int workbench_flag = 1;
	int robot_flag = 0;
	int flag_event = 1;
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
                    while (workbench_flag != workbench_num)
                    {
                        flag_event--;
                        break;
                    }
                    //根据横纵坐标，得到行与列数
                    //double workbench_row=(stod(data_each_frame[1])+0.25)/0.5;
                    //double workbench_col=(stod(data_each_frame[2])+0.25)/0.5;
                    double workbench_row = stod(data_each_frame[1]);
                    double workbench_col = stod(data_each_frame[2]);
                    //工作台的编号1--9
					
                    work_bench_cluster[stoi(data_each_frame[0])].Update(workbench_row,workbench_col,stoi(data_each_frame[3]),stoi(data_each_frame[4]),stoi(data_each_frame[5])); 
                    workbench_flag++;
                    break;
                }
                case 4:
                {
                    flag_event--;
                    robot_cluster[robot_flag].Update(stoi(data_each_frame[0]),stoi(data_each_frame[1]),stod(data_each_frame[2]),stod(data_each_frame[3]),stod(data_each_frame[4]),stod(data_each_frame[5]),stod(data_each_frame[6]),stod(data_each_frame[7]),stod(data_each_frame[8]),stod(data_each_frame[9]));
                    robot_flag++;
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
    int cnt = 0;
	struct workbench *wb;
	struct workbench *wb_tmp;

	while (fgets(line, sizeof line, stdin)) {
		if (line[0] == 'O' && line[1] == 'K') {
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
					Robot robot_tmp(j*0.5+0.25, i*0.5+0.25);
					robot_cluster.push_back(robot_tmp);
				}
				else if (input[j] >= '1' && input[j] <= '9')
				{
					int type = input[j] - '0';
					//更新工作台位置
					work_bench_cluster[type].Add(cnt++,(double)j*0.5+0.25, (double)i*0.5+0.25);
				}
			}
			i--;
		}
	}
	return false;
}

//初始化一个机器人的工作台位置数组并获得机器人的目标
WorkBenchNodeForRobot hw_compet::GetRobotTarget(Robot& robot) {
	//先清空机器人的工作台数组
	robot.Clear_vec();
	//先来给它初始化---注意工作台从1开始重新加
	for(int i = 1; i < work_bench_cluster.size(); ++i) {
		for(auto wb: work_bench_cluster[i].WorkBenchVec) {
			double dis = update_distance(wb, robot);
			robot.workbench_for_robot[i].push_back(WorkBenchNodeForRobot(wb.global_id, i, wb.x,wb.y, dis, wb.ori_material_status, wb.product_status));
		}
	}
	//随后得到改机器人的目标
    WorkBenchNodeForRobot target = robot.GetTarget();
	return target;
}
