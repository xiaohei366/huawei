#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <string>
#include <vector>
#include <array>
#include <unordered_map>
#include <unordered_set>
#include <cstdio>
#include <type_traits>
#include "robot.h"
#include "workbench.h"


using namespace std;

//pid控制
typedef struct
{
	double Kp;
	double Ki;
	double Kd;

	double tau;

	double lim_min;
	double lim_max;

	//sample time
	double T;

	//controller memory
	double integrator;
	double pre_err;
	double differentiator;
	double pre_measure;

	double out;
}pid_controller;

struct workbench {
	int id;
	double x;
	double y;
};

class hw_compet {
public:
	//4个机器人
	vector<Robot> robot_cluster;
	WorkBench work_bench_cluster_1;
	WorkBench work_bench_cluster_2;
	WorkBench work_bench_cluster_3;
	WorkBench work_bench_cluster_4;
	WorkBench work_bench_cluster_5;
	
	WorkBench work_bench_cluster_6;
	WorkBench work_bench_cluster_7;
	WorkBench work_bench_cluster_8;
	WorkBench work_bench_cluster_9;

	hw_compet()
	{
		//初始化pid参数
		pid_init(control_ptr);
		distance_wb.resize(10);
	}
	//存放机器人数据
	vector<array<float, 10>> robot;
	//存放工作台数据
	unordered_map<char, vector<array<float, 5>>> um_workbench;
	//存放距离
	vector<vector<float>> distance_wb;
	//yaw是机器人朝向，angle_x是向量与x轴正方向夹角
	double yaw, angle_x;
	//帧序号
	int frame_num = 0;
	int initial_money = 0;
	int workbench_num = 0;
	//初始化函数 读取地图
	bool init();
	//与判题器进行交互的函数
	bool readUntilOK();
	//得到pid调节参数yaw,angle_x
	void get_yaw_angle(double& yaw, double& angle_x, int robot_id, const vector<array<float, 10>>& robot, double wb_minX, double wb_minY);
	//更新每一个小车与所有工作台的距离
	void update_distance(const vector<array<float, 10>>& robot, const unordered_map<char, vector<array<float, 5>>>& um_wb, vector<vector<float>>& distance, int robot_id);

	//pid初始化和计算
	void pid_init(pid_controller *pid);
	double pid_update(pid_controller *pid, double setpoint, double measure);
	
private:
	pid_controller* control_ptr = new pid_controller();
	int flag_event = 1;
	int loop_num = 0;
};

int main()
{
	hw_compet obj;
	obj.init();
	cout << (obj.work_bench_cluster_1.Find(3, 3))->x << " " << (obj.work_bench_cluster_1.Find(3, 3))->y << endl;
	cout << (obj.work_bench_cluster_1.Find(50, 37))->x << " " << (obj.work_bench_cluster_1.Find(50, 37))->y << endl;
	puts("OK");
	while (1)
	{
		bool res = obj.readUntilOK();
		cout << obj.robot_cluster[0].workstation_id << endl;
		cout << obj.robot_cluster[1].workstation_id << endl;
		cout << obj.robot_cluster[2].workstation_id << endl;
		cout << obj.robot_cluster[3].workstation_id << endl;
		//for (int i = 0; i < 4; i++)
			//obj.update_distance(obj.robot, obj.um_workbench, obj.distance_wb, i);
	}
	return 0;
}


void hw_compet::get_yaw_angle(double& yaw, double& angle_x, int robot_id, const vector<array<float, 10>>& robot, double wb_minX, double wb_minY)
{
	//机器人朝向
	yaw = robot[robot_id][7];
	double delta_x = wb_minX - robot[robot_id][8];
	double delta_y = wb_minY - robot[robot_id][9];
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
	angle_x = acos(dot_product / (vec1_length * vec2_length));
	if (delta_y < 0)
		angle_x = -angle_x;

}



//更新单独一个机器人到各个工作台的距离(为了便于多线程)
//也可以修改成直接求解四个机器到工作台的距离
void hw_compet::update_distance(const vector<array<float,10>>& robot, const unordered_map<char, vector<array<float, 5>>>& um_wb,vector<vector<float>>& distance_wb,int robot_id)
{
	distance_wb.clear();
	distance_wb.resize(10);
	for (auto &kv : um_wb)
	{
		//工作台的编号
		int key = kv.first-'0';
		//同一ID工作台对应的不同参数
		vector<array<float, 5>> value = kv.second;
		for (int j = 0; j < value.size(); j++)
		{
			float num=pow(value[j][0] - robot[robot_id][8], 2) + pow(value[j][1] - robot[robot_id][9], 2);
			distance_wb[key].push_back(num);
		}
	}
}

//pid初始化
void hw_compet::pid_init(pid_controller *pid)
{
	pid->integrator = 0.0f;
	pid->differentiator = 0.0f;
	pid->pre_err = 0.0f;
	pid->pre_measure = 0.0f;
	pid->out = 0.0f;
	pid->T = 0.02;
	pid->lim_min = -3.0f;
	pid->lim_max = 3.0f;
	pid->Kp = 5.5;
	pid->Ki = 0.008;
	pid->Kd = 0.00;
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
	int workbench_flag=1;
	int robot_flag=0;

	while (fgets(line, sizeof line, stdin))
	{
		if (line[0] == 'O' && line[1] == 'K') {
			flag_event = 1;
			return true;
		}
		else
		{
			vector<string> data_each_frame;
			std::string input(line);
			std::istringstream iss(input);
			string one_str;
			while (iss >> one_str)
				data_each_frame.push_back(one_str);
			switch (flag_event) {
			case 1:
				{
					frame_num = stoi(data_each_frame[0]);
					initial_money = stoi(data_each_frame[1]);
					break;
				}
			case 2:
				{
					workbench_num = stoi(data_each_frame[0]);
					loop_num = workbench_num;
					break;
				}
			case 3:
				{
				while (loop_num != 1)
				{
					loop_num--;
					flag_event--;
					break;
				}
					//根据横纵坐标，得到行与列数
							int workbench_row=int((stof(data_each_frame[1])+0.25)/0.5);
							int workbench_col=int((stof(data_each_frame[2])+0.25)/0.5);
							switch(workbench_flag){
									case 1:
											work_bench_cluster_1.Update(workbench_row,workbench_col,stoi(data_each_frame[3]),stoi(data_each_frame[4]),stoi(data_each_frame[5]));
											break;
											/*
									case 2:
											work_bench_cluster_2.Update(workbench_row,workbench_col,data_each_frame[3],data_each_frame[4],data_each_frame[5]);
											break;
									case 3:
											work_bench_cluster_3.Update(workbench_row,workbench_col,data_each_frame[3],data_each_frame[4],data_each_frame[5]);
											break;
									case 4:
											work_bench_cluster_4.Update(workbench_row,workbench_col,data_each_frame[3],data_each_frame[4],data_each_frame[5]);
											break;
									case 5:
											work_bench_cluster_5.Update(workbench_row,workbench_col,data_each_frame[3],data_each_frame[4],data_each_frame[5]);
											break;
									case 6:
											work_bench_cluster_6.Update(workbench_row,workbench_col,data_each_frame[3],data_each_frame[4],data_each_frame[5]);
											break;
									case 7:
											work_bench_cluster_7.Update(workbench_row,workbench_col,data_each_frame[3],data_each_frame[4],data_each_frame[5]);
											break;
									case 8:
											work_bench_cluster_8.Update(workbench_row,workbench_col,data_each_frame[3],data_each_frame[4],data_each_frame[5]);
											break;
											*/
									case 9:
											work_bench_cluster_9.Update(workbench_row,workbench_col,stoi(data_each_frame[3]),stoi(data_each_frame[4]),stoi(data_each_frame[5]));
											break;
									default:
											break;
							}
							workbench_flag++;
				break;
				}
			case 4:
				{
					flag_event--;
					robot_cluster[robot_flag].Update(stoi(data_each_frame[0]),stoi(data_each_frame[1]),stof(data_each_frame[2]),stof(data_each_frame[3]),stof(data_each_frame[4]),stof(data_each_frame[5]),stof(data_each_frame[6]),stof(data_each_frame[7]),stof(data_each_frame[8]),stof(data_each_frame[9]));
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
	//地图行 表示坐标y
	//循环中j表示坐标x
	int i = 1;
	//double *robot_ptr = &robot_info;
	struct workbench *wb;
	struct workbench *wb_tmp;
	FILE *file;

	file = fopen("/home/songyj/huawei/maps/1.txt", "r");

	if (file == NULL)
 	{
		// 文件打开失败
		// 在这里处理错误
		printf("open err\n");
		return 1;
  }

	while (fgets(line, sizeof line, file)) {
		if (line[0] == 'O' && line[1] == 'K') {

			printf("reading done\n");

			return true;
		}
		else
		{
			std::string input(line);

			for (int j = 0; j <= 99; j++)
			{
				if (input[j] == 'A')
				{
					//更新机器人位置
					Robot robot_tmp(j*0.5-0.25, i*0.5-0.25);
					robot_cluster.push_back(robot_tmp);

			//			cout << j << " "<< i << endl;
				}
				else if (input[j] >= '1'&& input[j] <= '9')
				{
					//更新工作台位置
					switch(input[j])
					{
						case '1':
							work_bench_cluster_1.Add(j+1, i);
//							cout << j+1 << i << endl;
							work_bench_cluster_1.InitWorkBenchPostion(work_bench_cluster_1.Find(j+1, i), j+1, i);
							break;

						case '9':
							work_bench_cluster_9.Add(j+1, i);
							work_bench_cluster_9.InitWorkBenchPostion(work_bench_cluster_9.Find(j+1, i), j+1, i);
							break;

						default:
							break;
					}
				}
			}

			i++;
		}
	}
	return false;
}



/*int main() {
	readUntilOK();
	puts("OK");
	fflush(stdout);
	int frameID;
	while (scanf("%d", &frameID) != EOF) {
		readUntilOK();
		printf("%d\n", frameID);
		int lineSpeed = 3;
		double angleSpeed = 1.5;
		for (int robotId = 0; robotId < 4; robotId++) {
			printf("forward %d %d\n", robotId, lineSpeed);
			printf("rotate %d %f\n", robotId, angleSpeed);
		}
		printf("OK\n", frameID);
		fflush(stdout);
	}
	return 0;
}*/
