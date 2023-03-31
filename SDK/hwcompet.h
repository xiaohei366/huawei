#ifndef HW_Compet_H
#define HW_Compet_H

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
#include <stack>
#include <utility>
#include <limits>

using namespace std;

//pid控制
struct pid_controller
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
};








class hw_compet {
public:
	//4个机器人
	vector<Robot> robot_cluster;
	vector<vector<WorkBench>> work_bench_cluster;
    pid_controller* control_ptr[4];

	hw_compet()
	{
		//初始化地图编号
		map_id = 0;
		cost_map = vector<vector<int>>(100,std::vector<int>(100,0));
		cost_map_for_robot = vector<vector<vector<int>>>(4,vector<vector<int>>(100,vector<int>(100,0)));
		//初始化pid参数
		distance_wb.resize(10);
		//初始化工作台集群
		for(int j=0;j<4;j++)
		{
			robot_cluster.push_back(Robot(0,0));
			for(int i = 0; i <= 9; ++i) {
				workbenck_cluster_2d.push_back(WorkBench(i));
			}
			work_bench_cluster.push_back(workbenck_cluster_2d);
			workbenck_cluster_2d.clear();
		}
		
	}
	//存放机器人数据
	vector<array<double, 10>> robot;
	//存放工作台数据
	unordered_map<char, vector<array<double, 5>>> um_workbench;
	//存放距离
	vector<vector<double>> distance_wb;




    //帧序号
    int frame_num=0;
	int initial_money = 0;
	int workbench_num = 0;
    double angular_speed = 0;
    double linear_speed = 0;
    double yaw = 0;
    double vector_angle = 0;
    double distance_target = 0;
	//初始化函数 读取地图
	bool init();

	//处理两个小车相撞后的冲突
	void Deal_Clash(int RobotID);

	double CalDis(const Robot robot1, const Robot robot2) {
		return sqrt((robot1.location_x - robot2.location_x) * (robot1.location_x - robot2.location_x) + (robot1.location_y - robot2.location_y) * (robot1.location_y - robot2.location_y));
	};
	//与判题器进行交互的函数
	bool readUntilOK();
	//得到pid调节参数yaw,angle_x
    void get_yaw_angle(double& distance_target,double& yaw ,double& vector_angle ,double workbench_x ,double workbench_y ,const Robot& robot_cluster_get_angle);
	//更新每一个小车与所有工作台的距离
    double update_distance(const WorkBenchNode& workbench,const Robot robot);
    //初始化一个机器人的工作台位置数组
	WorkBenchNodeForRobot GetRobotTarget(Robot& robot, int robotId);
	//pid初始化和计算
    void vel_cmd_out(pid_controller **pid, double &aS, double &lS, double yaw, double angle, double distance, int map_id);
	void pid_init(pid_controller *pid, int map_id);
	double pid_update(pid_controller *pid, double setpoint, double measure);
	void check_map_id(string s, int i, int j, int type);
	//初始化得到的原始代价地图
	vector<vector<int>> cost_map;

	vector<vector<vector<int>>> cost_map_for_robot;
	//四张地图*四个机器人,
	vector<vector<vector<pair<int,int>>>> add_obstacle
	{
		{{{},{},{}},{{},{},{}},{{},{},{}},{{},{},{}}},
		{{{},{},{}},{{},{},{}},{{},{},{}},{{},{},{}}},
		{{{},{},{}},{{},{},{}},{{},{},{}},{{},{},{}}},
		{{{},{},{}},{{},{},{}},{{},{},{}},{{},{},{}}}
	};

	int map_id;
private:
    pid_controller controller[4];
	vector<WorkBench> workbenck_cluster_2d;
	//yaw是机器人朝向，angle_x是向量与x轴正方向夹角
	const std::vector<std::vector<double>> pid_param{
		//aS = (pid_update(*pid, pid_param[map_id][0], yaw - angle));
		// if(abs(a) < pid_param[map_id][1])
		// {
		// 	lS = (abs(pid_param[map_id][2]/a));
		// 	if(distance<pid_param[map_id][3])
		// 	{
		// 		lS = pid_param[map_id][4];
		// 	}
		// }
		// else
		// 	lS = pid_param[map_id][5];
		// pid->Kp = pid_param[map_id][6];
		// pid->Ki = pid_param[map_id][7];
		// pid->Kd = pid_param[map_id][8];
		// pid->tau = pid_param[map_id][9];
		{0.01, 0.5, 100, 0.5, 0.5, 0.5, 10, 5, 0, 0},
		//jian shao zhuang qiang shun shi hen duo
		{0.01, 0.5, 100, 0.5, 0.5, 0.5, 10, 5, 0, 0},
		{0.01, 0.5, 100, 0.5, 0.2, 0.3, 10, 5, 0, 0},
		//{0.1, 0.9, 100, 1, 3, 3.0, 10, 5, 0, 0}
		{0.01, 0.5, 100, 0.5, 0.2, 0.3, 10, 5, 0, 0}
		//{0.15, 0.9, 100, 1, 3.1, 3.5, 10, 5, 0, 0}
	};

//{1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43},
//{1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25},
//{1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50},
//{1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18}
 
	//需要删除的工作台坐标/序号
    const std::vector<std::vector<unordered_set<int>>> del_workbench_map{
        //map_1
		{{},
		//help robot
		{},
		{},
		{}
		},
		{{},
		{},
		{},
		{}},
		//map_3
		{{},
		{},
		{},
		{}},
		//map_4
		{{},
		{},
		{},
		{}}
    };
};






#endif
