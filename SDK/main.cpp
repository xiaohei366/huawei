#include <iostream>
#include <fstream>
#include <sstream>
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
	//帧序号
	int frame_num = 0;
	int initial_money = 0;
	int workbench_num = 0;


	//初始化函数
	bool init();
    //读取每一帧数据
	bool readUntilOK();

private:
	vector<vector<int>> distance_wb;
	int flag_event = 1;
	int loop_num = 0;
};


int main()
{
    hw_compet obj;
    obj.init();
    if(obj.readUntilOK())
        cout<<"read frame suucess"<<endl;
    return 0;
}



//更新机器人到各个工作台的距离
void update_distance(const vector<array<float,5>>& robot, const unordered_map<char, vector<array<float, 5>>>& um_wb,vector<int>& distance,int robot_id)
{
	for (auto &kv : um_wb)
	{
		//工作台的编号
		int key = kv.first-'0';
		//同一ID工作台对应的不同参数
		vector<array<float, 5>> value = kv.second;
		for (int j = 0; j < value.size(); j++)
		{
			distance[key] = pow(value[j][0] - robot[robot_id][8], 2) + pow(value[j][1] - robot[robot_id][9], 2);
		}
		cout << endl;
	}
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

	file = fopen("/home/cx/Desktop/LinuxRelease/maps/1.txt", "r");

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
					//cout << j << " "<< i << endl;
				}
				else if (input[j] >= '1'&& input[j] <= '9')
				{
					//更新工作台位置
					switch(input[j])
					{
						case '1':
							work_bench_cluster_1.Add(j+1, i);
							cout << j+1 << i << endl;
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


/*
bool hw_compet::readUntilOK() {
	char line[1024];
    int workbench_flag=1;
    int robot_flag=0;
    Robot robot_tmp;
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
				frame_num = stoi(data_each_frame[0]);
				initial_money = stoi(data_each_frame[1]);
				break;
			case 2:
				workbench_num = stoi(data_each_frame[0]);
				loop_num = workbench_num;
				break;
			case 3:
				while (loop_num != 1)
				{
					loop_num--;
					flag_event--;
					break;
				}
                //根据横纵坐标，得到行与列数
                int workbench_row=int((data_each_frame[1]+0.25)/0.5);
                int workbench_col=int((data_each_frame[2]+0.25)/0.5);
                switch(workbench_flag){
                    case 1:
                        work_bench_cluster_1.Updata(workbench_row,workbench_col,data_each_frame[3],data_each_frame[4],data_each_frame[5]);
                        break;
                    case 2:
                        work_bench_cluster_2.Updata(workbench_row,workbench_col,data_each_frame[3],data_each_frame[4],data_each_frame[5]);
                        break;
                    case 3:
                        work_bench_cluster_3.Updata(workbench_row,workbench_col,data_each_frame[3],data_each_frame[4],data_each_frame[5]);
                        break;
                    case 4:
                        work_bench_cluster_4.Updata(workbench_row,workbench_col,data_each_frame[3],data_each_frame[4],data_each_frame[5]);
                        break;
                    case 5:
                        work_bench_cluster_5.Updata(workbench_row,workbench_col,data_each_frame[3],data_each_frame[4],data_each_frame[5]);
                        break;
                    case 6:
                        work_bench_cluster_6.Updata(workbench_row,workbench_col,data_each_frame[3],data_each_frame[4],data_each_frame[5]);
                        break;
                    case 7:
                        work_bench_cluster_7.Updata(workbench_row,workbench_col,data_each_frame[3],data_each_frame[4],data_each_frame[5]);
                        break;
                    case 8:
                        work_bench_cluster_8.Updata(workbench_row,workbench_col,data_each_frame[3],data_each_frame[4],data_each_frame[5]);
                        break;
                    case 9:
                        work_bench_cluster_9.Updata(workbench_row,workbench_col,data_each_frame[3],data_each_frame[4],data_each_frame[5]);
                        break;
                    default:
                        break;
                }
                workbench_flag++;
				break;
			case 4:
				flag_event--;
                robot_tmp.Updata(stoi(data_each_frame[0]),stoi(data_each_frame[1]),stof(data_each_frame[2]),stof(data_each_frame[3]),stof(data_each_frame[4]),stof(data_each_frame[5]),stof(data_each_frame[6]),stof(data_each_frame[6]),stof(data_each_frame[7]),stof(data_each_frame[8]),stof(data_each_frame[9]));
                robot_cluster.push_back(robot_tmp);
                robot_flag++;
				break;
			default:
				break;
			}
			flag_event++;
		}
	}
	return false;
}
*/
