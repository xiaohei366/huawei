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
	//帧序号
	int frame_num = 0;
	int initial_money = 0;
	int workbench_num = 0;


	
	bool init();
	bool readUntilOK();

private:
	vector<vector<int>> distance_wb;
	int flag_event = 1;
	int loop_num = 0;
};



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
	while (fgets(line, sizeof line, stdin))
	{
		if (line[0] == 'O' && line[1] == 'K') {
			flag_event = 1;
			return true;
		}
		else
		{
			vector<string> numbers;
			std::string input(line);
			std::istringstream iss(input);
			string one_str;
			while (iss >> one_str)
				numbers.push_back(one_str);
			switch (flag_event) {
			case 1:
				frame_num = stoi(numbers[0]);
				initial_money = stoi(numbers[1]);
				break;
			case 2:
				workbench_num = stoi(numbers[0]);
				loop_num = workbench_num;
				break;
			case 3:
				while (loop_num != 1)
				{
					loop_num--;
					flag_event--;
					break;
				}
				array<float, 5> tem_var_machine;
				for (int i = 1; i < numbers.size(); i++)
					tem_var_machine[i - 1] = (stof(numbers[i]));
				um_workbench[numbers[0][0]].push_back(tem_var_machine);
				break;
			case 4:
				flag_event--;
				array<float, 10> tem_var_robot;
				for (int i = 0; i < numbers.size(); i++)
					tem_var_robot[i] = stof(numbers[i]);
				robot.push_back(tem_var_robot);
				break;
			default:
				break;
			}
			flag_event++;
		}
	}
	return false;
}



bool hw_compet::init() {
	char line[1024];
	char *ptr = &line[0];
	int line_cnt = 0;
	int col_cnt = 0;
	//double *robot_ptr = &robot_info;
	struct workbench *wb;
	struct workbench *wb_tmp;
	int wb_cnt = 0;


	while (fgets(line, sizeof line, file)) {
		if (line[0] == 'O' && line[1] == 'K') {

			printf("reading done\n");
			for (int n = 0; n < wb_cnt; n++)
			{
				printf("id = %c\n", (char)((wb + n)->id));
			}

			return true;
		}
		else
		{
			//指针重新归位
			//x清零
			line_cnt++;
			ptr = &line[0];
			col_cnt = 0;

			while (*ptr != '\n')
			{
				col_cnt++;
				switch (*ptr)
				{
				case '1':
				case '2':
				case '3':
				case '4':
				case '5':
				case '6':
				case '7':
				case '8':
				case '9':
				{
					wb_cnt++;
					if (wb == NULL)
					{
						wb = (struct workbench *)malloc(sizeof(struct workbench));
						printf("x=%d y=%d\n", col_cnt, line_cnt);
						wb->id = *ptr;
						wb->x = col_cnt;
						wb->y = line_cnt;
					}
					else
					{
						wb_tmp = (struct workbench *)realloc(wb, wb_cnt * sizeof(struct workbench));
						if (wb_tmp)
						{
							wb = wb_tmp;
							printf("x=%d y=%d\n", col_cnt, line_cnt);
							(wb + wb_cnt - 1)->id = *ptr;
							(wb + wb_cnt - 1)->x = col_cnt;
							(wb + wb_cnt - 1)->y = line_cnt;
						}
						else
						{
							printf("error\n");
							return 1;
						}
					}
					break;
				}
				case 'A':
				{
					//printf("x=%d y=%d\n", col_cnt, line_cnt);
					*robot_ptr++ = (double)col_cnt;
					*robot_ptr++ = (double)line_cnt;
					break;
				}
				default:
					break;
				}

				ptr++;
			}
		}
		//do something

	}
	return false;
}


int main()
{
	hw_compet obj;
	if (obj.readUntilOK())
		puts("OK");
	return 0;
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
