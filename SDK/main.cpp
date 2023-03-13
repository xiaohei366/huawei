#include "hwcompet.h"


using namespace std;




int main()
{
	hw_compet obj;
    pid_controller **ptr;
	obj.init();
	puts("OK");
	fflush(stdout);
	while (scanf("%d", &obj.frame_num) != EOF) {
		obj.readUntilOK();
		printf("%d\n", obj.frame_num);
        for(int i = 0; i < 4; i++)
        {
            WorkBenchNodeForRobot target = obj.GetRobotTarget(obj.robot_cluster[i]);
			//double workbench_x = 17;//target->x;
			double workbench_x = target.x;
			//double workbench_y = 50-17;//50-target->y;
			double workbench_y = target.y;
			double distance_target = target.dis;
            obj.get_yaw_angle(obj.yaw,obj.vector_angle,workbench_x,workbench_y,obj.robot_cluster[i]);
            ptr = obj.control_ptr;
            obj.vel_cmd_out(ptr,obj.angular_speed,obj.linear_speed,obj.yaw,obj.vector_angle,distance_target);	
            ptr++;
            printf("forward %d %f\n", i, obj.linear_speed);
            printf("rotate %d %f\n", i, obj.angular_speed);
        }
		//注意！！ 需要每次清空目标
		Robot::Clear_set();
		//购买和卖的逻辑
        for(int robotId = 0; robotId < 4; robotId++)
        {
            if(obj.robot_cluster[robotId].carried_item_type != 0)
            {
                printf("sell %d\n", robotId);
            }
            else
            {
                printf("buy %d\n", robotId);
            }
        }
		printf("OK\n");
		fflush(stdout);
	}
	return 0;
}

                             

