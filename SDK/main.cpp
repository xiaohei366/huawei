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
            double workbench_x,workbench_y;
            if(obj.robot_cluster[i].robot_goal_point.size() == 0)
            {
                WorkBenchNodeForRobot target = obj.GetRobotTarget(obj.robot_cluster[i]);
                
                if(obj.robot_cluster[i].robot_goal_point.size() == 0) {
                    workbench_x = 1;//target.x;
                    workbench_y = 1;//target.y;
                }
                else {
                    workbench_x = obj.robot_cluster[i].robot_goal_point.top().x;
                    workbench_y = obj.robot_cluster[i].robot_goal_point.top().y;
                }
            }
            else
            {
                workbench_x = obj.robot_cluster[i].robot_goal_point.top().x;
                workbench_y = obj.robot_cluster[i].robot_goal_point.top().y;
            }

			//double workbench_x = obj.robot_cluster[i].robot_goal_point.top()[0];
			//double workbench_y = obj.robot_cluster[i].robot_goal_point.top()[1];
            obj.get_yaw_angle(obj.distance_target,obj.yaw,obj.vector_angle,workbench_x,workbench_y,obj.robot_cluster[i]);
            ptr = obj.control_ptr;
            obj.vel_cmd_out(ptr,obj.angular_speed,obj.linear_speed,obj.yaw,obj.vector_angle,obj.distance_target);	
            ptr++;
            printf("forward %d %f\n", i, obj.linear_speed);
            printf("rotate %d %f\n", i, obj.angular_speed);
        }
		//购买和卖的逻辑
        for(int robotId = 0; robotId < 4; robotId++)
        {
            if(obj.robot_cluster[robotId].robot_goal_point.size() == 0)  
            {
                continue;
            }
            int robot_workbench_id = obj.robot_cluster[robotId].workstation_id;
            double target_id = obj.robot_cluster[robotId].robot_goal_point.top().global_id;
            double type = obj.robot_cluster[robotId].robot_goal_point.top().type;
            if(target_id != robot_workbench_id) continue;
            else
            {
                double erase_num = obj.robot_cluster[robotId].robot_goal_point.top().x*100+obj.robot_cluster[robotId].robot_goal_point.top().y;
                obj.robot_cluster[robotId].target_set.erase(erase_num);
                obj.robot_cluster[robotId].robot_goal_point.pop();
                if(obj.robot_cluster[robotId].carried_item_type != 0)
                {
                    printf("sell %d\n", robotId);
                }
                else if(obj.robot_cluster[robotId].carried_item_type == 0)
                {
                    printf("buy %d\n", robotId);
                }
            }
        }
		printf("OK\n");
		fflush(stdout);
	}
	return 0;
}

                             

