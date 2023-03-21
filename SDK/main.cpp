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
        //std::cerr<<"ok"<<std::endl;
        obj.readUntilOK();
        printf("%d\n", obj.frame_num);
        for(int i = 0; i < 4; i++)
        {
            double workbench_x,workbench_y;
            if(obj.robot_cluster[i].robot_goal_point.size() == 0)
            {
                WorkBenchNodeForRobot target = obj.GetRobotTarget(obj.robot_cluster[i], i);
                //if(i == 0)
                //{
                //    std::cerr<<obj.robot_cluster[i].robot_goal_point.size()<<std::endl;
                //}
                if(obj.robot_cluster[i].robot_goal_point.size() == 0) {
                    workbench_x = 25;//target.x;
                    workbench_y = 25;//target.y;
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
                //判断产品状态是否为1
                //if(i==0)
                //    std::cerr<<workbench_x<<"  "<<workbench_y<<"   "<<obj.robot_cluster[i].robot_goal_point.top().product_status<<std::endl;
            }

			//double workbench_x = obj.robot_cluster[i].robot_goal_point.top()[0];
			//double workbench_y = obj.robot_cluster[i].robot_goal_point.top()[1];
            
            obj.get_yaw_angle(obj.distance_target,obj.yaw,obj.vector_angle,workbench_x,workbench_y,obj.robot_cluster[i]);
            ptr = obj.control_ptr;
            obj.vel_cmd_out(ptr,obj.angular_speed,obj.linear_speed,obj.yaw,obj.vector_angle,obj.distance_target, obj.map_id);	
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
            int target_id = obj.robot_cluster[robotId].robot_goal_point.top().global_id;
            int status = obj.robot_cluster[robotId].robot_goal_point.top().product_status;
            // //只有购买或者卖成功了才pop掉目标
            // if((obj.robot_cluster[robotId].carried_item_type != 0 && obj.robot_cluster[robotId].robot_goal_point.size() == 2)
            // || (obj.robot_cluster[robotId].carried_item_type == 0 && obj.robot_cluster[robotId].robot_goal_point.size() == 1)) {

            // }
            //随后执行动作
            double pos_x = obj.robot_cluster[robotId].robot_goal_point.top().x;
            double pos_y = obj.robot_cluster[robotId].robot_goal_point.top().y;
            double erase_num = pos_x * 100 + pos_y;
            //注意，对于4-9的情况，这个类型里面是并不是4-9本身，而是它里面的产品格类型
            int type = obj.robot_cluster[robotId].robot_goal_point.top().type;
            if(target_id != robot_workbench_id) continue;
            else
            {
                
                if(obj.robot_cluster[robotId].carried_item_type != 0)
                {
                    obj.robot_cluster[robotId].target_set.erase({erase_num, type});
                    obj.robot_cluster[robotId].robot_goal_point.pop();
                    printf("sell %d\n", robotId);
                }
                else if(obj.robot_cluster[robotId].carried_item_type == 0 && obj.work_bench_cluster[robotId][type].GetProductStatus(pos_x, pos_y) == 1) 
                {
                    //快结束就别买啦--最后4s
                    if(obj.frame_num > 8800) continue;
                    obj.robot_cluster[robotId].target_set.erase({erase_num, type});
                    obj.robot_cluster[robotId].robot_goal_point.pop();
                    printf("buy %d\n", robotId);
                }
                
                //cerr << obj.robot_cluster[robotId].target_set.size() << endl;
            }
        }
		printf("OK\n");
		fflush(stdout);
	}
	return 0;
}

                             

