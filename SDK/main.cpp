#include "hwcompet.h"

using namespace std;

int main()
{
	hw_compet obj;
    pid_controller **ptr;
	obj.init();
	puts("OK");
	fflush(stdout);
    //std::cerr << "---------------------" << std::endl;
	while (scanf("%d", &obj.frame_num) != EOF) {
        obj.readUntilOK();
        printf("%d\n", obj.frame_num);
        for(int i = 0; i < 4; i++)
        {
            
            if(i == 1 && obj.map_id == 0 && obj.frame_num < 62) continue;
            double workbench_x = 25;
            double workbench_y = 25;
            //if(obj.robot_cluster[i].robot_execute_points.size() == 0) bool res = obj.robot_cluster[i].astar(&obj.robot_cluster[i].all_node[9790],&obj.robot_cluster[i].all_node[208]);
            //std::cerr<<obj.robot_cluster[i].robot_execute_points.size()<<std::endl;
            //std::cerr<<"****************************"<<obj.robot_cluster[i].robot_goal_point.size()<<std::endl;
            //workbench_x = obj.robot_cluster[i].robot_execute_points.top().x;
            //workbench_y = obj.robot_cluster[i].robot_execute_points.top().y;
            //std::cerr<<workbench_x<<"  "<<workbench_y<<std::endl;
            
            if(obj.robot_cluster[i].robot_goal_point.size() == 0) WorkBenchNodeForRobot target = obj.GetRobotTarget(obj.robot_cluster[i], i);
            //start planning
            if(obj.robot_cluster[i].robot_goal_point.size() != 0 && obj.robot_cluster[i].robot_execute_points.size() == 0){
                //start point == robot point
                obj.robot_cluster[i].node_tmp[0].y = (int)((obj.robot_cluster[i].location_y + 0.25)/0.5 - 1);
                obj.robot_cluster[i].node_tmp[0].x = (int)((obj.robot_cluster[i].location_x + 0.25)/0.5 - 1);
                //cerr<<obj.robot_cluster[i].node_tmp[0].x<<" &&&&&  "<<obj.robot_cluster[i].node_tmp[0].y<<endl;
                int start_node_num = obj.robot_cluster[i].node_tmp[0].y * 100 + obj.robot_cluster[i].node_tmp[0].x;
                
                
                //goal point == workbench point
                obj.robot_cluster[i].node_tmp[1].y = (int)((obj.robot_cluster[i].robot_goal_point.top().y + 0.25)/0.5 - 1);
                obj.robot_cluster[i].node_tmp[1].x = (int)((obj.robot_cluster[i].robot_goal_point.top().x + 0.25)/0.5 - 1);
                //std::cerr << obj.robot_cluster[i].robot_goal_point.top().x << "  ^^^^^^  " << obj.robot_cluster[i].robot_goal_point.top().y <<"  " << obj.robot_cluster[i].robot_goal_point.top().type <<endl;
                int end_node_num = obj.robot_cluster[i].node_tmp[1].y * 100 + obj.robot_cluster[i].node_tmp[1].x;
                
                //std::cerr << start_node_num << "  " << end_node_num << endl;
                //cerr<<" ******* "<<obj.robot_cluster[i].node_tmp[0].coordinate_x<<endl;
                bool res = obj.robot_cluster[i].astar(&obj.robot_cluster[i].all_node[start_node_num],&obj.robot_cluster[i].all_node[end_node_num]);
                //cerr<<obj.robot_cluster[i].robot_goal_point.top().x<<"  "<<obj.robot_cluster[i].robot_goal_point.top().y<<endl;
                //std::cerr<<"**************** "<<obj.robot_cluster[i].robot_execute_points.size()<<endl;
                //cerr<<"***************************"<<obj.robot_cluster[i].robot_execute_points.size()<<endl;
                workbench_x = obj.robot_cluster[i].robot_execute_points.top().x;
                workbench_y = obj.robot_cluster[i].robot_execute_points.top().y;
            }
            else if(obj.robot_cluster[i].robot_execute_points.size() != 0){
                workbench_x = obj.robot_cluster[i].robot_execute_points.top().x;
                workbench_y = obj.robot_cluster[i].robot_execute_points.top().y;
            }
            
            obj.get_yaw_angle(obj.distance_target,obj.yaw,obj.vector_angle,workbench_x,workbench_y,obj.robot_cluster[i]);
            ptr = obj.control_ptr;
            obj.vel_cmd_out(ptr,obj.angular_speed,obj.linear_speed,obj.yaw,obj.vector_angle,obj.distance_target, obj.map_id);	
            ptr++;
            obj.Deal_Clash(i);
            printf("forward %d %f\n", i, obj.linear_speed);
            printf("rotate %d %f\n", i, obj.angular_speed);

            
            if(obj.distance_target < 0.2) {
                obj.robot_cluster[i].robot_execute_points.pop();
            }

        }
        //std::cerr<<obj.distance_target<<std::endl;
        
        
        
        
        
        //购买和卖的逻辑
        
        for(int robotId = 0; robotId < 4; robotId++)
        {
            if(obj.robot_cluster[robotId].robot_execute_points.size() == 0){
                if(obj.robot_cluster[robotId].robot_goal_point.size() == 0) continue;
                int robot_workbench_id = obj.robot_cluster[robotId].workstation_id;
                int target_id = obj.robot_cluster[robotId].robot_goal_point.top().global_id;
                int status = obj.robot_cluster[robotId].robot_goal_point.top().product_status;
                //随后执行动作
                double pos_x = obj.robot_cluster[robotId].robot_goal_point.top().x;
                double pos_y = obj.robot_cluster[robotId].robot_goal_point.top().y;
                double erase_num = pos_x * 100 + pos_y;
                //注意，对于4-9的情况，这个类型里面是并不是4-9本身，而是它里面的产品格类型
                int type = obj.robot_cluster[robotId].robot_goal_point.top().type;
                    //判断是否在目标工作台附近
                if(target_id != robot_workbench_id) continue;
                else
                {
                    //判断在目标工作台附近后，通过判断小车是否携带物品确定，小车是要sell还是要buy
                    if(obj.robot_cluster[robotId].carried_item_type != 0)
                    {
                        //将目标点从target_set中erase，并更新robot_goal_poin队列
                        obj.robot_cluster[robotId].target_set.erase({erase_num, type});
                        obj.robot_cluster[robotId].robot_goal_point.pop();
                        printf("sell %d\n", robotId);
                    }   
                    else if(obj.robot_cluster[robotId].carried_item_type == 0 && obj.work_bench_cluster[robotId][type].GetProductStatus(pos_x, pos_y) == 1) 
                    {
                        //快结束就别买啦--最后4s
                        if(obj.frame_num > 8850) continue;
                        obj.robot_cluster[robotId].target_set.erase({erase_num, type});
                        obj.robot_cluster[robotId].robot_goal_point.pop();
                        //cerr<<obj.robot_cluster[robotId].robot_goal_point.top().x << " %%%%%% " << obj.robot_cluster[robotId].robot_goal_point.top().y<<endl;
                        printf("buy %d\n", robotId);
                    }
                }   
            }
        }
		printf("OK\n");
		fflush(stdout);
	}
	return 0;
}

                             

