#include "hwcompet.h"

using namespace std;

int main()
{
	hw_compet obj;
    pid_controller **ptr;
	obj.init();
	puts("OK");
	std::fflush(stdout);
    int global_id_start;
	while (scanf("%d", &obj.frame_num) != EOF) {
        obj.readUntilOK();
        std::printf("%d\n", obj.frame_num);
        for(int i = 0; i < 4; i++)
        {
            //std::cerr<<"*****************     "<<obj.work_bench_cluster[0][1].WorkBenchVec[0].workbench_route_workbench.size()<<std::endl;
            //if(i == 1 && obj.map_id == 0 && obj.frame_num < 62) continue;
            double workbench_x = 25;
            double workbench_y = 25;
            if(obj.robot_cluster[i].robot_goal_point.size() == 0){
                WorkBenchNodeForRobot target = obj.GetRobotTarget(obj.robot_cluster[i], i);
                /*if(obj.robot_cluster[i].robot_goal_point.size() == 0){
                    cerr<<i<<"  66666 "<<workbench_x<<"    "<<workbench_y<<endl;
                    workbench_x = obj.robot_cluster[i].location_x;
                    workbench_y = obj.robot_cluster[i].location_y;
                }*/
            }
            //cerr<<"????????????????????"<<obj.robot_cluster[i].robot_goal_point.size()<<endl;
            //start planning
            if(obj.robot_cluster[i].robot_goal_point.size() != 0 && obj.robot_cluster[i].robot_execute_points.size() == 0){
                //start point == robot point
                obj.robot_cluster[i].node_tmp_empty[0].y = (int)((obj.robot_cluster[i].location_y + 0.25)/0.5 - 1);
                obj.robot_cluster[i].node_tmp_empty[0].x = (int)((obj.robot_cluster[i].location_x + 0.25)/0.5 - 1);
                int start_node_num = obj.robot_cluster[i].node_tmp_empty[0].y * 100 + obj.robot_cluster[i].node_tmp_empty[0].x;
                //end_point
                obj.robot_cluster[i].node_tmp_empty[1].y = (int)((obj.robot_cluster[i].robot_goal_point.top().y + 0.25)/0.5 - 1);
                obj.robot_cluster[i].node_tmp_empty[1].x = (int)((obj.robot_cluster[i].robot_goal_point.top().x + 0.25)/0.5 - 1);
                int end_node_num = obj.robot_cluster[i].node_tmp_empty[1].y * 100 + obj.robot_cluster[i].node_tmp_empty[1].x;
                /*if(i == 2){
                    cerr<<start_node_num<<" $$$$$$$$$$$$$$$$  "<<end_node_num<<endl;
                    cerr<<"obj.robot_cluster[i].carried_item_type is " <<obj.robot_cluster[i].carried_item_type <<endl;
                }*/
                if(obj.robot_cluster[i].carried_item_type == 0){
                    //cerr<<"***************"<<obj.robot_cluster[i].robot_goal_point.top().global_id<<endl;
                    //if(i == 2) cerr<<" %%%%%%%%%%%%%%%%%%% "<<global_id_start<<endl;
                    global_id_start = obj.robot_cluster[i].robot_goal_point.top().global_id;
                    //std::cerr<<"**************start "<<global_id_start<<"   "<<obj.robot_cluster[i].robot_goal_point.top().x<<"    "<<obj.robot_cluster[i].robot_goal_point.top().y<<endl;
                    
                    int point_nums = obj.astar(&obj.robot_cluster[i].all_node_empty[start_node_num], &obj.robot_cluster[i].all_node_empty[end_node_num], i, obj.robot_cluster[i].carried_item_type);
                    //if(i == 2) cerr<<"**********************  "<<point_nums<<endl;
                }
                else
                {
                    //if(i == 2 ) cerr<<"%%%%%%%%%%%%%%%%%%%global_id_start "<<global_id_start<<endl;
                    int point_nums = obj.astar(&obj.robot_cluster[i].all_node_object[start_node_num], &obj.robot_cluster[i].all_node_object[end_node_num], i, obj.robot_cluster[i].carried_item_type);
                    //cerr<<"^^^^^^^^^^^^^^^^^  "<<point_nums<<endl;
                }
                /*else{
                    int global_id_end = obj.robot_cluster[i].robot_goal_point.top().global_id;
                    //-------------------------
                    if(obj.robot_cluster[i].target_route_points == 0){
                        int point_nums = obj.astar(&obj.robot_cluster[i].all_node_object[start_node_num], &obj.robot_cluster[i].all_node_object[end_node_num], i, obj.robot_cluster[i].carried_item_type);
                    }
                    else{
                        int workbench_type_end_point = obj.robot_cluster[i].robot_goal_point.top().real_type;
                        //std::cerr<<"**************end "<<obj.robot_cluster[i].robot_goal_point.top().global_id<<std::endl;
                        for(auto &m : obj.robot_cluster[i].target_route_points[global_id_start][workbench_type_end_point]){
                            if(m[0][1] == obj.robot_cluster[i].robot_goal_point.top().global_id){
                                WorkBenchNodeForRobot point;
                                for(auto &n : m){
                                    point.x = n[2];
                                    point.y = n[3];
                                    obj.robot_cluster[i].robot_execute_points.push(point);
                                }
                            }
                        }     
                                                
                    }
                    
                }*/
                //std::cerr<<obj.robot_cluster[i].robot_execute_points.size()<<endl;
                workbench_x = obj.robot_cluster[i].robot_execute_points.top().x;
                workbench_y = obj.robot_cluster[i].robot_execute_points.top().y;
            }
            else if(obj.robot_cluster[i].robot_execute_points.size() != 0){
                workbench_x = obj.robot_cluster[i].robot_execute_points.top().x;
                workbench_y = obj.robot_cluster[i].robot_execute_points.top().y;
            }
            //else{
            //   workbench_x = obj.robot_cluster[i].location_x;
            //    workbench_y = obj.robot_cluster[i].location_y;
            //}
            obj.get_yaw_angle(obj.distance_target,obj.yaw,obj.vector_angle,workbench_x,workbench_y,obj.robot_cluster[i]);
            ptr = obj.control_ptr;
            obj.vel_cmd_out(ptr,obj.angular_speed,obj.linear_speed,obj.yaw,obj.vector_angle,obj.distance_target, obj.map_id);	
            ptr++;
            obj.Deal_Clash(i);
            //cerr<<i<<"   "<<obj.robot_cluster[i].robot_execute_points.size()<<endl;
            //std::cerr<<obj.linear_speed<<"  "<<obj.angular_speed<<std::endl;
            std::printf("forward %d %f\n", i, obj.linear_speed);
            std::printf("rotate %d %f\n", i, obj.angular_speed);
            if(obj.robot_cluster[i].robot_execute_points.size() > 1){
                if(obj.distance_target < 0.8) {
                    obj.robot_cluster[i].robot_execute_points.pop();
                }
            }
            else if(obj.robot_cluster[i].robot_execute_points.size() == 1){
                if(obj.distance_target < 0.2) {
                    obj.robot_cluster[i].robot_execute_points.pop();
                }
            }
        }

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
                        std::printf("sell %d\n", robotId);
                    }   
                    else if(obj.robot_cluster[robotId].carried_item_type == 0 && obj.work_bench_cluster[robotId][type].GetProductStatus(pos_x, pos_y) == 1) 
                    {
                        //快结束就别买啦--最后4s
                        //if(obj.frame_num > 8850) continue;
                        obj.robot_cluster[robotId].target_set.erase({erase_num, type});
                        //cerr<<"global_id is    "<<obj.robot_cluster[robotId].robot_goal_point.top().global_id<<endl;
                        obj.robot_cluster[robotId].robot_goal_point.pop();
                        //cerr<<"global_id is    "<<obj.robot_cluster[robotId].robot_goal_point.top().global_id<<endl;
                        //cerr<<obj.robot_cluster[robotId].robot_goal_point.top().x << " %%%%%% " << obj.robot_cluster[robotId].robot_goal_point.top().y<<endl;
                        std::printf("buy %d\n", robotId);
                    }
                }   
            }
        }
		std::printf("OK\n");
		std::fflush(stdout);
	}
	return 0;
}

                             

