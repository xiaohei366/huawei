#include "workbench.h"




void WorkBench::Add(int x, int y) {
    int id = WorkBenchVec.size();
    WorkBenchNode* node = new WorkBenchNode(id, this->type_, x, y);
    WorkBenchVec.push_back(node);
    finder_[x*map_shape_+y] = id;
}

WorkBenchNode* WorkBench::Find(int x, int y) {
    int id = x*map_shape_+y;
    return WorkBenchVec[finder_[id]];
}


void WorkBench::Update(int x, int y, int remain_production_time, int ori_material_status, int product_status) {
    WorkBenchNode* node = Find(x, y);
    node->remain_production_time = remain_production_time;
    node->ori_material_status = ori_material_status;
    node->product_status = product_status;
}

void WorkBench::InitWorkBenchPostion(WorkBenchNode * WB_ptr, int x_val, int y_val)
{
	WB_ptr->x = (float)x_val*0.5-0.25;
	WB_ptr->y = (float)y_val*0.5-0.25;
}
