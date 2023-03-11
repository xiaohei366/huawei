#include "workbench.h"




void WorkBench::Add(double x, double y) {
    int id = WorkBenchVec.size();
    WorkBenchNode* node = new WorkBenchNode(id, this->type_, x, y);
    WorkBenchVec.push_back(node);
    finder_[x*map_shape_+y] = id;
}

WorkBenchNode* WorkBench::Find(double x, double y) {
    double id = x*map_shape_+y;
    return WorkBenchVec[finder_[id]];
}


void WorkBench::Update(double x, double y, int remain_production_time, int ori_material_status, int product_status) {
    WorkBenchNode* node = Find(x, y);
    node->remain_production_time = remain_production_time;
    node->ori_material_status = ori_material_status;
    node->product_status = product_status;
}

