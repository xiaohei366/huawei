#ifndef Work_Bench_H
#define Work_Bench_H

#include <unordered_map>
#include <vector>
#include <iostream>
#include <stack>
#include <array>

struct WorkBenchNode {
    int global_id;
    int id;     
    int type;   
    double x, y; 
    int remain_production_time;
    int ori_material_status;
    int product_status;
    std::vector<std::vector<std::vector<std::array<double,4>>>> workbench_route_workbench;

    WorkBenchNode(int g_id, int Id, int Type, double X, double Y):global_id(g_id), id(Id), type(Type), x(X), y(Y),remain_production_time(0),ori_material_status(0),product_status(0){
        workbench_route_workbench = std::vector<std::vector<std::vector<std::array<double,4>>>>(10);
    };
    //vector<stack<pair<int, int>>>;
};



class WorkBench {
public:
    explicit WorkBench(int tp):type_(tp){}
    ~WorkBench() = default;
    std::vector<WorkBenchNode> WorkBenchVec;
    //初始化时加入工作台
    void Add(int g_id, double x, double y);
    void Update(double x, double y, int remain_production_time, int ori_material_status, int product_status);
    
    WorkBenchNode* Find(double x, double y);

    int GetSize() {return WorkBenchVec.size();};

    int GetProductStatus(double x, double y);
    






private:
    int type_;
    static constexpr int map_shape_ = 100;//地图大小，生成唯一坐标
    std::unordered_map<double, int> finder_;//x*100+y其一个位置对应一个id
};



#endif 
