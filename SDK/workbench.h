#ifndef Work_Bench_H
#define Work_Bench_H

#include <unordered_map>
#include <vector>

struct WorkBenchNode {
    int id;     
    int type;   
    double x, y; 
    int remain_production_time;
    int ori_material_status;
    int product_status;
    WorkBenchNode(int Id, int Type, int X, int Y):id(Id), type(Type), x(X), y(Y){};
};



class WorkBench {
public:
    explicit WorkBench(int tp):type_(tp){}
    ~WorkBench() = default;
    std::vector<WorkBenchNode*> WorkBenchVec;
    //初始化时加入工作台
    void Add(double x, double y);
    void Update(double x, double y, int remain_production_time, int ori_material_status, int product_status);
    
    WorkBenchNode* Find(double x, double y);

    int GetSize() {return WorkBenchVec.size();};







private:
    std::unordered_map<double, int> finder_;//x*100+y其一个位置对应一个id
    int type_;
    static constexpr int map_shape_ = 100;//地图大小，生成唯一坐标
};



#endif 
