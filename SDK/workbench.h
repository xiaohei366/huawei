#ifndef Work_Bench_H
#define Work_Bench_H

#include <unordered_map>
#include <vector>

struct WorkBenchNode {
    int id;     //冗余
    int type;   //冗余
    float x, y; //冗余
    int remain_production_time;
    int ori_material_status;
    int product_status;
    WorkBenchNode(int Id, int Type, int X, int Y):id(Id), type(Type), x(X), y(Y){};
};



class WorkBench {
public:
    explicit WorkBench(){}
    ~WorkBench() = default;
    std::vector<WorkBenchNode*> WorkBenchVec;
    //初始化时加入工作台
    void Add(int x, int y);
    void Update(int x, int y, int remain_production_time, int ori_material_status, int product_status);
		void InitWorkBenchPostion(WorkBenchNode * WB_ptr, int x_val, int y_val);
    WorkBenchNode* Find(int x, int y);

    int GetSize() {return WorkBenchVec.size();};







private:
    std::unordered_map<int, int> finder_;//x*100+y其一个位置对应一个id
    int type_;
    static constexpr int map_shape_ = 100;//地图大小，生成唯一坐标
};



#endif 
