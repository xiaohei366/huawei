#ifndef Robot_H
#define Robot_H

class Robot {
    public:
        explicit Robot(int x, int y):location_x(x), location_y(y) {}
        ~Robot() = default;


        void Updata(int workstation_id, int carried_item_type, float time_value_coefficient, float collision_value_coefficient, 
        float angular_velocity, float linear_velocity_x, float linear_velocity_y, float direction,
        float location_x, float location_y);






        //下面是机器人的属性
        int workstation_id;
        int carried_item_type;
        float time_value_coefficient;
        float collision_value_coefficient;
        float angular_velocity;
        float linear_velocity_x, linear_velocity_y;
        float direction;
        float location_x, location_y;

};

















#endif