#ifndef Robot_H
#define Robot_H

class Robot {
    public:
        explicit Robot(double x, double y):location_x(x), location_y(y) {}
        ~Robot() = default;


        void Update(int workstation_id, int carried_item_type, double time_value_coefficient, double collision_value_coefficient, 
        double angular_velocity, double linear_velocity_x, double linear_velocity_y, double direction,
        double location_x, double location_y);






        //下面是机器人的属性
        int workstation_id;
        int carried_item_type;
        double time_value_coefficient;
        double collision_value_coefficient;
        double angular_velocity;
        double linear_velocity_x, linear_velocity_y;
        double direction;
        double location_x, location_y;

};

















#endif
