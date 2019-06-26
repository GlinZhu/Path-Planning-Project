#ifndef PLANNER_H
#define PLANNER_H

#include<iostream>
#include<map>
#include<string>
#include<vector>

using std::vector;
using std::string;
using std::map;

class Vehicle{
    public:
    //constructor
    Vehicle();
    Vehicle(int lane, double s, double d, double v, double a, string state="KL");
    //destructor
    virtual ~Vehicle();

    //functions for behavior planner
    //construct functions for FSM
    vector<vector<double>> get_best_traj(vector<vector<double>> &predictions, double duration);
    double next_chosen_states(vector<vector<double>> detected_car_left, vector<vector<double>> detected_car_middle, vector<vector<double>> detected_car_right, double d_init, int pre_size);
    vector<bool> detect_otherCar_beside(vector<vector<double>> &predictions);
    map<string, int> lane_direction={{"LCL", -1}, {"LCR", 1}, {"KL", 0}};
    map<string, int> Road_lane={{"left", 0}, {"middle", 1}, {"right", 2}};
    struct collider{
        bool collision;
        float time;
    };

    //define all class member data
    int lane;
    double s, d, v, a, target_v, current_v, current_s, current_d;
    double d_dot, d_ddot;
    string state; 


};



#endif  // PLANNER_H
