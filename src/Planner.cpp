#include<iostream>
#include<iterator>
#include<math.h>
#include<algorithm>
#include<string>
#include<vector>
#include<map>
//#include <algorithm>
//include all head files 
#include "Planner.h" //include all functions
#include "Costs.h"
#include "Constant.h"
//using std::map;
using std::vector;
using std::string;
using std::cout;
using std::endl;
Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, double s, double d, double v, double a, string state){
    this->lane=lane;
    this->s=s;
    this->d=d;
    this->v=v;
    this->a=a;
    this->state=state;
    //max_accel=10; //define the max accel for vehicle, and move it to constant.h
}

Vehicle::~Vehicle(){}

double Vehicle::next_chosen_states(vector<vector<double>> detected_car_left, vector<vector<double>> detected_car_middle, vector<vector<double>> detected_car_right, double d_init, int pre_size){
    vector<string> states;
    states.push_back("KL");
    //string state=this->state;
    //Vehicle vel_ahead;
    if(current_d>8.0){
        states.push_back("LCL");
    }
    else if(current_d<4){
        states.push_back("LCR");
    }
    else{
        states.push_back("LCL");
        states.push_back("LCR");
    }
    //debug
    //std::cout<<"the size of states are: "<<states.size()<<std::endl;
    //debug
    for(unsigned i=0; i<states.size(); ++i){
        cout<<"The next available state is: "<<states[i]<<" "<<endl;
    }
    //cout<<"the available states are: "<<states[0]<<" "<<states[1]<<" "<<states[2]<<endl;;
    vector<double> costs;
    for(auto i=0; i<states.size(); ++i){
        //std::cout<<"start costs loop: "<<i<<endl;
        if(states[i].compare("KL")==0){
            if(current_d>8.0){
                double d_end=this->current_d;
                double current_cost=total_costs(detected_car_right, this->current_s, this->s, this->current_v, d_init, d_end, T_, pre_size);
                costs.push_back(current_cost);
            }
            else if(current_d>4.0){
                double d_end=this->current_d;
                double current_cost=total_costs(detected_car_middle, this->current_s, this->s,this->current_v, d_init, d_end,T_, pre_size);
                costs.push_back(current_cost);
            }
            else{
                double d_end=this->current_d;
                double current_cost=total_costs(detected_car_left, this->current_s, this->s,this->current_v, d_init, d_end,T_, pre_size);
                costs.push_back(current_cost);
            }
            
        }
        else if(states[i].compare("LCL")==0){
            if(current_d>8.0){
                double d_end=this->current_d-4.0;
                double current_cost=total_costs(detected_car_middle, this->current_s, this->s,this->current_v, d_init, d_end, T_, pre_size);
                costs.push_back(current_cost);                
            }
            else if(current_d>4.0){
                double d_end=this->current_d-4.0;
                double current_cost=total_costs(detected_car_left, this->current_s, this->s,this->current_v, d_init, d_end, T_, pre_size);
                costs.push_back(current_cost);   
            }
            
        }
        else if(states[i].compare("LCR")==0){
            if(current_d>4.0&&current_d<8){
                double d_end=this->current_d+4.0;
                double current_cost=total_costs(detected_car_right, this->current_s, this->s,this->current_v, d_init, d_end, T_, pre_size);
                costs.push_back(current_cost);
            }
            else if(current_d<4){
                double d_end=this->current_d+4.0;
                double current_cost=total_costs(detected_car_middle, this->current_s, this->s,this->current_v, d_init, d_end, T_, pre_size);
                costs.push_back(current_cost);
            }
            
        }
        
        //std::cout<<"the costs loop is completed: "<<endl;
        
    }
    //double best_cost;
    //debug
    for(unsigned i=0; i<costs.size(); ++i){
        cout<<"The cost for "<<states[i]<<" is: "<<costs[i]<<" "<<endl;
    }
    cout<<endl;
    string best_state; 
    vector<double>::iterator best_cost=std::min_element(begin(costs), end(costs));
    int best_idx=std::distance(begin(costs), best_cost);
    best_state=states[best_idx];
    std::cout<<"the Best state is: "<<best_state<<std::endl;
    //std::cout<<"the lane direction is: "<<lane_direction[best_state]<<std::endl;
    return lane_direction[best_state];
}




/* 
bool Vehicle::get_vehicle_ahead(vector<vector<double>> detected_car_middle, double pre_car_s, double pre_size){
    bool found=false;
    Vehicle temp_vehicle;
    int min_s=std::numeric_limits<int>::max();
    for(auto i=0;i<detected_car_middle.size();++i){
        //temp_vehicle=predictions[i];
        double Front_s=detected_car_middle[i][1];
        double Front_v=detected_car_middle[i][3];
        Front_s+=Front_v*TIMESTEP*pre_size;
        if((Front_s>pre_car_s)&&((Front_s-pre_car_s)<DIST_BUFFER)){
            found=true;
        }
        return found;
    }
    return found;
}
*/

vector<bool> Vehicle::detect_otherCar_beside(vector<vector<double>> &predictions){
    bool car_right=false, car_left=false, car_ahead=false;
    for(auto i=0;i<predictions.size();++i){
        double otherCar_d=predictions[i][6];
        double r_s=predictions[i][5];
        double d_diff=otherCar_d-this->d;
        if(d_diff>2&&d_diff<6&&(fabs(this->s-r_s))<SAFETYGAP){
            car_right=true;
        }
        else if(d_diff>-6&&d_diff<-2&&(fabs(this->s-r_s))<SAFETYGAP){
            car_left=true;
        }
        else if(d_diff>-2&&d_diff<2&&(r_s-this->s)<DIST_BUFFER){
            car_ahead=true;
        }
    }
    return {car_left, car_ahead, car_right};
}



