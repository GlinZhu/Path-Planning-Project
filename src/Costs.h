#ifndef COSTS_H
#define COSTS_H

#include<iostream>
#include<map>
#include<string>
#include<vector>
#include<math.h>
//#include "Planner.h"
#include "Constant.h"
using std::vector;
using std::string;
using std::map;
// define sigmoid function
double sigmoid(double x){
    return 2.0/(exp(-x)+1)-1;
}


double cost_lane_change(double d_start, double d_end){

  //cout << setw(25) << "start of calculating cost of lane chagnge " << endl;
  if( abs(d_start - d_end)< 0.05 ){
    return 0.0;
  }
  else{
    return 1.0;
  }
}

double not_middle_lane(double car_d){
  double d_diff=pow((car_d-6),2);
  return sigmoid(d_diff);
}

// Define cost functions
double efficiency_cost(vector<vector<double>> detected_car_list, double current_s, double pre_s, double current_v, double duration, double pre_size){
  int front_idx=-1;
  double tar_car_v;
  if(detected_car_list.size()>0){
    for(auto i=0;i<detected_car_list.size();++i){
    double target_car_s=detected_car_list[detected_car_list.size()-i-1][1];
    double target_car_v=detected_car_list[detected_car_list.size()-i-1][3];
    target_car_s+=(double)pre_size*TIMESTEP*target_car_v;
    if((target_car_s-pre_s)>0&&(target_car_s-pre_s)<current_v*duration*2){
      if(front_idx==-1){front_idx=detected_car_list.size()-i-1;}
    }
    }
    
    if(front_idx!=-1){
      tar_car_v=detected_car_list[front_idx][3];
      //double s1=detected_car_list[front_idx+1][1];
      //double v1=detected_car_list[front_idx+1][3];
      if(tar_car_v>MAX_SPEED){
        tar_car_v=MAX_SPEED;
      }
    }
    else{
      tar_car_v=MAX_SPEED;
    }
  }
  else{
    tar_car_v=MAX_SPEED;
  }
  
  return 1-tar_car_v/MAX_SPEED;
  
}


double check_collision(vector<vector<double>> detected_car_list, double current_s, double current_v, double duration, int pre_size) {
  // check if there is car in the lane while doing lane changing
  int front_idx=-1;
  for(auto i=0;i<detected_car_list.size();++i){
    double target_car_s=detected_car_list[detected_car_list.size()-i-1][1];
    double target_car_v=detected_car_list[detected_car_list.size()-i-1][3];
    //target_car_s+=(double)pre_size*TIMESTEP*target_car_v;
    if((target_car_s-current_s)>0){
      if(front_idx==-1){front_idx=detected_car_list.size()-i-1;}
    }
  }
  //there are three cases where all cars in the front of ego_car, and all cars is behind of the ego_car, and ego car is in the middle of detected cars;
  if(front_idx==-1&&detected_car_list.size()!=0){
    double tar_car_s=detected_car_list[detected_car_list.size()-1][1];
    double tar_car_v=detected_car_list[detected_car_list.size()-1][3];
      //double tar_car_s=detected_car_list[0][1];
      //double tar_car_v=detected_car_list[0][3];
    //tar_car_s+=(double)pre_size*TIMESTEP*tar_car_v;
    if(tar_car_s+tar_car_v*duration+0.5*LCBuffer>current_s+current_v*duration){
      return 1.0; //collision
    }
    else{
      return 0.0;
    }
  }
  else if(front_idx==detected_car_list.size()-1&&detected_car_list.size()!=0){
    double tar_car_s=detected_car_list[detected_car_list.size()-1][1];
    double tar_car_v=detected_car_list[detected_car_list.size()-1][3];
    //tar_car_s+=(double)pre_size*TIMESTEP*tar_car_v;
    if(tar_car_s+tar_car_v*duration<current_s+current_v*duration+LCBuffer){
      return 1.0; //collision
    }
    else{
      return 0.0;
    }
  }
  else if(front_idx != -1){
    double s1 = detected_car_list[front_idx+1][1];
    double v1 = detected_car_list[front_idx+1][3];
    double s2 = detected_car_list[front_idx][1];
    double v2 = detected_car_list[front_idx][3];
    //s1+=(double)pre_size*TIMESTEP*v1;
    //s2+=(double)pre_size*TIMESTEP*v2;
    if( s1+v1*duration+0.8*LCBuffer > current_s+current_v*duration){
      //cout << "CAUTIOUS!!! " << s1 - car_s << " " << v1 - car_speed << endl;
      return 1.0;
    }
    else if( s2+v2*duration < current_s+current_v*duration+LCBuffer ){
      //cout << "CAUTIOUS!!! " << s2 - car_s << " " << v2 - car_speed << endl;
      return 1.0;
    }
    else {return 0.0;}
  }
  else{return 0.0;}
  

}


double total_costs(vector<vector<double>> detected_car_list, double current_s, double pre_s, double current_v, double d_init, double d_end, double duration, int pre_size){
  // define the weights for all cost functions
  double Check_collision=check_collision(detected_car_list, current_s, current_v, duration, pre_size);
  //std::cout<<"check if the collision function excuted"<<std::endl;
  double Collision_weight=1.5;    //1.11
  double speed_cost=efficiency_cost(detected_car_list, current_s, pre_s, current_v, duration, pre_size);
  double lane_change_weight=0.20;
  double lane_change=cost_lane_change(d_init, d_end);
  //std::cout<<"check if the efficiency function excuted"<<std::endl;
  double Speed_weight=0.9;
  double total_cost;
  double middle_lane=not_middle_lane(d_end);
  double middle_lane_weight=0.00;
  total_cost=Check_collision*Collision_weight+speed_cost*Speed_weight+lane_change_weight*lane_change+middle_lane_weight*middle_lane;
  //debug
  //std::cout<<"check if the total cost function excuted?"<<std::endl;
  return total_cost;
}


#endif
