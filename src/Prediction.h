#ifndef PREDICTION_H
#define PREDICTION_H

#include<iostream>
#include<map>
#include<string>
#include<vector>
#include<math.h>
using std::string;
using std::vector;
#include "spline.h"
#include "Constant.h"
// udpate sensor fusion list 
void sensor_processing(vector<vector<double>> sensor_fusion, vector<vector<double>> &sensor_car_list_left, vector<vector<double>> &sensor_car_list_mid, vector<vector<double>> &sensor_car_list_right){

  int sensor_list_size = sensor_fusion.size();

  //cout << "Total number of cars in sensor list: " << sensor_list_size << endl;
  for (int i=0; i<sensor_list_size; i++){
    //cout << sensor_fusion[i][0] << endl;
    double sensor_id = sensor_fusion[i][0];
    double sensor_vx = sensor_fusion[i][3];
    double sensor_vy = sensor_fusion[i][4];
    double sensor_s = sensor_fusion[i][5];
    double sensor_d = sensor_fusion[i][6];
    double sensor_v = sqrt(sensor_vx*sensor_vx+sensor_vy*sensor_vy);

    if (sensor_d>8.0){
      bool flag = true;
      for(int j=0; j<sensor_car_list_right.size(); j++){
        if(sensor_s>sensor_car_list_right[j][1]){
            sensor_car_list_right.insert(sensor_car_list_right.begin()+j, {sensor_id, sensor_s, sensor_d, sensor_v});
            flag = false;
            break;
        }
      }
      if (flag==true){sensor_car_list_right.push_back({sensor_id, sensor_s, sensor_d, sensor_v});}
    }
    else if(sensor_d>4.0){
      bool flag = true;
      for(int j=0; j<sensor_car_list_mid.size(); j++){
        if(sensor_s>sensor_car_list_mid[j][1]){
            sensor_car_list_mid.insert(sensor_car_list_mid.begin()+j, {sensor_id, sensor_s, sensor_d, sensor_v});
            flag = false;
            break;
        }
      }
      if (flag==true){sensor_car_list_mid.push_back({sensor_id, sensor_s, sensor_d, sensor_v});}
    }
    else{
      bool flag = true;
      for(int j=0; j<sensor_car_list_left.size(); j++){
        if(sensor_s>sensor_car_list_left[j][1]){
            sensor_car_list_left.insert(sensor_car_list_left.begin()+j, {sensor_id, sensor_s, sensor_d, sensor_v});
            flag = false;
            break;
        }
      }
      if (flag==true){sensor_car_list_left.push_back({sensor_id, sensor_s, sensor_d, sensor_v});}
    }
  }
    

}




int get_lane_val(double d){
    int lane;
    if(d>0&&d<4){
        lane=0;
    }
    else if(d>4&&d<8){
        lane=1;
    }
    else if(d>8&&d<12){
        lane=2;
    }
    return lane;
}

double get_car_ahead(vector<vector<double>> &detected_car_list, double &car_ahead_s, bool &flag, int pre_size, double car_s_pre){
    int front_idx=-1;
    double ahead_v;
    for(auto i=0;i<detected_car_list.size();++i){
    double target_car_s=detected_car_list[detected_car_list.size()-i-1][1];
    double target_car_v=detected_car_list[detected_car_list.size()-i-1][3];
    target_car_s+=(double)pre_size*TIMESTEP*target_car_v;
    if((target_car_s-car_s_pre)>0.05&&(target_car_s-car_s_pre)<DIST_BUFFER){
        if(front_idx==-1){
          front_idx=detected_car_list.size()-i-1;
          ahead_v=target_car_v;
          car_ahead_s=detected_car_list[front_idx][1];
          }
        }
    }
    if(front_idx==-1){  //menas that no cars in the front
        flag=false;
    }
    else if(front_idx!=-1&&detected_car_list.size()!=0){
        flag=true; // vehicel ahead is detected
    }
    return ahead_v;
        
}

void get_car_aside(vector<vector<double>> &detected_car_list, bool &flag, int pre_size, double car_s_pre){
   int right_idx=-1;
   double tar_v;
   double lane_buffer=10;
    for(auto i=0;i<detected_car_list.size();++i){
        double right_car_s=detected_car_list[detected_car_list.size()-i-1][1];
        double right_car_v=detected_car_list[detected_car_list.size()-i-1][3];
        right_car_s+=(double)pre_size*TIMESTEP*right_car_v;
        if(fabs(right_car_s-car_s_pre)<lane_buffer){
            if(right_idx==-1){right_idx=detected_car_list.size()-i-1;}
        }
    }
    if(right_idx==-1){
        flag=false;
    }
    else if(right_idx!=-1&&detected_car_list.size()!=0){
        flag=true;
        tar_v=detected_car_list[right_idx][3];
        //std::cout<<"car on the right"<<std::endl;
    }
        
}



vector<double> getXY_spline(double s, double d, tk::spline &s_x, tk::spline &s_y, tk::spline &s_dx, tk::spline &s_dy)
{
  double x = s_x(s) + d*s_dx(s);
  double y = s_y(s) + d*s_dy(s);

	return {x,y};
}



#endif