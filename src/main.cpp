#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <math.h>

#include "Planner.h" //include the planner functions
#include "Constant.h"
#include "Prediction.h"
// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;
//initialize variables
double max_s=6945.554;
double lane_width=4.0;
double car_s_init=1.249392e+02;
double car_d_init=6.164833e+00;
double car_v_init=0.0;
double car_v_end=MAX_SPEED;
double  car_d_init_global = car_d_init;
double car_d_end_global = car_d_init;
bool lane_keep=true;
bool lane_change=true;
vector<double> s_history, d_history;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0


  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  

  double ref_v=0.5;
  int lane=1;
  //create a class
  Vehicle ego_vehicle=Vehicle();

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ego_vehicle, &ref_v, &lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          //car_speed=car_speed/2.23694;
          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
          //cout<<"the size of sensor fusion is :"<<sensor_fusion.size()<<endl;
          //cout<<"sensor fusion id:"<<sensor_fusion[4][0]<<" x:"<<sensor_fusion[4][1]<<" y:"<<sensor_fusion[4][2]<<" vx:"<<sensor_fusion[4][3]<<" vy:"<<sensor_fusion[4][4]<<" s:"<<sensor_fusion[4][5]<<" d:"<<sensor_fusion[4][6]<<endl;
          //cout<<"sensor fusion id:"<<sensor_fusion[0][6]<<" x:"<<sensor_fusion[2][6]<<" y:"<<sensor_fusion[4][6]<<" vx:"<<sensor_fusion[6][6]<<" vy:"<<sensor_fusion[8][6]<<" s:"<<sensor_fusion[10][6]<<" d:"<<sensor_fusion[11][6]<<endl;
          
          int pre_size=previous_path_x.size();
          //cout<<"===============start the code==============================="<<endl;
          //cout<<"the size of previous path: "<<pre_size<<endl;
          int avail_path=std::min(MAX_PATH_KEPT, pre_size);
          //cout<<"the size of avail path: "<<avail_path<<endl;
          json msgJson;
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          vector<vector<double>> detected_car_left, detected_car_middle, detected_car_right;
          double pre_s_pos, pre_s_pos2, pre_d_pos, pre_d_pos2, pre_car_v, pre_car_v2, pre_d_dot, pre_d_dot2, pre_d_ddot, pre_car_accel, pre_car_angle;
          double current_car_s, current_car_d, current_car_v;
          current_car_s=car_s;
          current_car_d=car_d;
          current_car_v=car_speed;
          //std::cout<<"the data type of sensor fusion"<<typeid(sensor_fusion).name()<<"\n";
          /**
           * define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
  
          //cout<<"after assign the current state to Vehicle variables"<<endl;
//========initialize the vehicle state
          int ego_lane;
          ego_lane=LaneDetect(car_d);
          double pre_car_x, pre_car_y, pre_car_x1, pre_car_y1;
          double car_s_pre, car_d_pre;
          double ref_yaw=deg2rad(car_yaw);
          if(pre_size<2){
            
            pre_car_x=car_x-cos(ref_yaw);
            pre_car_y=car_y-sin(ref_yaw);
            car_s_pre=car_s_init;
            car_d_pre=car_d_init;
          }
          else{
            pre_car_x=previous_path_x[pre_size-1];
            pre_car_y=previous_path_x[pre_size-2];
            pre_car_x1=previous_path_y[pre_size-1];
            pre_car_y1=previous_path_y[pre_size-2];
            ref_yaw=atan2(pre_car_y-pre_car_y1, pre_car_x-pre_car_x1);
            car_s_pre=end_path_s;
            car_d_pre=end_path_d;
          }
          

          ego_vehicle.lane=ego_lane;
          ego_vehicle.s=car_s_pre;
          ego_vehicle.d=car_d_pre;
          //ego_vehicle.v=pre_car_v;
          //ego_vehicle.a=pre_car_accel;
          //ego_vehicle.d_dot=pre_d_dot;
          //ego_vehicle.d_ddot=pre_d_ddot;
          //ego_vehicle.state="CS";
          ego_vehicle.target_v=MAX_SPEED;
          ego_vehicle.current_s=current_car_s;
          ego_vehicle.current_v=current_car_v;
          ego_vehicle.current_d=current_car_d;
          cout<<"The current s: "<<current_car_s<<" the current v: "<<current_car_v<<" the current d is: "<<current_car_d<<endl;;
          // Pre-process the sensor fusion
          sensor_processing(sensor_fusion, detected_car_left, detected_car_middle, detected_car_right);
          //cout<<"after sensor fusion processing"<<endl;
          //cout<<"the size of detected_car_left is: "<<detected_car_left.size()<<endl;
          //cout<<"the size of detected_car_middle is: "<<detected_car_middle.size()<<endl;
          //cout<<"the size of detected_car_right is: "<<detected_car_right.size()<<endl;
          double best_state=ego_vehicle.next_chosen_states(detected_car_left, detected_car_middle, detected_car_right, current_car_d, pre_size);
          //cout<<"Next_chosen states function is passed"<<endl;
          double v_init = car_v_init; double v_end = car_v_end;
          //double d_init = car_d_init_global; double d_end = car_d_end_global;
          double duration=T_;
          bool sensor=false;
          if(sensor)
            {  cout <<"============Sensor Info for Right Lane: Difference, car_d, car_v====================" << endl;
            for(int i=0; i<detected_car_right.size(); i++){
            //for(int j=0; j<4; j++){ cout << sensor_car_list_right[i][j] << " ";}
              cout << detected_car_right[i][1] - car_s<< " " <<detected_car_right[i][2]<< " " <<detected_car_right[i][3];
              cout << endl;
            }
            cout << "====================Sensor Info for Mid Lane========================" << endl;
            for(int i=0; i<detected_car_middle.size(); i++){
              //for(int j=0; j<4; j++){cout << sensor_car_list_mid[i][j] << " ";}
              cout << detected_car_middle[i][1] - car_s << " " <<detected_car_middle[i][2]<< " " <<detected_car_middle[i][3];
              cout << endl;
              }
            cout << "========================Sensor Info for Left Lane=======================" << endl;
            for(int i=0; i<detected_car_left.size(); i++){
            //for(int j=0; j<4; j++){ cout << sensor_car_list_left[i][j] << " ";}
              cout << detected_car_left[i][1] - car_s<< " " <<detected_car_left[i][2]<< " " <<detected_car_left[i][3];
              cout << endl;
              }}
            //cout<<"The current lane is: "<<ego_lane<<endl;

          bool car_ahead=false;
          bool LaneChange=false, TooClose=false;
          vector<bool> car_detected;
          double left_tar_v,right_tar_v, ahead_tar_v;
          bool car_left0=false, car_left1=false, car_left2=false;
          bool left_right0=false, car_right1=false, car_right2=false;
          // define new method
          // detect the vehicle aside
          
          //get_car_aside(detected_car_left, car_left, pre_size, car_s_pre);
          //if(car_left){
          //  cout<<"car on the left!!"<<endl;
          //}
          //detect the cars on the right
          //get_car_aside(detected_car_right, car_right, pre_size, car_s_pre);
          //if(car_right){
          //  cout<<"car on the right!!"<<endl;
          //}
        
        if(best_state>0.0){
          cout<<std::setw(25)<<"======================Ready to change to right lane===================="<<endl;
          //LaneChange=true;
        }
        else if(best_state<0.0){
          cout<<std::setw(25)<<"========================Ready to change to left lane======================="<<endl;
          //LaneChange=true;
        }
        else{
          cout<<std::setw(25)<<"=========================Lane Following=============================="<<endl;
        }
        
          // define behavior
    
    
        int lane_end=1;
        double speed_diff=0.25;
        double ACC=0.28;
        double car_ahead_s=std::numeric_limits<double>::max();
        double Max_accel=2;
        //double Kp(0.05), Ki(0.001);
        
        if(best_state==0.0||ref_v<13){
          if(car_d>0&&car_d<4){
          ahead_tar_v=get_car_ahead(detected_car_left, car_ahead_s, TooClose, pre_size, car_s_pre);
          get_car_aside(detected_car_middle, car_right1, pre_size, car_s_pre);
          
            if(TooClose){
              cout<<"==============Slowing down! Too close！==================="<<endl;
              ref_v-=speed_diff+0.05;
              //double s_diff=car_ahead_s-current_car_s;
              //double v_diff=(ahead_tar_v-car_speed/2.24);
              //ref_v-=0.01*s_diff-0.005*v_diff;
              if(ref_v>ahead_tar_v){
                ref_v-=speed_diff;
              }
              else{
                ref_v+=speed_diff;
              }
            }
            else if(ref_v<MAX_SPEED){
              ref_v+=ACC;
              cout<<"=============Speed up ！====================="<<endl;
            }
            }
            else if(car_d>4&&car_d<8){
              ahead_tar_v=get_car_ahead(detected_car_middle, car_ahead_s, TooClose, pre_size, car_s_pre);
              get_car_aside(detected_car_left, car_left0, pre_size, car_s_pre);
              get_car_aside(detected_car_right, car_right2, pre_size, car_s_pre);
              if(TooClose){
              cout<<"================Slowing down! Too close！====================="<<endl;
              //double s_diff=car_ahead_s-current_car_s;
              //double v_diff=(ahead_tar_v-car_speed/2.24);
              //ref_v-=0.01*s_diff-0.005*v_diff;
              ref_v-=speed_diff+0.05;
              if(ref_v>ahead_tar_v){
                ref_v-=speed_diff;
              }
              else{
                ref_v+=speed_diff;
              }
            }
            else if(ref_v<MAX_SPEED){
              ref_v+=ACC;
              cout<<"==================Speed up！==========================="<<endl;
            }
            }
            else if(car_d>8){
              ahead_tar_v=get_car_ahead(detected_car_right, car_ahead_s, TooClose, pre_size, car_s_pre);
              get_car_aside(detected_car_middle, car_left1, pre_size, car_s_pre);
              if(TooClose){
                ref_v-=speed_diff+0.05;
                cout<<"================Slowing down! Too close=====================！"<<endl;
                //double s_diff=car_ahead_s-current_car_s;
                //double v_diff=(ahead_tar_v-car_speed/2.24);
                //ref_v-=0.01*s_diff-0.005*v_diff;
                if(ref_v>ahead_tar_v){
                ref_v-=speed_diff;
                }
                else{
                ref_v+=speed_diff;
                }
            }
            else if(ref_v<MAX_SPEED){
              ref_v+=ACC;
              cout<<"====================Speed up！============================"<<endl;
              }
            }
          }
        else if(best_state<0.0){
          get_car_aside(detected_car_left, car_left0, pre_size, car_s_pre);
          get_car_aside(detected_car_middle, car_left1, pre_size, car_s_pre);
          //get_car_aside(detected_car_right, car_left2, pre_size, car_s_pre);
          if(car_d>8&&!car_left1){
            lane=1;
            cout<<"Turn Left!, No car on the left"<<endl;
          }
          else if(car_d>4&&car_d<8&&!car_left0){
            lane=0;
            cout<<"Turn Left!, No car on the left"<<endl;
          }
          
          LaneChange=true;
          
        }
        else if(best_state>0.0){
          get_car_aside(detected_car_middle, car_right1, pre_size, car_s_pre);
          get_car_aside(detected_car_right, car_right2, pre_size, car_s_pre);
          if(car_d<4&&!car_right1){
            lane=1;
            cout<<"Turn Right!, No car on the Right"<<endl;
          }
          else if(car_d>4&&car_d<8&&!car_right2){
            lane=2;
            cout<<"Turn Right!, No car on the Right"<<endl;
          }
          
        }
          //||((2.0+4.0*(lane+1))-current_car_d)>0.75
        //||get_lane_val(car_d)!=(ego_lane-1)
        //||get_lane_val(car_d)!=(ego_lane+1)
        cout<<" "<<endl;
        cout<<"The current Lane is: "<<get_lane_val(current_car_d)<<" ================> Target Lane: "<<lane<<endl;
        cout<<endl;
        //cout<<"The Target Lane is: "<<lane<<endl;
 
          




/* ============================Trajectory generation ==========================================*/
          if(pre_size>0){
            car_s=end_path_s;
          }

	        vector<double> ptsx;
          vector<double> ptsy;
          double ref_x, ref_y, ref_x_pre, ref_y_pre;
          double pre_s, prev_d;
          ref_x=car_x;
          ref_y=car_y;
          //pre_s=s_pos-car_V*TIMESTEP;
          vector<double> pt_s;
          ref_yaw=deg2rad(car_yaw);
          //cout<<"the size of previous path: "<<pre_size<<endl;

          if(pre_size<2){
            ref_x_pre=car_x-cos(ref_yaw);
            ref_y_pre=car_y-sin(ref_yaw);
            ptsx.push_back(ref_x_pre);
            ptsx.push_back(car_x);
            ptsy.push_back(ref_y_pre);
            ptsy.push_back(car_y);
            //double pre_s=s_pos-1.0;
            //pt_s.push_back(pre_s);
            //pt_s.push_back(s_pos);
          }
          else{
            ref_x=previous_path_x[pre_size-1];
            ref_x_pre=previous_path_x[pre_size-2];
            ref_y=previous_path_y[pre_size-1];
            ref_y_pre=previous_path_y[pre_size-2];
            ref_yaw=atan2(ref_y-ref_y_pre, ref_x-ref_x_pre);
            ptsx.push_back(ref_x_pre);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_pre);
            ptsy.push_back(ref_y);
            //pt_s.push_back(pre_s);
            //pt_s.push_back(s_pos);
          }
          //cout<<"tets point 1"<<endl;
          //for(unsigned i=0;i<pt_s.size();++i){
          //  cout<<"The current pt_s is "<<pt_s[i]<<endl;
          //}
          
          //std::cout << std::setw(25) << "=======================Trajectory Generation ===============================" << std::endl;
          
          
          //creating a path
          //shifting to the vehicle coordinates
          //creating waypoints of a spline with 30m space evenly
          vector<double> WP1;
          vector<double> WP2;
          vector<double> WP3;
          vector<double> WP4;
          double d_target=2.0+4.0*lane;
          //cout<<"The target lane is :"<<lane<<endl;
          double current_s_pos=car_s;
          double s_next=current_s_pos+35;
          double s_next1=current_s_pos+60;
          double s_next2=current_s_pos+90;
          double s_next3=current_s_pos+120;
          WP1=getXY(s_next, d_target, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          WP2=getXY(s_next1, d_target, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          WP3=getXY(s_next2, d_target, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          WP4=getXY(s_next3, d_target, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(WP1[0]);
          ptsx.push_back(WP2[0]);
          ptsx.push_back(WP3[0]);
          ptsx.push_back(WP4[0]);
          //pt_s.push_back(s_next);
          //pt_s.push_back(s_next1);
          //pt_s.push_back(s_next2);
          ptsy.push_back(WP1[1]);
          ptsy.push_back(WP2[1]);
          ptsy.push_back(WP3[1]);
          ptsy.push_back(WP4[1]);

          //debug ================
          //for(unsigned i=0;i<pt_s.size();++i){
          //  cout<<"The current pt_s is "<<pt_s[i]<<endl;
          //}
          //for(unsigned i=0;i<ptsy.size();++i){
          //  cout<<"The current ptsy is "<<ptsy[i]<<endl;
          //}

          //=========================================

          
          
          // generate a trajectory using s_trajectory and pts_x as x, and y axis, and interpolated using increment_s_points
          if(ptsy.size()!=ptsx.size()){
            std::cout<<"error, there is mismatch between pt_s and ptsx"<<std::endl;
          }
          
          for(auto i=0;i<pre_size;++i){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          //debug
          /*
          for(unsigned i=0;i<pre_size;++i){
            cout<<"The previous path points are "<<previous_path_x[i]<<endl;
          }
          */

    


          for(auto i=0;i<ptsx.size();++i){
            double shifted_x=ptsx[i]-ref_x;
            double shifted_y=ptsy[i]-ref_y;
            ptsx[i]=cos(ref_yaw)*shifted_x+sin(ref_yaw)*shifted_y;
            ptsy[i]=sin(-ref_yaw)*shifted_x+cos(ref_yaw)*shifted_y;
          }
          tk::spline t;
          t.set_points(ptsx,ptsy);
  
          double target_x=40;
          double target_y;
          target_y=t(target_x);
          double tar_dist=distance(0, 0, target_x, target_y);
          double x_add_on(0);
          for(auto i=1;i<=80-pre_size;i++){
            double N=tar_dist/(ref_v*TIMESTEP/2.24);
            double x_point=x_add_on+target_x/N;
            double y_point=t(x_point);
            x_add_on=x_point;
            double shifted_x=x_point;
            double shifted_y=y_point;
            double new_x=cos(ref_yaw)*shifted_x-sin(ref_yaw)*shifted_y;
            double new_y=sin(ref_yaw)*shifted_x+cos(ref_yaw)*shifted_y;
            new_x+=ref_x;
            new_y+=ref_y;
            next_x_vals.push_back(new_x);
            next_y_vals.push_back(new_y);
          }

    
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}

