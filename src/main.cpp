#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"


using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

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
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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
  
  int lane_no=1, prev_lane;
  double vel_car=0, accln_car=0;
  
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane_no, &vel_car, &accln_car, &prev_lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            
            //int prev_pts = 0;
            double req_speed = 49.5;  //// velocity in mile/hour, will me converted later to meters/second
            accln_car  = 0.02* 0.1;  //// dt * accleration in meters/(second)^2
            
            prev_lane = lane_no ;
            double other_cars_d, other_cars_s,other_cars_vel ;
            int other_cars_lane;
            //// Sensor-fusion data analysis,   data format for each car is: [ id, x, y, vx, vy, s, d].
            vector<bool> check_car = {false, false, false};   //// lane: (car-y/n, distance s)
            vector<double> checked_car_vel ={70.0, 70.0, 70.0};
            double max_value= 1000000.0;
            vector<double> checked_car_s = {max_value, max_value, max_value};
            
            vector<bool> check_rear_car = {false, false, false};   //// lane: (car-y/n, distance s)
            vector<double> checked_rear_car_vel ={70.0, 70.0, 70.0};
            vector<double> checked_rear_car_s = {0.0, 0.0, 0.0};
            for (unsigned int i = 0; i < sensor_fusion.size(); i++ ) {
              other_cars_d = sensor_fusion[i][6];
              other_cars_vel = distance(0,0,sensor_fusion[i][3], sensor_fusion[i][4]);
              other_cars_s = sensor_fusion[i][5];
              if (other_cars_d>0 && other_cars_d<4){
                other_cars_lane = 0;
              }
              else if (other_cars_d>=4 && other_cars_d<8){
                other_cars_lane = 1;
              }
              else if (other_cars_d>=8 && other_cars_d<12){
                other_cars_lane = 2;
              }
              //cout << "other_cars_lane: " << other_cars_lane << endl;
              //if (other_cars_lane==lane_no){
              if ((other_cars_s > car_s) &&  ((other_cars_s-car_s) < 30)){ //// ahead car is closer than 50 meters in fwd 
                if (other_cars_s <  checked_car_s[other_cars_lane]){  //// to check if this if the nearest other car in that lane
                  check_car[other_cars_lane] = true;
                  checked_car_s[other_cars_lane] = other_cars_s;
                  checked_car_vel[other_cars_lane] = other_cars_vel;
                  //cout << "other_cars_lane: "<< other_cars_lane <<", s: "<<checked_car_s[other_cars_lane]<<", o_vel: "<<checked_car_vel[other_cars_lane] << endl;
                }
              }
              else if ((other_cars_s < car_s) &&  ((car_s -other_cars_s) < 10) && (other_cars_lane!=lane_no)){   /// rear cars in other lanes
                //if (other_cars_vel > car_speed-30){      // checking faster rear cars
                if (other_cars_s >  checked_rear_car_s[other_cars_lane]){ //// to check if this if the nearest other car in that lane
                  check_rear_car[other_cars_lane] = true;
                  checked_rear_car_s[other_cars_lane] = other_cars_s;
                  checked_rear_car_vel[other_cars_lane] = other_cars_vel;
                  //cout << "REAR_cars_lane: "<< other_cars_lane <<", s: "<<checked_rear_car_s[other_cars_lane]<<", o_vel: "<<checked_rear_car_vel[other_cars_lane] << endl;
                }
              }
              //}
            
            }
            
            double buffer_distance = 5.0 + 0.45* sqrt(pow(car_speed,2) - pow(checked_car_vel[lane_no],2));  ////as buffer distance in terms of deviation of speed 
            double rear_car_dist = 7.0;
            //Behaviour planning
            if (check_car[lane_no]){  /// if there is a other car in my lane i am coming closer to
              if(lane_no==0 ){ //// if my car on left most lane
                if(not (check_car[lane_no+1] or  (car_s-checked_rear_car_s[lane_no+1])< rear_car_dist) ){  // choosing lane 1 as next lane
                  lane_no++;
                }
/*                 else if((not(check_car[lane_no+2] or  check_rear_car[lane_no+2])) and ((checked_car_s[lane_no+1]-car_s)> 20) and ((car_s-checked_rear_car_s[lane_no+1])> rear_car_dist)  ) {  // choosing lane 2 as next lane
                  lane_no +=2;
                  req_speed -=20; //// to avoid high normal accleration, by turning at slower speed, with faster decreasing speed
                  accln_car *=7;
                } */
                else if ((checked_car_s[lane_no]-car_s) < buffer_distance){   // vehicle is front of my and both other lanes also, reducing my speed
                  req_speed = checked_car_vel[lane_no];
                }
              }
              else if(lane_no==1){ //// if my car on center lane
                if(not (check_car[lane_no-1] or  ((car_s-checked_rear_car_s[lane_no-1])< rear_car_dist)) ){  // choosing lane 0 as next lane
                  lane_no--;
                }
                else if(not (check_car[lane_no+1] or  ((car_s-checked_rear_car_s[lane_no+1])< rear_car_dist)) ){  // choosing lane 2 as next lane
                  lane_no++;
                }
                else if ((checked_car_s[lane_no]-car_s) < buffer_distance){   // vehicle is front of my and both other lanes also, reducing my speed
                  req_speed = checked_car_vel[lane_no];
                }
              } 
              else if(lane_no==2){ //// if my car on right most lane
                if(not (check_car[lane_no-1] or  ((car_s-checked_rear_car_s[lane_no-1])< rear_car_dist)) ){  // choosing lane 1 as next lane
                  lane_no--;
                }
/*                 else if(not (check_car[lane_no-2] or  check_rear_car[lane_no-2]) and ((checked_car_s[lane_no-1]-car_s)> 20) and ((car_s-checked_rear_car_s[lane_no-1])> rear_car_dist) ){  // choosing lane 0 as next lane
                  lane_no -=2;
                  req_speed -=20; //// to avoid high normal accleration
                  accln_car *=7;
                } */
                else if ((checked_car_s[lane_no]-car_s) < buffer_distance){   // vehicle is front of my and both other lanes also, reducing my speed
                  req_speed = checked_car_vel[lane_no];
                }                
              }
            }
            //cout << "prev lane: "<< prev_lane << ", current lane: "<< lane_no  << ", req_speed: " << req_speed << endl;
                  
            ///// Waypoint Trajectory generation
            int prev_pts = previous_path_x.size();
            int idx_next;
            double x_gen, y_gen;
            double next_s, next_d, prev_theta;
            vector<double> sd;
            //cout << "print prev_pts: " << prev_pts << endl;
            
            vector<double> pt_x;
            vector<double> pt_y;
            
            if (prev_pts>3){
              for(int i = 0; i < prev_pts; i++){
                    next_x_vals.push_back(previous_path_x[i]);
                    next_y_vals.push_back(previous_path_y[i]);
/*                     if (i>0){
                      prev_theta = atan2(next_y_vals[i] - next_y_vals[i-1], next_x_vals[i] - next_x_vals[i-1]);
                      sd = getFrenet(next_x_vals[i], next_y_vals[i], prev_theta, map_waypoints_x, map_waypoints_y);
                      cout << "prev_x,y: " << next_x_vals[i] << ", " << next_y_vals[i] << " prev_s,d: " << sd[0] << ", " << sd[1] << endl;
                    }   */
              }
              prev_theta = atan2(next_y_vals[prev_pts-1] - next_y_vals[prev_pts-2], next_x_vals[prev_pts-1] - next_x_vals[prev_pts-2]);
              x_gen = next_x_vals[prev_pts-1];
              y_gen = next_y_vals[prev_pts-1];
              
              sd = getFrenet(x_gen, y_gen, prev_theta, map_waypoints_x, map_waypoints_y);
              next_s = sd[0];
            
              pt_x.push_back(next_x_vals[prev_pts-2]);
              pt_y.push_back(next_y_vals[prev_pts-2]);
              pt_x.push_back(x_gen);
              pt_y.push_back(y_gen);
            }
            else  // when not even 2 previous points are available
            {
              prev_theta = deg2rad(car_yaw);
              x_gen = car_x;
              y_gen = car_y;
              next_s = car_s;
              
              pt_x.push_back(car_x - cos(prev_theta));
              pt_y.push_back(car_y - sin(prev_theta));
              pt_x.push_back(car_x);
              pt_y.push_back(car_y);

            }
            
            next_d = 2 + lane_no*4;
            //creating spline class object
            //setting x,y points for spline curve generation
            //vector<double> prev_wp0 = getXY(next_s-30, 2 + prev_lane*4, map_waypoints_s, map_waypoints_x, map_waypoints_y); 
            //vector<double> nxt_wp_1 = getXY(next_s+15, (2 + (lane_no + prev_lane)*2), map_waypoints_s, map_waypoints_x, map_waypoints_y); 
            vector<double> nxt_wp1 = getXY(next_s+30, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y); 
            vector<double> nxt_wp2 = getXY(next_s+60, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> nxt_wp3 = getXY(next_s+90, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	
            //pt_x.push_back(nxt_wp_1[0]);
            pt_x.push_back(nxt_wp1[0]);
            pt_x.push_back(nxt_wp2[0]);
            pt_x.push_back(nxt_wp3[0]);

            //pt_y.push_back(nxt_wp_1[1]);
            pt_y.push_back(nxt_wp1[1]);
            pt_y.push_back(nxt_wp2[1]);
            pt_y.push_back(nxt_wp3[1]);
            
            
            ////converting to car coordinates
            for ( int i = 0; i < pt_x.size(); i++ ) {
              double shift_x = pt_x[i] - x_gen;
              double shift_y = pt_y[i] - y_gen;

              pt_x[i] = shift_x * cos(0 - prev_theta) - shift_y * sin(0 - prev_theta);
              pt_y[i] = shift_x * sin(0 - prev_theta) + shift_y * cos(0 - prev_theta);
            }
            
            tk::spline spln, spln_sx, spln_sy;
            spln.set_points(pt_x, pt_y);
            //spln.set_points({car_x, nxt_wp1[0], nxt_wp2[0], nxt_wp3[0]}, {car_y, nxt_wp1[1], nxt_wp2[1], nxt_wp3[1]});
            //considering spline for 5 points
            //spln_sx.set_points({next_s, next_s+15, next_s+30, next_s+60, next_s+90}, {x_gen, nxt_wp_1[0], nxt_wp1[0], nxt_wp2[0], nxt_wp3[0]});
            //spln_sy.set_points({next_s, next_s+15, next_s+30, next_s+60, next_s+90}, {y_gen, nxt_wp_1[1], nxt_wp1[1], nxt_wp2[1], nxt_wp3[1]});
            
            //spln_sx.set_points({next_s-30, next_s, next_s+30, next_s+60, next_s+90}, {prev_wp0[0], x_gen, nxt_wp1[0], nxt_wp2[0], nxt_wp3[0]});
            //spln_sy.set_points({next_s-30, next_s, next_s+30, next_s+60, next_s+90}, {prev_wp0[1], y_gen, nxt_wp1[1], nxt_wp2[1], nxt_wp3[1]});
            //spln_sx.set_points({next_s, next_s+30, next_s+60, next_s+90}, {x_gen, nxt_wp1[0], nxt_wp2[0], nxt_wp3[0]});
            //spln_sy.set_points({next_s, next_s+30, next_s+60, next_s+90}, {y_gen, nxt_wp1[1], nxt_wp2[1], nxt_wp3[1]});
            //prev_pts = 0;
            
            double dist_inc = 0.45;
            //cout << "Print trajector points:" << endl;
            double car_c_x = 0.0, car_c_y = 0.0;
            double comp_vel = (30/sqrt(900 + pow(spln(30),2))); //   to get the component of velocity in the direction of x
            for(int i = 0; i < 50-prev_pts; i++)
            {
              double req_speed_conv = (0.02*req_speed/2.25);   ////converting speed to meters/second and multplying with time internal between 2 trajectory points generation
              if (vel_car >= req_speed_conv){
                if (fabs(vel_car - req_speed_conv ) > accln_car){
                  vel_car -= accln_car;
                }
                else{
                  vel_car = req_speed_conv;
                }
              }
              else{
                vel_car += accln_car;
              } 

              car_c_x += comp_vel * vel_car; ////  the component of velocity in the direction of x
              car_c_y = spln(car_c_x);
              //cout << "car c x,y: " << car_c_x << ", " << car_c_y << endl;
              //////converting car coordinates back to global coordinates
              double global_x = car_c_x * cos(prev_theta) - car_c_y * sin(prev_theta) + x_gen;
              double global_y = car_c_x * sin(prev_theta) + car_c_y * cos(prev_theta) + y_gen;
              
              next_x_vals.push_back(global_x);
              next_y_vals.push_back(global_y);
              
              //x_gen += dist_inc;  //increamenting x of global coordinates, like this and generating y from spline is giving error in d. 
              //next_y_vals.push_back(spln(x_gen));
              

              ////print new generated x,y and s,d 
/*               if (i>=1){
                double theta = atan2(next_y_vals[prev_pts+i] - next_y_vals[prev_pts+i-1], next_x_vals[prev_pts+i] - next_x_vals[prev_pts+i-1]);
                sd = getFrenet(next_x_vals[prev_pts+i], next_y_vals[prev_pts+i], theta, map_waypoints_x, map_waypoints_y);
                cout <<"i: "<< i<< ", x,y_gen:" << next_x_vals[prev_pts+i] << ", " << next_y_vals[prev_pts+i] <<" new_s,d: " << sd[0] << ", " << sd[1] << ", vel:" <<vel_car<< endl;
              } */
              
              // test straight line motion    
              //next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
              //next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
            }

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
