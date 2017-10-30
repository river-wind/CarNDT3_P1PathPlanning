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

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
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




/////////////////////////////////////////////////////////////////////////////////////

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

  //start in lane 1
  int lane = 1;

  // Have a reference velocity to target
  double ref_vel = 0; //mph

  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {

    //bool too_close= false ;
    //double ref_vel = 0;
    //int lane=1;

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') 
    {
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
		double car_s_now = car_s;
		int prev_size = previous_path_x.size();

		if(prev_size > 0)
		{
		  car_s = end_path_s;
		}

		//std::cout<<"car_d ("<<car_d<<")  vs lane ("<<lane<<") = "<<fmod(round(car_d),2.0)<<std::endl;
  		//Check if too close to car in front
  		bool too_close = false; 
                bool left_lane_open = true;
                bool right_lane_open = true;
                bool just_changed = false;
		
		      double closest_front_L = 100000.0;
		      double closest_front_R = 100000.0;
		      double closest_rear_L = 100000.0;
		      double closest_rear_R = 100000.0;
		      double closest_front_Lv = 100000.0;
		      double closest_front_Rv = 100000.0;
		      double closest_rear_Lv = 100000.0;
		      double closest_rear_Rv = 100000.0;
		      double closest_front_Lfs = 100000.0;
		      double closest_front_Rfs = 100000.0;
		      double closest_rear_Lfs = 100000.0;
		      double closest_rear_Rfs = 100000.0;
		double too_close_speed = 1000.0;
		double too_close_sd = 30;
		//loop through sensor fusion data, AKA car list
		for (int i = 0; i < sensor_fusion.size(); i++)
		{
		  //car is in my lane
		  float d = sensor_fusion[i][6];
		  if(d < (2+4*lane+2) && d > (2+4*lane-2) )
		  {
		    double vx = sensor_fusion[i][3];
		    double vy = sensor_fusion[i][4];
		    double check_speed = sqrt(vx * vx + vy * vy);
		    double check_car_s = sensor_fusion[i][5];

		    check_car_s +=((double)prev_size*.02*check_speed);  //if using previous points can project s value out
		    //check s values greater than mine and s gap
		    if((check_car_s > car_s) && ((check_car_s - car_s) <30))
		    {

	              //Do some logic here, lower reference velocity so we don't crash into the car in front of us, could
		      //also flag to try to change lanes
		      //ref_vel = check_speed; //mph
		      
                      too_close = true;
		      too_close_speed = check_speed;
                      too_close_sd = check_car_s - car_s_now;
		    }
//Adding basic logic for lane changes

		      closest_front_L = 100000.0;
		      closest_front_R = 100000.0;
		      closest_rear_L = 100000.0;
		      closest_rear_R = 100000.0;
		      closest_front_Lv = 100000.0;
		      closest_front_Rv = 100000.0;
		      closest_rear_Lv = 100000.0;
		      closest_rear_Rv = 100000.0;
		      closest_front_Lfs = 100000.0;
		      closest_front_Rfs = 100000.0;
		      closest_rear_Lfs = 100000.0;
		      closest_rear_Rfs = 100000.0;


                      // loop through cars again to find any in adjacent lanes to avoid before trying to change lanes
                      for (int i2 = 0; i2 < sensor_fusion.size(); i2++)
                      {
                        float d2 = sensor_fusion[i2][6];
                                    
                        // check left lane for this car
                        if ((d2 < (2+4*(lane-1)+2.0)) && (d2 >(2+4*(lane-1)-2.0)))
                        {
                          double vx2 = sensor_fusion[i2][3];
                          double vy2 = sensor_fusion[i2][4];
                          double check_speed2 = sqrt(vx2*vx2+vy2*vy2);
                          double check_car_s2 = sensor_fusion[i2][5];
			  double check_car_s_future2 = check_car_s2 + (check_speed2*0.2);
			  double distance = 0;

			  //Check future positions of car, instead of the current positions:
			 


			  distance = abs(check_car_s2 - car_s_now);
			  if(car_s_now < check_car_s2)  //if check car is ahead of us
			  {
			    if(distance < closest_front_L)
			    {
			      closest_front_L = distance;
			      closest_front_Lv = check_speed2;
			      closest_front_Lfs = check_car_s_future2;
				//cout<<"closest front left: "<<closest_front_L;
			    }
			  }
			  else   //if check car is behind us
			  {
			    if(distance < closest_rear_L)
			    {
			      closest_rear_L = distance;
			      closest_rear_Lv = check_speed2;
			      closest_rear_Lfs = check_car_s_future2;
			    }
			  }
			  //std::cout<<"closest Left Lane distances - rear:"<<closest_rear_L<<" front:"<<closest_front_L << std::endl;
                          // predict where this car will be at the end of our path (based on front car projection)
                          //check_car_s_future2 += (double)prev_size*.02*check_speed2;
  
                          // check s values greater than mine and s gap 
                          //if ((check_car_s2 > car_s - 5) && ((check_car_s_future2 - car_s) < 40) && (check_car_s_future2 < check_car_s))
                          //{
                          //  left_lane_open = false;

                            //std::cout << "We're at s="<<car_s << "car to left at s="<<check_car_s2<<", not clear" <<  std::endl;
                          //}
                        }
                        // check right lane for this car
                        if ((d2 < (2+4*(lane+1)+2.0)) && (d2 >(2+4*(lane+1)-2.0)))
                        {
                          double vx2 = sensor_fusion[i2][3];
                          double vy2 = sensor_fusion[i2][4];
                          double check_speed2 = sqrt(vx2*vx2+vy2*vy2);
                          double check_car_s2 = sensor_fusion[i2][5];
			  double check_car_s_future2 = check_car_s2 + (check_speed2*0.2);

			  double distance = 0;

			  distance = abs(check_car_s2 - car_s_now);
			  if(car_s_now < check_car_s2)  //if check car is ahead of us
			  {
			    if(distance < closest_front_R)
			    {
			      closest_front_R = distance;
			      closest_front_Rv = check_speed2;
			      closest_front_Rfs = check_car_s_future2;
			      //cout<<"NEW closest front right: "<<closest_front_R;
			    }
			  }
			  else   //if check car is behind us
			  {
			    if(distance < closest_rear_R)
			    {
			      closest_rear_R = distance;
			      closest_rear_Rv = check_speed2;
			      closest_rear_Rfs = check_car_s_future2;
			    }
			  }

			  //std::cout<<"closest Right Lane distances - rear:"<<closest_rear_R<<" front:"<<closest_front_R<<std::endl;
                          // predict where this car will be in the future
                          //check_car_s_future2 += (double)prev_size*.02*check_speed2;

                          // check s values greater than mine and s gap 
                          //if ((check_car_s2 > car_s - 5) && ((check_car_s_future2 - car_s) < 40) && (check_car_s_future2 < check_car_s))
                          //{
                          //  right_lane_open = false;
                            //std::cout << "We're at s="<<car_s << "car to right at s="<<check_car_s2<<", not clear" <<  std::endl;
                          //}
			  //if((!left_lane_open) && (!right_lane_open)){break;}
                        }
                      }
                    //}
                  }
		}

std::cout<<closest_front_L<<"  :  "<<closest_front_R<<std::endl;
std::cout<<closest_rear_L<<"  :  "<<closest_rear_R<<std::endl<<std::endl;

		double costKL = 0.0;
		//if (too_close_speed!=1000.0){costKL += 1/too_close_speed;}
		costKL += 1/too_close_speed;
		double costR = 0.0;
		if (lane==2){costR += 1000;}
		           //if (closest_rear_R!=100000.0){costR += 1/closest_rear_R;}
		           //if (closest_front_R!=100000.0){costR += (1/closest_front_R);}
		costR += ((1/closest_front_R)*.20);
		costR += ((1/closest_front_Rv)*.80);
		           //if (closest_front_Rv!=100000.0){costR += (1/closest_front_Rv)*5;}
		if (closest_front_R < 25){costR += 1000;}
		if (closest_rear_R < 20){costR += 1000;}
		if (abs(closest_front_Rfs - (car_s_now+car_speed*0.2)) < 25){costR += 1000;}
		if (abs(closest_rear_Rfs - (car_s_now+car_speed*0.2)) < 20){costR += 1000;}
		double costL = 0.0;	
		if (lane==0){costL += 1000;}
		        //if (closest_rear_L!=100000.0){costL += 1/closest_rear_L;}
		        //if (closest_front_L!=100000.0){costL += (closest_front_L);}
		costL += ((1/closest_front_L)*.20);
		costL += ((1/closest_front_Lv)*.80);
		        //if (closest_front_Lv!=100000.0){costL += (1/closest_front_Lv)*5;}
		if (closest_front_L < 25){costL += 1000;}
		if (closest_rear_L < 20){costL += 1000;}
		if (abs(closest_front_Lfs - (car_s_now+car_speed*0.2)) < 25){costL += 1000;}
		if (abs(closest_rear_Lfs - (car_s_now+car_speed*0.2)) < 20){costL += 1000;}


		if(too_close){
	    	  //slow down
		  ref_vel -= .056;
		  if(too_close_sd < 10){ref_vel -= .224;}  //emergency braking
		  if(abs(car_speed - too_close_speed>5)){ref_vel+=0.056;}

		  //change lanes if it is cheaper then following

	          if((costR>0.0001 && costR!=1000) || (costL>0.0001 && costL!=1000))
	          {
  	            std::cout<<"costKL: "<<costKL<<" costR: "<<costR<<" costL:"<<costL<<std::endl;	
	            //if(closest_front_R<30 || closest_front_L<30) std::cout<<"closest front right: "<<closest_front_R<<" closest front left: "<<closest_front_L<<std::endl;
		    //if(closest_rear_R<30 || closest_rear_L<30) std::cout<<"closest rear right: "<<closest_rear_R<<" closest rear left: "<<closest_rear_L<<std::endl;
	            //std::cout<<"closest rear right dist: "<<closest_rear_R<<" closest rear left dist: "<<closest_rear_L<<std::endl;
	            //std::cout<<"closest rear right future dist: "<<abs(closest_rear_Rfs - (car_s_now+(car_speed*0.2)))<<" closest rear left future: "<<abs(closest_rear_Lfs-(car_s+(car_speed*0.2)))<<std::endl;
		  }
                  //if (lane>0 && left_lane_open && abs(car_d-(4*lane)-2)<1)
                  if(costL < costKL && costL < costR)
		  {
	            std::cout<<"lane "<<lane<<", changing lanes left"<<" delta d:"<<abs(car_d - (4*lane) - 2)<<std::endl;
                    lane--;
                  }
                  //else if (lane<2 && right_lane_open && abs(car_d-(4*lane)-2)<1)
		  else if (costR < costKL && costR < costL)
                  {
	            std::cout<<"lane "<<lane<<", changing lanes right"<<" delta d:"<<abs(car_d-(4*lane)-2)<<std::endl;
                    lane++;
                  }
                }
   	        else if (ref_vel < 49.5)
	        {
 	          ref_vel += .224;
	        }
		//more ideal would be to find the speed of the lane to be changed to, then slow to that speed, and then change when safe.                    
                
		  
//ending basic logic
                 

		vector<double> ptsx;
		vector<double> ptsy;

		double ref_x = car_x;
		double ref_y = car_y;
		double ref_yaw = deg2rad(car_yaw);

		if(prev_size < 2)
		{
		  //Use two points that make the path tanget to the car
		  double prev_car_x = car_x - cos(car_yaw);
		  double prev_car_y = car_y - sin(car_yaw);

		  ptsx.push_back(prev_car_x);
		  ptsx.push_back(car_x);

		  ptsy.push_back(prev_car_y);
		  ptsy.push_back(car_y);
		}
		//use the previous path's end point as starting reference
		else
		{
		  //Redefine reference state as previous path and point
		  ref_x = previous_path_x[prev_size-1];
		  ref_y = previous_path_y[prev_size-1];

		  double ref_x_prev = previous_path_x[prev_size-2];
		  double ref_y_prev = previous_path_y[prev_size-2];
		  ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

		  //Use two points that make the path tangent to the previous path's end point
		  ptsx.push_back(ref_x_prev);
		  ptsx.push_back(ref_x);

		  ptsy.push_back(ref_y_prev);
		  ptsy.push_back(ref_y);
		}

		vector<double> next_wp0 = getXY(car_s + 30, (2+4*lane), map_waypoints_s , map_waypoints_x, map_waypoints_y);
		vector<double> next_wp1 = getXY(car_s + 60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		vector<double> next_wp2 = getXY(car_s + 90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

		ptsx.push_back(next_wp0[0]);
		ptsx.push_back(next_wp1[0]);
		ptsx.push_back(next_wp2[0]);

		ptsy.push_back(next_wp0[1]);
		ptsy.push_back(next_wp1[1]);
		ptsy.push_back(next_wp2[1]);

		for(int i = 0; i < ptsx.size(); i++)
		{
		  //shift car reference angle to 0 degress
		  double shift_x = ptsx[i]-ref_x;
		  double shift_y = ptsy[i]-ref_y;

		  ptsx[i] = (shift_x *cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
		  ptsy[i] = (shift_x *sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
		}

		// create a spline
		tk::spline s;

		//set (x,y) points to the spline
		s.set_points(ptsx, ptsy);

		//start with all the previous path points from last time; helps to smooth new trajectories
		for(int i = 0; i<previous_path_x.size(); i++)
		{
		  next_x_vals.push_back(previous_path_x[i]);
		  next_y_vals.push_back(previous_path_y[i]);
		}
		
		//Calculate how to break up spline points so that we travel at our desired reference velocity
		double target_x = 30.0;
		double target_y = s(target_x);
		double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
		
		double x_add_on = 0;

		//Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
		for (int i = 1; i <= 50-previous_path_x.size(); i++) 
		{
		  double N = (target_dist / (0.02 * ref_vel / 2.24));
		  double x_point = x_add_on + (target_x)/N;
		  double y_point = s(x_point);

		  x_add_on = x_point;

		  double x_ref = x_point;
		  double y_ref = y_point;

		  //rotate back to normal after rotating it earlier
		  x_point = (x_ref *cos(ref_yaw) - y_ref * sin(ref_yaw));
		  y_point = (x_ref *sin(ref_yaw) + y_ref * cos(ref_yaw));

		  x_point += ref_x;
		  y_point += ref_y;

		  next_x_vals.push_back(x_point);
		  next_y_vals.push_back(y_point);
		}

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
    		/*double dist_inc = 0.5;
    		for(int i = 0; i < 50; i++)
    		{
		      double next_s = car_s + (i+1) * dist_inc;
		      double next_d = 6;
		      vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		      next_x_vals.push_back(xy[0]);
		      next_y_vals.push_back(xy[1]);

    		      //next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
    		      //next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
    		}*/

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
