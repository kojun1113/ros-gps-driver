#pragma once

#include "ros/ros.h"
#include "nmea_msgs/Sentence.h" // use it 
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/NavSatStatus.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "tf2/LinearMath/Quaternion.h"

#include <iostream>
#include <vector>
#include <iterator>	
#include <string>
#include <sstream>
#include <algorithm>
#include <iomanip>
#include <vector>
#include <cctype>
#include <ctime>
#include <cmath>


class node_gps{

public:

    node_gps(ros::NodeHandle &nh);
    
    void load_param();
    void register_sub(); 
    void register_pub(); 
    void callback_sentence(const nmea_msgs::Sentence::ConstPtr &pmsg); 
    void publish_GGA(const ros::Time &utc_time,const double &latitude,const double &longitude,const int &fix_type,const double &hdop,const double &altitude);
    void publish_RMC(const ros::Time &utc_time,const bool &fix_status,const double &latitude,const double &longitude,const double &speed,const double &true_course);

    std::string &trim(std::string &s);
    bool data_check(std::string s);
    void data_parse();

    int safe_int(const std::string& s);
    double safe_double(const std::string& s);
    double deg2rad (double degrees);

private:
    ros::NodeHandle n;
    nmea_msgs::Sentence sentence;
    std::string s;

    double rate;
    int baud;
    std::string port;
    bool use_GNSS_time;

    ros::Publisher pub_fix;                                            
    ros::Publisher pub_vel;
    ros::Publisher pub_heading;

    ros::Subscriber sub_sentence;






};

