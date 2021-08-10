#pragma once

#include "ros/ros.h"
#include "nmea_msgs/Sentence.h" // use it 
#include "serial/serial.h"

#include <string>

class node_serial{

public:

    node_serial(ros::NodeHandle &nh);
    serial::Serial sp;
    nmea_msgs::Sentence sentence;

    void load_param();
    void register_pub(); 
    void publish_sentence(); 


    std::string &trim(std::string &s);

private:
    ros::NodeHandle n;
    

    int baud;
    std::string port;

    ros::Publisher pub_sentence;

    

};

