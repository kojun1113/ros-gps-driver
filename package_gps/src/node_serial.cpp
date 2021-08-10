

#include "ros/ros.h"
#include "nmea_msgs/Sentence.h"
#include "serial/serial.h"

#include "node_serial.h"

#include <string>



node_serial::node_serial(ros::NodeHandle &nh){
    n = nh;
}

void node_serial::load_param(){

    if(!n.param<std::string>("port",port,"/dev/ttyUSB0")){
        ROS_WARN_STREAM("load param error , default value is "<<"/dev/ttyUSB0");
    }
    
    if(!n.param<int>("baud",baud,4800)){
        ROS_WARN_STREAM("load param error , default value is "<<4800);
    }
    serial::Timeout to = serial::Timeout::simpleTimeout(1000); // ??
    sp.setPort(port);
    sp.setBaudrate(baud);
    sp.setTimeout(to); // 0.1s

}

void node_serial::register_pub(){
    pub_sentence = n.advertise<nmea_msgs::Sentence>("/gps/serial_sentence", 1);
}

void node_serial::publish_sentence(){
    pub_sentence.publish(sentence);
}









std::string& node_serial::trim(std::string &s){
	if (!s.empty()) {
		s = s.erase(0, s.find_first_not_of(" "));
		s = s.erase(s.find_last_not_of(" ") + 1);
	}
	else {
		return s;
	}
	// blank inside string 
	if (!s.empty()) {
		std::string::size_type i = 0;
		while ( (i = s.find(' ')) != std::string::npos ) {
			s.erase(i, 1);
		}
	}
	else {
		return s;
	}
	return s;
}
