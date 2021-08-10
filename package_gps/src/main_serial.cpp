

#include "node_serial.h"

#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv,"node_serial"); // node

    ros::NodeHandle nh("~"); // node handler
    
    node_serial serial(nh);  

    serial.load_param();
    serial.register_pub();
        
    try{
        serial.sp.open();
    }
    catch(serial::IOException &e){
        ROS_WARN_STREAM("fail to open serial");
        return -1;
    }

    if(serial.sp.isOpen()){
        ROS_INFO("serial initialized");
    }
    else{
        ROS_WARN_STREAM("serial initialize fail, not open");
        return -1;           
    }

    try{

        while (serial.sp.isOpen()){
            std::string data;
            data = serial.sp.readline();
            data = serial.trim(data);

            // data decode('ascii') ???#############################################################################################
            serial.sentence.sentence = data;
            serial.sentence.header.frame_id="/gps/serial_sentence";
            serial.sentence.header.stamp=ros::Time::now();
        
            serial.publish_sentence();

            std::cout<<data<<std::endl;

        }
    }
    catch(...){                  // catch is a freaking hole
        ROS_WARN_STREAM("serial out");
        serial.sp.close();
        ROS_WARN_STREAM("serial is closed");
        return -1;
    }
        
    serial.sp.close();
    ROS_WARN_STREAM("serial is closed");
    

    return 0;
}












