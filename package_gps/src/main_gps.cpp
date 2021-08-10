
#include "node_gps.h"







int main(int argc, char **argv) {

    ros::init(argc, argv,"node_gps"); // node

    ros::NodeHandle nh("~"); // node handler

    node_gps gps(nh);

	gps.load_param();
	gps.register_sub();
	gps.register_pub();

	// main do in callback function in class node_gps
	ros::spin();


























	// std::string sentence[] = {
	// 	"$GPGGA,092927.000,2235.9058,N,11400.0518,E,2,9,1.03,53.1,M,-2.4,M,0.0,0*6B",
		
	// 	"$GPRMC,092927.000,A,2235.9058,N,11400.0518,E,0.000,74.11,151216,,D * 49",
	// 	"$GPVTG,74.11,T,,M,0.000,N,0.000,K,D * 0B",
	// 	"$GPGGA,092927.000,2235.9058,N,11400.0518,E,2,9,1.03,53.1,M,-2.4,M,0.0,0 * 6B",
		
	// 	"$GPGSA,A,3,29,18,12,25,10,193,32,14,31,,,,1.34,1.03,0.85 * 31",
	// 	"$GPGSV,3,1,12,10,77,192,17,25,59,077,42,32,51,359,39,193,49,157,36 * 48",
	// 	"$GPGSV,3,2,12,31,47,274,25,50,46,122,37,18,45,158,37,14,36,326,18 * 70",
	// 	"$GPGSV,3,3,12,12,24,045,45,26,17,200,18,29,07,128,38,21,02,174,*79"
	// };

    // ros::Rate loop_rate(1);
    // while(ros::ok){
	// std::vector<std::string> str;
	// str.assign(sentence, sentence + 8);
	// for (int i = 0; i < str.size();i++) {
	// 	gps.data_check(str[i]);
	// 	std::cout << std::endl;
	// }

    // loop_rate.sleep();
    // }



    // return 0;
}

