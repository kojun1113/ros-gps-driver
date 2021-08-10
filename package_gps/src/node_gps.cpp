#include "node_gps.h"



node_gps::node_gps(ros::NodeHandle &nh){
    n = nh;
}

void node_gps::load_param(){


    if(!n.param<double>("rate",rate,20.0)){
        ROS_WARN_STREAM("load param error , default value is "<<20.0);
    }
    
    if(!n.param<std::string>("port",port,"/dev/ttyUSB0")){
        ROS_WARN_STREAM("load param error , default value is "<<"/dev/ttyUSB0");
    }
    
    if(!n.param<int>("baud",baud,4800)){
        ROS_WARN_STREAM("load param error , default value is "<<4800);
    }

	if(!n.param<bool>("use_GNSS_time",use_GNSS_time,false)){
        ROS_WARN_STREAM("load param error , default value is "<<false);
    }
}


void node_gps::register_sub(){
    ROS_INFO("register subscriber");
    sub_sentence = n.subscribe("/gps/serial_sentence", 1, &node_gps::callback_sentence, this);
}

void node_gps::register_pub(){
    ROS_INFO("register publisher");
	pub_fix = n.advertise<sensor_msgs::NavSatFix>("/gps/fix",1);
	pub_vel = n.advertise<geometry_msgs::TwistStamped>("/gps/vel",1);
	pub_heading = n.advertise<geometry_msgs::QuaternionStamped>("/gps/heading",1);
}

//   modify##################################
void node_gps::callback_sentence(const nmea_msgs::Sentence::ConstPtr &pmsg){
	sentence = (*pmsg);
	
	// publish in data_parse()
	data_parse();

}



void node_gps::data_parse(){
	std::string s = sentence.sentence;
	s=sentence.sentence;
	s=trim(s);
	if(!data_check(s)){
		ROS_WARN_STREAM("gps data check_sum wrong !");
		return;
	}
	// only receive GNGGA GNRMC format sentence
	if( (s.substr(3,3)!="GGA") && (s.substr(3,3)!="RMC") ) {
		ROS_INFO("ignore not GGA RMC type nmea_sentence");  // comment
		return;
	}

	// 0 => GGA / 1 => RMC
	int s_type = 0;
	if(s.substr(3,3)=="GGA"){
		s_type = 0;		
	}
	else if(s.substr(3,3)=="RMC"){
		s_type = 1;
	}
	else{
		ROS_WARN_STREAM("wrong in nmea_sentence type check !");
		return;
	}

	// data_parse
	std::vector<std::string> buffer_GGA;
	std::vector<std::string> buffer_RMC;
	std::string sp_1 = "*";
	std::string sp_2 = "$";
	std::string sp_3 = ",";
	std::string::size_type pos = 0; 
	std::string::size_type pos_1 = s.find(sp_1);
	std::string::size_type pos_2 = s.find(sp_2);

	switch (s_type)
	{
	case 0: {
		// modify sentence no $ *
		if (pos_1 != std::string::npos) {
			s = s.substr(pos_2 + 1, pos_1 - pos_2 - 1);
		}
		else
		{
			ROS_WARN_STREAM("no found * in sentence");
			return;
		}
		// split each value in vector buffer
		s = s + ",";
		// modify sentence no $ * and add a comma
		// split value into a vector buffer_GGA size() is 15 with to M (meter) and first data GPGGA
		while (  (pos = s.find(',')) != std::string::npos ) 
		{
		buffer_GGA.push_back(s.substr(0, pos));
		s = s.substr(pos + 1);
		}
		// set each value
		// timeStamp - GGA
		ros::Time utc_time;
		int s_hour, s_min, s_sec, day_offset;
		time_t t;
		std::time(&t);
		struct tm* pt;
		pt = std::gmtime(&t);

		if(use_GNSS_time==false){
			utc_time = ros::Time::now();
		}
		else{
			try {
				s_hour = safe_int(buffer_GGA[1].substr(0, 2));
				s_min = safe_int(buffer_GGA[1].substr(2, 2));
				s_sec = safe_int(buffer_GGA[1].substr(4, 2));
			}
			catch (...) {
				ROS_WARN_STREAM("wrong in stoi function");
				utc_time = ros::Time::now();
			}
			// to resolve the ambiguity of day / reference to nmea_navsat_driver/driver.py comment
			// it is kind of a augmented way / but not sure 100% right
			try{	
				day_offset = int((s_hour - pt->tm_hour) / 12.0);
				pt->tm_mday = pt->tm_mday + day_offset;
				pt->tm_hour = s_hour;
				pt->tm_min = s_min;
				pt->tm_sec = s_sec;
				t = std::mktime(pt);
				ros::Time utc_time_transfer(t);
				utc_time = utc_time_transfer;
			}	
			catch (...) {
				ROS_WARN_STREAM("wrong in ctime tm time_t setting");
				utc_time = ros::Time::now();
			}
		}
		// timeStamp - GGA
		// latitude - GGA
		double latitude,latitude_1,latitude_2;
		try {
			latitude_1 = safe_double(buffer_GGA[2].substr(0, 2));
			latitude_2 = safe_double(buffer_GGA[2].substr(2));
			latitude = latitude_1 + (latitude_2 / 60);
		}
		catch (...) {
			latitude = 0;
			ROS_WARN_STREAM("wrong in no latitude data in sentence");
		}
		// latitude - GGA
		// latitude_direction - GGA
		std::string latitude_direction;
		if(buffer_GGA[3].empty()){
			latitude_direction = "N";
			ROS_WARN_STREAM("wrong in no latitude direction data in sentence, default N");
		}
		else{
			latitude_direction = buffer_GGA[3];
		}
		// latitude_direction - GGA
		// longitude - GGA
		double longitude, longitude_1, longitude_2;
		try {
			longitude_1 = safe_double(buffer_GGA[4].substr(0, 3));
			longitude_2 = safe_double(buffer_GGA[4].substr(3));
			longitude = longitude_1 + (longitude_2 / 60);
		}
		catch (...) {
			longitude = 0;
			ROS_WARN_STREAM("wrong in no longitude data in sentence");
		}
		// longitude - GGA
		// longitude_direction - GGA
		std::string longitude_direction;
		if(buffer_GGA[5].empty()){
			longitude_direction = "E";
			ROS_WARN_STREAM("wrong in no longitude direction data in sentence, default E");
		}
		else{
			longitude_direction = buffer_GGA[5];
		}
		// longitude_direction - GGA
		// gps_status - GGA
		int fix_type;
		fix_type = safe_int(buffer_GGA[6]);
		// gps_status - GGA
		// num_satellites - GGA
		int num_satellites;
		num_satellites = safe_int(buffer_GGA[7]);
		// num_satellites - GGA
		// hdop - GGA
		double hdop;
		hdop = safe_double(buffer_GGA[8]);
		// hdop - GGA
		// altitude - GGA
		// altitude to eplisoid 
		double eplisoid_altitude;
		eplisoid_altitude = safe_double(buffer_GGA[9]);
		// altitude - GGA
		// delta_altitude - GGA
		// delta altitude eplisoid to datum
		double delta_altitude;
		delta_altitude = safe_double(buffer_GGA[11]);
		// delta_altitude - GGA
		// set each value
		
		// data acess
		double altitude;
		altitude = eplisoid_altitude + delta_altitude;
		if(latitude_direction == "S"){
			latitude = -latitude;
		}
		if(longitude_direction == "W"){
			longitude = -longitude;
		}


		// publish
		publish_GGA(utc_time,latitude,longitude,fix_type,hdop,altitude);

		break;
	}
	case 1: {
			// modify sentence no $ *
		if (pos_1 != std::string::npos) {
			s = s.substr(pos_2 + 1, pos_1 - pos_2 - 1);
		}
		else
		{
			ROS_WARN_STREAM("no found * in sentence");
			return;
		}
		// split each value in vector buffer
		s = s + ",";
		// modify sentence no $ * and add a comma
		// split value into a vector buffer_RMC size() is 15 with to M (meter) and first data GPRMC
		while (  (pos = s.find(',')) != std::string::npos ) 
		{
		buffer_RMC.push_back(s.substr(0, pos));
		s = s.substr(pos + 1);
		}

		// set each value
		// timeStamp - RMC
		ros::Time utc_time;
		int s_hour, s_min, s_sec, s_day, s_mon, s_year;
		time_t t;
		std::time(&t);
		struct tm* pt;
		pt = std::gmtime(&t);

		if(use_GNSS_time==false){
			utc_time = ros::Time::now();
		}
		else{
			try {
				s_hour = safe_int(buffer_RMC[1].substr(0, 2));
				s_min = safe_int(buffer_RMC[1].substr(2, 2));
				s_sec = safe_int(buffer_RMC[1].substr(4, 2));
				s_day = safe_int(buffer_RMC[9].substr(0, 2));
				s_mon = safe_int(buffer_RMC[9].substr(2, 2));
				s_year = safe_int(buffer_RMC[9].substr(4, 2));
			}
			catch (...) {
				ROS_WARN_STREAM("wrong in stoi function");
				utc_time = ros::Time::now();
			}
			try{
				pt->tm_year = s_year - 1900 + 2000;
				pt->tm_mon = s_mon - 1;
				pt->tm_mday = s_day;
				pt->tm_hour = s_hour;
				pt->tm_min = s_min;
				pt->tm_sec = s_sec;

				t = std::mktime(pt);
				ros::Time utc_time_transfer(t);
				utc_time = utc_time_transfer;
			}
			catch (...) {
				ROS_WARN_STREAM("wrong in ctime tm time_t setting");
				utc_time = ros::Time::now();
			}
		}
		// timeStamp - RMC
		// fix_status - RMC
			std::string fix_status_str;
			bool fix_status = true;
			fix_status_str = buffer_RMC[2];
			if(fix_status_str.empty()){
				fix_status = false;
				ROS_WARN_STREAM("wrong in no fix_status in RMC sentence");
			}
			else if(fix_status_str == "A"){
				fix_status = true;
			}
			else if(fix_status_str == "V"){
				fix_status = false;
			}
			else{
				fix_status = false;
				ROS_WARN_STREAM("wrong in fix_status in RMC sentence");
			}
		// fix_status - RMC
		// latitude - RMC
		double latitude,latitude_1,latitude_2;
		try {
			latitude_1 = safe_double(buffer_RMC[3].substr(0, 2));
			latitude_2 = safe_double(buffer_RMC[3].substr(2));
			latitude = latitude_1 + (latitude_2 / 60);
		}
		catch (...) {
			latitude = 0;
			ROS_WARN_STREAM("wrong in no latitude data in sentence");
		}
		// latitude - RMC
		// latitude_direction - RMC
		std::string latitude_direction;
		if(buffer_RMC[4].empty()){
			latitude_direction = "N";
			ROS_WARN_STREAM("wrong in no latitude direction data in sentence, default N");
		}
		else{
			latitude_direction = buffer_RMC[4];
		}
		// latitude_direction - RMC
		// longitude - RMC
		double longitude, longitude_1, longitude_2;
		try {
			longitude_1 = safe_double(buffer_RMC[5].substr(0, 3));
			longitude_2 = safe_double(buffer_RMC[5].substr(3));
			longitude = longitude_1 + (longitude_2 / 60);
		}
		catch (...) {
			longitude = 0;
			ROS_WARN_STREAM("wrong in no longitude data in sentence");
		}
		// longitude - RMC
		// longitude_direction - RMC
		std::string longitude_direction;
		if(buffer_RMC[6].empty()){
			longitude_direction = "E";
			ROS_WARN_STREAM("wrong in no longitude direction data in sentence, default E");
		}
		else{
			longitude_direction = buffer_RMC[6];
		}
		// longitude_direction - RMC
		// speed - RMC
		// speed knot to m/s
		double speed;
		speed = safe_double(buffer_RMC[7]) * 0.51444444;
		// speed - RMC
		// true_course - RMC
		// course deg to radian
		double true_course;
		true_course = deg2rad(safe_double(buffer_RMC[8]));
		// true_course - RMC
		// set each value
		
		// bool fix_status;
		// data acess
		if(latitude_direction == "S"){
			latitude = -latitude;
		}
		if(longitude_direction == "W"){
			longitude = -longitude;
		}


		// publish 
		publish_RMC(utc_time,fix_status,latitude,longitude,speed,true_course);		

		break;
	}
	default:
		ROS_WARN_STREAM("?? wrong in data_parse()");
		break;
	}




}
void node_gps::publish_GGA(const ros::Time &utc_time,const double &latitude,const double &longitude,const int &fix_type,const double &hdop,const double &altitude){

	// navsatfix navsatstatus twiststamped quarternionstamped
	sensor_msgs::NavSatFix fix;

	fix.header.frame_id = "/gps/fix";
	fix.header.stamp = utc_time;
	fix.latitude = latitude;
	fix.longitude = longitude;
	fix.altitude = altitude;

	double epe_quality0 = 1000000;
	double epe_quality1 = 4.0;
	double epe_quality2 = 0.1;
	double epe_quality4 = 0.02;
	double epe_quality5 = 4.0;
	double epe_quality9 = 3.0;
	double epe = 0;
	double lat_std_dev,lon_std_dev,alt_std_dev;
	

	//     ????????????? only GPS ?? BEIDOU => SERVICE_COMPASS ??
	fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

	switch (fix_type)
	{
	// INVALID
	case 0:{
		epe = epe_quality0;
		lat_std_dev = epe;
		lon_std_dev = epe;
		alt_std_dev = epe*2;
		fix.position_covariance[0] = std::pow((hdop * lon_std_dev),2); 
		fix.position_covariance[4] = std::pow((hdop * lat_std_dev),2);
		fix.position_covariance[8] = std::pow((2 * hdop * alt_std_dev),2);
		fix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
		fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
		break;
	}
	// SPS single point
	case 1:{
		epe = epe_quality1;
		lat_std_dev = epe;
		lon_std_dev = epe;
		alt_std_dev = epe*2;
		fix.position_covariance[0] = std::pow((hdop * lon_std_dev),2); 
		fix.position_covariance[4] = std::pow((hdop * lat_std_dev),2);
		fix.position_covariance[8] = std::pow((2 * hdop * alt_std_dev),2);
		fix.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
		fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
		break;
	}
	// DGPS pseudorange
	case 2:{
		epe = epe_quality2;
		lat_std_dev = epe;
		lon_std_dev = epe;
		alt_std_dev = epe*2;
		fix.position_covariance[0] = std::pow((hdop * lon_std_dev),2); 
		fix.position_covariance[4] = std::pow((hdop * lat_std_dev),2);
		fix.position_covariance[8] = std::pow((2 * hdop * alt_std_dev),2);
		fix.status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
		fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
		break;
	}
	// RTK fix
	case 4:{
		epe = epe_quality4;
		lat_std_dev = epe;
		lon_std_dev = epe;
		alt_std_dev = epe*2;
		fix.position_covariance[0] = std::pow((hdop * lon_std_dev),2); 
		fix.position_covariance[4] = std::pow((hdop * lat_std_dev),2);
		fix.position_covariance[8] = std::pow((2 * hdop * alt_std_dev),2);
		fix.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
		fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
		break;
	}
	// RTK float
	case 5:{
		epe = epe_quality5;
		lat_std_dev = epe;
		lon_std_dev = epe;
		alt_std_dev = epe*2;
		fix.position_covariance[0] = std::pow((hdop * lon_std_dev),2); 
		fix.position_covariance[4] = std::pow((hdop * lat_std_dev),2);
		fix.position_covariance[8] = std::pow((2 * hdop * alt_std_dev),2);
		fix.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
		fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
		break;
	}
	// WAAS
	case 9:{
		epe = epe_quality9;
		lat_std_dev = epe;
		lon_std_dev = epe;
		alt_std_dev = epe*2;
		fix.position_covariance[0] = std::pow((hdop * lon_std_dev),2); 
		fix.position_covariance[4] = std::pow((hdop * lat_std_dev),2);
		fix.position_covariance[8] = std::pow((2 * hdop * alt_std_dev),2);
		fix.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
		fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
		break;
	}
	// UNKNOWN
	default:{
		epe = epe_quality0;
		lat_std_dev = epe;
		lon_std_dev = epe;
		alt_std_dev = epe*2;
		fix.position_covariance[0] = std::pow((hdop * lon_std_dev),2); 
		fix.position_covariance[4] = std::pow((hdop * lat_std_dev),2);
		fix.position_covariance[8] = std::pow((2 * hdop * alt_std_dev),2);
		fix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
		fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
		break;
	}
	
	}


	pub_fix.publish(fix);
}

void node_gps::publish_RMC(const ros::Time &utc_time,const bool &fix_status,const double &latitude,const double &longitude,const double &speed,const double &true_course){

	// navsatfix navsatstatus twiststamped quarternionstamped
	sensor_msgs::NavSatFix fix;
	geometry_msgs::TwistStamped vel;
	geometry_msgs::QuaternionStamped heading;

	//fix
	fix.header.frame_id = "/gps/fix";
	fix.header.stamp = utc_time;
	fix.latitude = latitude;
	fix.longitude = longitude;
	
	// GPRMC => dont have altitude message / output 0, need to check if using altitude
	fix.altitude = 0;

	if(fix_status){
		fix.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
	}
	else{
		fix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
	}

	fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

	// unset covariancec in RMC
	fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

	pub_fix.publish(fix);
	
	// publish vel heading only while valid
	if(fix_status){
		//vel
		vel.header.frame_id = "/gps/vel";
		vel.header.stamp = utc_time;
		vel.twist.linear.x = speed * std::sin(true_course);
		vel.twist.linear.y = speed * std::cos(true_course);
		pub_vel.publish(vel);
		//heading
		heading.header.frame_id = "/gps/heading";
		heading.header.stamp = utc_time;
		tf2::Quaternion qtn;
		qtn.setRPY(0,0,true_course);
		heading.quaternion.w = qtn.getW();
		heading.quaternion.x = qtn.getX();
		heading.quaternion.y = qtn.getY();
		heading.quaternion.z = qtn.getZ();
		pub_heading.publish(heading);
	}


}







std::string& node_gps::trim(std::string &s){
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
// data_check
bool node_gps::data_check(std::string s) {
	s = trim(s);
	//std::cout << s << std::endl;
	//std::vector<std::string> buffer;
	std::stringstream sstream;
	std::string check_value_str;
	std::string check_sum_str;
	int check_sum = 0;
	std::string ss;
	std::string sp_1 = "*";
	std::string sp_2 = "$";
	std::string sp_3 = ",";
	std::string::size_type pos = 0;
	std::string::size_type pos_1 = s.find(sp_1);
	std::string::size_type pos_2 = s.find(sp_2);
	// modify sentence no $ *
	if (pos_1 != std::string::npos) {
		check_value_str = s.substr(pos_1 + 1, 2);
		ss = s.substr(pos_2 + 1, pos_1 - pos_2 - 1);
	}
	else
	{
		ROS_WARN_STREAM("no found * in sentence data_check()");	
		return false;
	}
	// checksum
	for (int i = 0; i < ss.size(); i++) {
		check_sum ^= int(ss[i]);
	}
	sstream << std::hex << std::setfill('0') << std::setw(2) << check_sum;
	check_sum_str = sstream.str();
	std::transform(check_sum_str.begin(), check_sum_str.end(), check_sum_str.begin(), ((int(*)(int)) (std::toupper)) );// std::toupper in <cctype> use int(*)(int) form
	if (check_sum_str == check_value_str) {
		ROS_INFO("check right"); // comment
		return true;
	}
	else
	{
		ROS_WARN_STREAM("check wrong");
		return false;
	}
}

int node_gps::safe_int(const std::string &s) {
	int result;
	try {
		result = std::stoi(s);
		return result;
	}
	catch (...) {
		result = 0;
		ROS_WARN_STREAM("wrong in stoi() no sentence data");
		return result;
	}

}

double node_gps::safe_double(const std::string& s) {
	double result;
	try {
		result = std::stod(s);
		return result;
	}
	catch (...) {
		result = 0;
		ROS_WARN_STREAM("wrong in stod() no sentence data");
		return result;
	}
}

double node_gps::deg2rad (double degrees) {
    return degrees * 3.1415926 / 180.0;
}
