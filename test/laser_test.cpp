/*
 *  YDLIDAR SYSTEM
 *
 *  Copyright 2015 - 2018 EAI TEAM
 *  http://www.ydlidar.com
 * 
 */

#include "laser_test.h"
using namespace serial;
using namespace ydlidar;
Lasertest::DEVICE_STATE Lasertest::device_state_ = CLOSED;

Lasertest::Lasertest()
       :publish_freq_(40),
          scan_no_(0) {
    port_ = "/dev/ttyACM0";
    baudrate_ = 115200;
    angle_min_ = -180;
    angle_max_ = 180;
    intensities_ = false;
   
}

Lasertest::~Lasertest() {
}

void Lasertest::setPort(std::string port)
{
    port_ = port;
}

void Lasertest::setBaudrate(int baud)
{
    baudrate_ = baud;
}

void  Lasertest::setIntensities(bool intensities)
{
    intensities_ = intensities;
}


void Lasertest::run() {
    Open();
    Start();
}

void  Lasertest::closeApp(int signo){
    device_state_ = CLOSED;
    signal(signo, SIG_DFL);
    YDlidarDriver::singleton()->disconnect();
    printf("Now YDLIDAR is stopping .......\n");
    YDlidarDriver::done();
    exit(0);
}

/** Returns true if the device is connected & operative */
bool Lasertest::getDeviceInfo()
{
    if (!YDlidarDriver::singleton()){
        return false;
    }

    device_info devinfo;
    if (YDlidarDriver::singleton()->getDeviceInfo(devinfo) !=RESULT_OK){
        fprintf(stderr,"[YDLIDAR] get DeviceInfo Error\n" );
        return false;
    }
         
    std::string model;
    switch(devinfo.model){
        case 1:
            model="F4";
            break;
        case 2:
            model="T1";
            break;
        case 3:
            model="F2";
            break;
        case 4:
            model="S4";
            break;
        case 5:
            model="G4";
            break;
        case 6:
            model="X4";
            break;
        default:
            model = "Unknown";
    }

    unsigned int maxv = (unsigned int)(devinfo.firmware_version>>8);
    unsigned int midv = (unsigned int)(devinfo.firmware_version&0xff)/10;
    unsigned int minv = (unsigned int)(devinfo.firmware_version&0xff)%10;
    if(midv==0){
        midv = minv;
        minv = 0;
    }

    printf("[YDLIDAR] Connection established in [%s]:\n"
			   "Firmware version: %u.%u.%u\n"
			   "Hardware version: %u\n"
			   "Model: %s\n"
			   "Serial: ",
			    port_.c_str(),
			    maxv,
			    midv,
                	    minv,
			    (unsigned int)devinfo.hardware_version,
			    model.c_str());

    for (int i=0;i<16;i++){
        printf("%01X",devinfo.serialnum[i]&0xff);
    }
    printf("\n");
    return true;

}

/** Returns true if the device is connected & operative */
bool Lasertest::getDeviceHealth()
{
    if (!YDlidarDriver::singleton()) return false;

    result_t op_result;
    device_health healthinfo;

    op_result = YDlidarDriver::singleton()->getHealth(healthinfo);
    if (op_result == RESULT_OK) { 
        fprintf(stdout,"[YDLIDAR] running correctly ! The health status:%s\n" ,healthinfo.status==0? "good":"bad");
        if (healthinfo.status == 2) {
            fprintf(stderr, "Error, [YDLIDAR] internal error detected. Please reboot the device to retry.\n");
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve YDLIDAR health code: %x\n", op_result);
        return false;
    }
}

void Lasertest::Open() {
    try {
	if(!YDlidarDriver::singleton()){
	    YDlidarDriver::initDriver();
        } 
	result_t op_result = YDlidarDriver::singleton()->connect(port_.c_str(), (uint32_t)baudrate_);
	if (op_result != RESULT_OK) {
	  fprintf(stdout,"open Lidar is failed! Exit!! ......\n");
	  return;
	}

	signal(SIGINT, closeApp); 
    	signal(SIGTERM, closeApp);

	if(!getDeviceHealth()||!getDeviceInfo()){
	    YDlidarDriver::singleton()->disconnect();
	    YDlidarDriver::done();
	     return;
	}
        
	result_t ans=YDlidarDriver::singleton()->startScan();
	if(ans != RESULT_OK){
	    fprintf(stdout,"start Lidar is failed! Exit!! ......\n");
	    YDlidarDriver::singleton()->disconnect();
	    YDlidarDriver::done();
	    return;	
	}

        YDlidarDriver::singleton()->setIntensities(intensities_);
        fprintf(stdout,"Device opened successfully.\n");
        device_state_ = OPENED;
    } catch (std::exception &e) {
        Close();
        fprintf(stdout,"can't open laser\n ");
    }
}

void Lasertest::Start() {    
    if(device_state_ !=OPENED|| !YDlidarDriver::singleton()){
        return;
    }
    node_info all_nodes[NODE_COUNTS];
    memset(all_nodes, 0, NODE_COUNTS*sizeof(node_info));
    fprintf(stdout,"Now YDLIDAR is scanning.\n");
    device_state_ = RUNNING;

    double scan_duration;
    result_t op_result;

    while (device_state_ == RUNNING) {
        try {
            node_info nodes[360*2];
            size_t   count = _countof(nodes);
            uint64_t start_scan_time = getms();
            op_result = YDlidarDriver::singleton()->grabScanData(nodes, count);
            uint64_t end_scan_time = getms();
            scan_duration = (end_scan_time - start_scan_time);

            if (op_result == RESULT_OK) {
                op_result = YDlidarDriver::singleton()->ascendScanData(nodes, count);
		if (op_result == RESULT_OK) {
                    memset(all_nodes, 0, NODE_COUNTS*sizeof(node_info));
                    for(size_t i =0 ; i < count; i++) {
                        if (nodes[i].distance_q2 != 0) {
                            float angle = (float)((nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
                            int inter =(int)( angle / EACH_ANGLE );
                            float angle_pre = angle - inter * EACH_ANGLE;
                            float angle_next = (inter+1) * EACH_ANGLE - angle;
                            if(angle_pre < angle_next){
			        if(inter < NODE_COUNTS)
                                    all_nodes[inter]=nodes[i];
                           	}else{
				    if(inter < NODE_COUNTS-1)
                                	all_nodes[inter+1]=nodes[i];
                            	    }
                       		}
                    	    }
                        }
                    }
		    publicScanData(all_nodes,start_scan_time,scan_duration,NODE_COUNTS,angle_min_,angle_max_,ignore_array_);
                }	
	    }else if(op_result == RESULT_FAIL){
	    }
		
        } catch (std::exception& e) {
            Close();
            fprintf(stderr,"Exception thrown while starting YDLIDAR.\n ");
            return;
   	} catch (...) {
            fprintf(stderr,"Exception thrown while trying to get scan: \n");
       	    Close();
	    return;
    	}
    }   
}

void Lasertest::Stop() {
    device_state_ = OPENED;
}

void Lasertest::Close() {
    try {
        YDlidarDriver::singleton()->disconnect();
	YDlidarDriver::done();
        device_state_ = CLOSED;
	exit(0);
    } catch (std::exception &e) {
        return;
    }
}

std::vector<int> Lasertest::split(const std::string &s, char delim) {
    std::vector<int> elems;
    std::stringstream ss(s);
    std::string number;
    while(std::getline(ss, number, delim)) {
        elems.push_back(atoi(number.c_str()));
    }
    return elems;
}

void Lasertest::publicScanData(node_info *nodes, uint64_t start,double scan_time, size_t node_count, float angle_min, float angle_max,std::vector<int> ignore_array) {
    fprintf(stdout,"publicScanData: %lud   ,  %i\n",start, (int)node_count);

    float nodes_array[node_count];
    float quality_array[node_count];
    for (size_t i = 0; i < node_count; i++) {
        if(i<node_count/2){
            nodes_array[node_count/2-1-i] = (float)nodes[i].distance_q2/4.0f/1000;
            quality_array[node_count/2-1-i] = (float)(nodes[i].sync_quality >> 2);
        }else{
            nodes_array[node_count-1-(i-node_count/2)] = (float)nodes[i].distance_q2/4.0f/1000;
            quality_array[node_count-1-(i-node_count/2)] = (float)(nodes[i].sync_quality >> 2);
        }

        if(ignore_array.size() != 0){
	    float angle = (float)((nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
            if(angle>180){
                angle=360-angle;
            }else{
                angle=-angle;
            }
	    for(unsigned int j = 0; j < ignore_array.size();j = j+2){
                if((ignore_array[j] < angle) && (angle <= ignore_array[j+1])){
                    if(i<node_count/2){
                        nodes_array[node_count/2-1-i] = 0;
                    }else{
                        nodes_array[node_count-1-(i-node_count/2)] = 0;
                    }
		   break;
		}
	    }
	}

    }
    

    int counts = node_count*((angle_max-angle_min)/360.0f);
    int angle_start = 180+angle_min;
    int node_start = node_count*(angle_start/360.0f);

    LaserScan scan;
    scan.system_time_stamp = start;
    scan.self_time_stamp = start;

    float radian_min = DEG2RAD(angle_min);
    float radian_max = DEG2RAD(angle_max);

    scan.config.min_angle = radian_min;
    scan.config.max_angle = radian_max;
    scan.config.ang_increment = (radian_max - radian_min) / (double)counts;
    scan.config.time_increment = scan_time / (double)counts;
    scan.config.scan_time = scan_time;
    scan.config.min_angle = 0.1;
    scan.config.max_range = 15.;

    scan.ranges.resize(counts);
    scan.intensities.resize(counts);
    for (size_t i = 0; i < counts; i++) {
        scan.ranges[i] = nodes_array[i+node_start];
        scan.intensities[i] = quality_array[i+node_start];
    }

    for(int i = 0; i < counts; i++) {
        float degree = RAD2DEG(scan.config.min_angle + scan.config.ang_increment * i);
        if(scan.ranges[i] >scan.config.min_angle){
            printf(": [%f, %f]", degree, scan.ranges[i]);
        }
    }

    scan_no_++;

}
