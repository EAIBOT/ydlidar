/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS Node Client 
 *
 *  Copyright 2015 - 2018 EAI TEAM
 *  http://www.ydlidar.com
 * 
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "ydlidar_driver.h"
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>

#define DELAY_SECONDS 4
#define DEG2RAD(x) ((x)*M_PI/180.)

using namespace ydlidar;

static int nodes_count = 720;
static float each_angle = 0.5;

void publish_scan(ros::Publisher *pub,  node_info *nodes,  size_t node_count, ros::Time start, double scan_time, float angle_min, float angle_max, std::string frame_id, std::vector<int> ignore_array, double min_range , double max_range)
{
    sensor_msgs::LaserScan scan_msg;

    int counts = node_count*((angle_max-angle_min)/360.0f);
    int angle_start = 180+angle_min;
    int node_start = node_count*(angle_start/360.0f);

    scan_msg.ranges.resize(counts);
    scan_msg.intensities.resize(counts);

    float range = 0.0;
    float intensity = 0.0;
    int index = 0;
    for (size_t i = 0; i < node_count; i++) {
	range = (float)nodes[i].distance_q2/4.0f/1000;
	intensity = (float)(nodes[i].sync_quality >> 2);

        if(i<node_count/2){
	    index = node_count/2-1-i;	    
        }else{
            index =node_count-1-(i-node_count/2);
        }

        if(ignore_array.size() != 0){
            float angle = (float)((nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
            if(angle>180){
                angle=360-angle;
            }else{
                angle=-angle;
            }
            for(uint16_t j = 0; j < ignore_array.size();j = j+2){
                if((ignore_array[j] < angle) && (angle <= ignore_array[j+1])){
                    range = 0.0;
		    break;
		}
	    }
	}

	if(range > max_range || range < min_range){
	    range = 0.0;
        }

        int pos = index - node_start ;
        if(0<= pos && pos < counts){
	    scan_msg.ranges[pos] =  range;
            scan_msg.intensities[pos] = intensity;
        }
    }

    scan_msg.header.stamp = start;
    scan_msg.header.frame_id = frame_id;
    float radian_min = DEG2RAD(angle_min);
    float radian_max = DEG2RAD(angle_max);
    scan_msg.angle_min = radian_min;
    scan_msg.angle_max = radian_max;
    scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)counts;
    scan_msg.scan_time = scan_time;
    scan_msg.time_increment = scan_time / (double)counts;
    scan_msg.range_min = min_range;
    scan_msg.range_max = max_range;

    pub->publish(scan_msg);
}


std::vector<int> split(const std::string &s, char delim) {
    std::vector<int> elems;
    std::stringstream ss(s);
    std::string number;
    while(std::getline(ss, number, delim)) {
        elems.push_back(atoi(number.c_str()));
    }
    return elems;
}

bool getDeviceInfo(std::string port)
{
    if (!YDlidarDriver::singleton()){
        return false;
    }

    device_info devinfo;
    if (YDlidarDriver::singleton()->getDeviceInfo(devinfo) !=RESULT_OK){
        ROS_ERROR("YDLIDAR get DeviceInfo Error\n" );
        return false;
    }
    int _samp_rate=4;
    std::string model;
    float freq = 7.0f;
    switch(devinfo.model){
            case 1:
                model="F4";
                _samp_rate=4;
                freq = 7.0;
                break;
            case 4:
                model="S4";
                _samp_rate=4;
                freq = 7.0;
                break;
            case 5:
                model="G4";
                freq = 7.0;
                    sampling_rate _rate;
                    YDlidarDriver::singleton()->getSamplingRate(_rate);
                    switch(_rate.rate){
                        case 0:
                            break;
                        case 1:
                            nodes_count = 1440;
                            each_angle = 0.25;
			    _samp_rate=8;
                            break;
                        case 2:
                            nodes_count = 1440;
                            each_angle = 0.25;
			    _samp_rate=9;
                            break;
                    }
                break;
            case 6:
                model="X4";
                _samp_rate=5;
                freq = 7.0;
                break;
            default:
                model = "Unknown";
    }

    uint16_t maxv = (uint16_t)(devinfo.firmware_version>>8);
    uint16_t midv = (uint16_t)(devinfo.firmware_version&0xff)/10;
    uint16_t minv = (uint16_t)(devinfo.firmware_version&0xff)%10;
    if(midv==0){
        midv = minv;
        minv = 0;
    }

    printf("[YDLIDAR INFO] Connection established in %s:\n"
            "Firmware version: %u.%u.%u\n"
            "Hardware version: %u\n"
            "Model: %s\n"
            "Serial: ",
            port.c_str(),
            maxv,
            midv,
            minv,
            (uint16_t)devinfo.hardware_version,
            model.c_str());

    for (int i=0;i<16;i++){
        printf("%01X",devinfo.serialnum[i]&0xff);
    }
    printf("\n");

    printf("[YDLIDAR INFO] Current Sampling Rate : %dK\n" , _samp_rate);
    printf("[YDLIDAR INFO] Current Scan Frequency : %fHz\n" , freq);

    return true;

}


bool getDeviceHealth()
{
    if (!YDlidarDriver::singleton()){
        return false;
    }

    result_t op_result;
    device_health healthinfo;

    op_result = YDlidarDriver::singleton()->getHealth(healthinfo);
    if (op_result == RESULT_OK) { 
        printf("[YDLIDAR INFO] YDLIDAR running correctly! The health status: %s\n", healthinfo.status==0?"well":"bad");
        
        if (healthinfo.status == 2) {
            ROS_ERROR("Error, YDLIDAR internal error detected. Please reboot the device to retry.");
            return false;
        } else {
            return true;
        }
    } else {
        ROS_ERROR("Error, cannot retrieve YDLIDAR health code: %x", op_result);
        return false;
    }

}

int main(int argc, char * argv[]) {
    ros::init(argc, argv, "ydlidar_node"); 

    std::string port;
    int baudrate=115200;
    std::string model;
    std::string frame_id;
    bool angle_fixed, intensities_,low_power,reversion;
    double angle_max,angle_min;
    result_t op_result;
    std::string list;
    std::vector<int> ignore_array;  
    double max_range , min_range;

    ros::NodeHandle nh;
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("port", port, "/dev/ydlidar"); 
    nh_private.param<std::string>("model", model, "G4"); 
    nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
    nh_private.param<bool>("angle_fixed", angle_fixed, "true");
    nh_private.param<bool>("intensities", intensities_, "false");
    nh_private.param<bool>("low_power", low_power, "false");
    nh_private.param<double>("angle_max", angle_max , 180);
    nh_private.param<double>("angle_min", angle_min , -180);
    nh_private.param<double>("range_max", max_range , 16.0);
    nh_private.param<double>("range_min", min_range , 0.08);
    nh_private.param<std::string>("ignore_array",list,"");
    ignore_array = split(list ,',');

    if(strcmp(model.c_str() , "X4")==0){
        baudrate=128000;
        reversion=false;
    }else if(strcmp(model.c_str() , "S4")==0){
        baudrate=115200;
        reversion=false;
    }else if(strcmp(model.c_str() , "F4")==0){
        baudrate=115200;
        reversion=false;
    }else if(strcmp(model.c_str() , "G4")==0){
        baudrate=230400;
        reversion=true;
    }

    if(ignore_array.size()%2){
        ROS_ERROR_STREAM("ignore array is odd need be even");
    }

    for(uint16_t i =0 ; i < ignore_array.size();i++){
        if(ignore_array[i] < -180 && ignore_array[i] > 180){
            ROS_ERROR_STREAM("ignore array should be between -180 and 180");
        }
    }

    YDlidarDriver::initDriver(); 
    if (!YDlidarDriver::singleton()) {
        ROS_ERROR("YDLIDAR Create Driver fail, exit\n");
        return -2;
    }

    printf("[YDLIDAR INFO] Current SDK Version: %s\n",YDlidarDriver::singleton()->getSDKVersion().c_str());

    op_result = YDlidarDriver::singleton()->connect(port.c_str(), (uint32_t)baudrate);
    if (op_result != RESULT_OK) {
        int seconds=0;
        while(seconds <= DELAY_SECONDS){
            sleep(2);
            seconds = seconds + 2;
            YDlidarDriver::singleton()->disconnect();
            op_result = YDlidarDriver::singleton()->connect(port.c_str(), (uint32_t)baudrate);
            printf("[YDLIDAR INFO] Try to connect the port %s again  after %d s .\n", port.c_str() , seconds);
            if(op_result==RESULT_OK){
                break;
            }
        }
        
        if(seconds > DELAY_SECONDS){
            ROS_ERROR("YDLIDAR Cannot bind to the specified serial port %s" , port.c_str());
	    YDlidarDriver::singleton()->disconnect();
	    YDlidarDriver::done();
            return -1;
        }
    }

    printf("[YDLIDAR INFO] Connected to YDLIDAR on port %s at %d \n" , port.c_str(), baudrate);
    if(!getDeviceHealth()||!getDeviceInfo(port)){
        YDlidarDriver::singleton()->disconnect();
        YDlidarDriver::done();
        return -1;
    }
    YDlidarDriver::singleton()->setIntensities(intensities_);
    if(intensities_){
	scan_power power;
	int cnt = 0;
        while((YDlidarDriver::singleton()->setLowPower(power) == RESULT_OK) && (cnt<3)){
            if(power.power != low_power){
                ROS_INFO("set POWER MODEL SUCCESS!!!");
                break;
            }
            cnt++;
        }
	if(cnt>=3){
            ROS_ERROR("set LOW POWER MODEL FALIED!!!");
	}
	
    }

    result_t ans=YDlidarDriver::singleton()->startScan();
    if(ans != RESULT_OK){
        ans = YDlidarDriver::singleton()->startScan();
        if(ans != RESULT_OK){
            ROS_ERROR("start YDLIDAR is failed! Exit!! ......");
            YDlidarDriver::singleton()->disconnect();
            YDlidarDriver::done();
            return 0;
        }
    }
	
    printf("[YDLIDAR INFO] Now YDLIDAR is scanning ......\n");
    ros::Time start_scan_time;
    ros::Time end_scan_time;
    double scan_duration;
    ros::Rate rate(30);

    node_info all_nodes[nodes_count];
    memset(all_nodes, 0, nodes_count*sizeof(node_info));

    while (ros::ok()) {
        try{
            node_info nodes[nodes_count];
            size_t count = _countof(nodes);

            start_scan_time = ros::Time::now();
            op_result = YDlidarDriver::singleton()->grabScanData(nodes, count);
            end_scan_time = ros::Time::now();

            if (op_result == RESULT_OK) {
		if(nodes[0].stamp > 0){
                    start_scan_time.sec = nodes[0].stamp/1000000000;
                    start_scan_time.nsec = (nodes[0].stamp%1000000000);
	    	}
	    	scan_duration = (end_scan_time - start_scan_time).toSec();

                op_result = YDlidarDriver::singleton()->ascendScanData(nodes, count);
            
                if (op_result == RESULT_OK) {
                    if (angle_fixed) {
                        memset(all_nodes, 0, nodes_count*sizeof(node_info));
                    
                        for(size_t i = 0; i < count; i++) {
                            if (nodes[i].distance_q2 != 0) {
                                float angle = (float)((nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
                                if(reversion){
                                    angle=angle+180;
                                    if(angle>=360){ angle=angle-360;}
                                    nodes[i].angle_q6_checkbit = ((uint16_t)(angle * 64.0f)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT;
                                }
                                int inter =(int)( angle / each_angle );
                                float angle_pre = angle - inter * each_angle;
                                float angle_next = (inter+1) * each_angle - angle;
                                if(angle_pre < angle_next){
                                    if(inter < nodes_count){
                                        all_nodes[inter]=nodes[i];
                                    }
                                }else{
                                    if(inter < nodes_count-1){
                                        all_nodes[inter+1]=nodes[i];
                                    }
                                }
                            }
                        }
                        publish_scan(&scan_pub, all_nodes, nodes_count, start_scan_time, scan_duration, angle_min, angle_max, frame_id, ignore_array, min_range , max_range);
                    } else {
                        int start_node = 0, end_node = 0;
                        int i = 0;
                        while (nodes[i++].distance_q2 == 0&&i<count);
                        start_node = i-1;
                        i = count -1;
                        while (nodes[i--].distance_q2 == 0&&i>=0);
                        end_node = i+1;

                        angle_min = (float)(nodes[start_node].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
                        angle_max = (float)(nodes[end_node].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;

                        publish_scan(&scan_pub, &nodes[start_node], end_node-start_node +1,  start_scan_time, scan_duration, angle_min, angle_max, frame_id, ignore_array, min_range , max_range);
                   }
                }
            }
            rate.sleep();
            ros::spinOnce();
	}catch(std::exception &e){//
            ROS_ERROR_STREAM("Unhandled Exception: " << e.what() );
            break;
	}catch(...){//anthor exception
            ROS_ERROR("Unhandled Exception:Unknown ");
            break;
	}
    }

    YDlidarDriver::singleton()->disconnect();
    printf("[YDLIDAR INFO] Now YDLIDAR is stopping .......\n");
    YDlidarDriver::done();
    return 0;
}
