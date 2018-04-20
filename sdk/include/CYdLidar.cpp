#include "CYdLidar.h"
#include "common.h"
#include <math.h>

#ifndef __countof
#define __countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif


using namespace std;
using namespace ydlidar;
using namespace impl;

 int CYdLidar::NODE_COUNTS =720;
 double CYdLidar::EACH_ANGLE = 0.5;
/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
CYdLidar::CYdLidar() :
	m_com_port(""),
	m_com_port_baudrate(115200),
	m_intensity(false),
	max_range(16.0),
	min_range(0.08)
{
}

/*-------------------------------------------------------------
					~CFlashLidar
-------------------------------------------------------------*/
CYdLidar::~CYdLidar()
{
    disconnecting();
}

void CYdLidar::disconnecting()
{
    if(YDlidarDriver::singleton()){
        YDlidarDriver::singleton()->disconnect();
        YDlidarDriver::done();
    }
}

/*-------------------------------------------------------------
						doProcessSimple
-------------------------------------------------------------*/
bool  CYdLidar::doProcessSimple(LaserScan &outscan, bool &hardwareError){
	hardwareError			= false;

	// Bound?
	if (!checkCOMMs())
	{
		hardwareError = true;
		return false;
	}

	node_info nodes[NODE_COUNTS];
    	size_t   count = __countof(nodes);

	// Scan:
	const uint64_t tim_scan_start = getus();
	result_t op_result =  YDlidarDriver::singleton()->grabScanData(nodes, count);
	const uint64_t tim_scan_end = getus();


	const bool compensate = true;

	// Fill in scan data:
	if (op_result == RESULT_OK)
	{
		op_result = YDlidarDriver::singleton()->ascendScanData(nodes, count);

		//同步后的时间
		if(nodes[0].stamp > 0){
	   	 	start_scan_time.sec = nodes[0].stamp/1000000000;
           	 	start_scan_time.nsec = (nodes[0].stamp%1000000000);
	    	}
		const double scan_time = tim_scan_end - tim_scan_start;


		if (op_result == RESULT_OK)
		{
			if(compensate){
				const int angle_compensate_nodes_count = NODE_COUNTS;
    			node_info angle_compensate_nodes[NODE_COUNTS];
				memset(angle_compensate_nodes, 0, angle_compensate_nodes_count*sizeof(node_info));

				unsigned int i = 0;
                for( ; i < count; i++) {
                    if (nodes[i].distance_q2 != 0) {
                        float angle = (float)((nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
                        int inter =(int)( angle / EACH_ANGLE );
                        float angle_pre = angle - inter * EACH_ANGLE;
                        float angle_next = (inter+1) * EACH_ANGLE - angle;
                        if(angle_pre < angle_next){
							if(inter < NODE_COUNTS)
                                angle_compensate_nodes[inter]=nodes[i];
                        }else{
							if(inter < NODE_COUNTS -1)
                               	angle_compensate_nodes[inter+1]=nodes[i];
                        }
                    }
            	 }

			LaserScan scan_msg;
   			scan_msg.ranges.resize(NODE_COUNTS);
    			scan_msg.intensities.resize(NODE_COUNTS);
    			float range = 0.0;
    			float intensity = 0.0;
    			int index = 0;


    			for (size_t i = 0; i < NODE_COUNTS; i++) {
					range = (float)angle_compensate_nodes[i].distance_q2/4.0f/1000;
					intensity = (float)(angle_compensate_nodes[i].sync_quality >> LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);

        			if(i<NODE_COUNTS/2){
	    				index = NODE_COUNTS/2-1-i;	    
        			}else{
	    				index =NODE_COUNTS-1-(i-NODE_COUNTS/2);
        			}
				
					if(range > max_range|| range < min_range){
	    				range = 0.0;
        			}

	

					int pos = index ;
        			if(0<= pos && pos < NODE_COUNTS){
	    				scan_msg.ranges[pos] =  range;
	    				scan_msg.intensities[pos] = intensity;
					}
				}

				scan_msg.system_time_stamp = tim_scan_start;
    			scan_msg.self_time_stamp = tim_scan_start;
                scan_msg.config.min_angle = -PI;
                scan_msg.config.max_angle = PI;
   		 		scan_msg.config.ang_increment = (scan_msg.config.max_angle - scan_msg.config.min_angle) / (double)NODE_COUNTS;
    	 		scan_msg.config.time_increment = scan_time / (double)NODE_COUNTS;
    	 		scan_msg.config.scan_time = scan_time;
    			scan_msg.config.min_angle = min_range;
   				scan_msg.config.max_range = max_range;
				outscan = scan_msg;
				return true;

				


   			}
			
		}

	}
	else
	{
		if (op_result==RESULT_FAIL)
		{
			// Error? Retry connection
			//this->disconnect();
		}
	}

	return false;

}


/*-------------------------------------------------------------
						turnOn
-------------------------------------------------------------*/
bool  CYdLidar::turnOn()
{
	bool ret = checkCOMMs();
	if (ret && YDlidarDriver::singleton()){
		YDlidarDriver::singleton()->startMotor();
	}

	return ret;
}

/*-------------------------------------------------------------
						turnOff
-------------------------------------------------------------*/
bool  CYdLidar::turnOff()
{
	if (YDlidarDriver::singleton()) {
		YDlidarDriver::singleton()->stop();
		YDlidarDriver::singleton()->stopMotor();
	}
	return true;
}

/** Returns true if the device is connected & operative */
bool CYdLidar::getDeviceHealth() const
{
	if (!YDlidarDriver::singleton()) return false;

	result_t op_result;
        device_health healthinfo;

	op_result = YDlidarDriver::singleton()->getHealth(healthinfo);
    	if (op_result == RESULT_OK) { 
        	printf("Yd Lidar running correctly ! The health status: %s\n", (int)healthinfo.status==0?"good":"bad");
        
        	if (healthinfo.status == 2) {
            		fprintf(stderr, "Error, Yd Lidar internal error detected. Please reboot the device to retry.\n");
           	 	return false;
        	} else {
            		return true;
        	}

    	} else {
        	fprintf(stderr, "Error, cannot retrieve Yd Lidar health code: %x\n", op_result);
        	return false;
    	}
}

bool CYdLidar::getDeviceDeviceInfo() const{
	if (!YDlidarDriver::singleton()) return false;


	device_info devinfo;
	if (YDlidarDriver::singleton()->getDeviceInfo(devinfo) !=0){
		fprintf(stderr, "get DeviceInfo Error\n" );
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
        {
	    model="G4";
            sampling_rate _rate;
            YDlidarDriver::singleton()->getSamplingRate(_rate);
                
            switch(_rate.rate){
                case 0:
                    fprintf(stdout,"Current Sampling Rate : 4K\n");
                    break;
                case 1:
                    NODE_COUNTS = 1440;
                    EACH_ANGLE = 0.25;
                    fprintf(stdout,"Current Sampling Rate : 8K\n");
                    break;
                case 2:
                    NODE_COUNTS = 1440;
                    EACH_ANGLE = 0.25;
                    fprintf(stdout,"Current Sampling Rate : 9K\n");
                    break;
			}

	    }
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
			    m_com_port.c_str(),
			    maxv,
			    midv,
                minv,
			    (unsigned int)devinfo.hardware_version,
			    model.c_str());

		for (int i=0;i<16;i++)
			printf("%01X",devinfo.serialnum[i]&0xff);
		printf("\n");

		return true;
	

}



/*-------------------------------------------------------------
						checkCOMMs
-------------------------------------------------------------*/
bool  CYdLidar::checkCOMMs()
{
	if (YDlidarDriver::singleton())
		return true;

	// create the driver instance
	YDlidarDriver::initDriver();
	if(!YDlidarDriver::singleton()){
		 fprintf(stderr, "Create Driver fail\n");
        return false;

	}


	// Is it COMX, X>4? ->  "\\.\COMX"
	if (m_com_port.size()>=3)
	{
		if ( tolower( m_com_port[0]) =='c' && tolower( m_com_port[1]) =='o' && tolower( m_com_port[2]) =='m' )
		{
			// Need to add "\\.\"?
			if (m_com_port.size()>4 || m_com_port[3]>'4')
				m_com_port = std::string("\\\\.\\") + m_com_port;
		}
	}

	// make connection...
	result_t op_result = YDlidarDriver::singleton()->connect(m_com_port.c_str(), m_com_port_baudrate);
	if (op_result != RESULT_OK)
	{
		fprintf(stderr, "[CYdLidar] Error, cannot bind to the specified serial port %s\n",  m_com_port.c_str() );
		return false;
	}

	

	// check health:
	bool ret = getDeviceHealth()

	if(!getDeviceDeviceInfo()&&!ret)
		return false;
	
	YDlidarDriver::singleton()->setIntensities(m_intensity);
	 // start scan...
   	int s_result= YDlidarDriver::singleton()->startScan();
	if (s_result !=RESULT_OK)
	{
		fprintf(stderr, "[CYdLidar] Error starting scanning mode: %x\n", s_result);
		return false;
	}


	return true;
}

/*-------------------------------------------------------------
						initialize
-------------------------------------------------------------*/
void CYdLidar::initialize()
{
	bool ret = true;
	if (!checkCOMMs()){
		fprintf(stderr,"[CYdLidar::initialize] Error initializing YDLIDAR scanner.");
	
	}
	if (!turnOn()){
		fprintf(stderr,"[CYdLidar::initialize] Error initializing YDLIDAR scanner.");
		
	}
	
}


void CYdLidar::setSerialPort(const std::string &port_name)
{
	if (YDlidarDriver::singleton()){
		fprintf(stderr,"Can't change serial port while connected!");
		return;
	}
	m_com_port = port_name;

}

void CYdLidar::setSerialBaud(const uint32_t baudrate)
{
	if (YDlidarDriver::singleton()){
		fprintf(stderr,"Can't change serial baudrate while connected!");
		return ;
	}
	m_com_port_baudrate = baudrate;

}

void CYdLidar::setIntensities( bool  intensity)
{
	if (YDlidarDriver::singleton()){
		fprintf(stderr,"Can't change intensity while connected!");
		return ;
	}
	m_intensity = intensity;

}

void CYdLidar::setMaxRange(const float range)
{
	max_range = range;
}
			
			
void CYdLidar::setMinRange(const float range)
{
	min_range = range;
}

