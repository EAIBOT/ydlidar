
#include "CYdLidar.h"
#include <iostream>
#include <string>
#include <signal.h>
#include <memory>
//#include <unistd.h>
using namespace std;
using namespace ydlidar;
CYdLidar laser;
static bool running = false;

static void Stop(int signo)   
{  
    
    printf("Received exit signal\n");
    running = true;
     
}  

int main(int argc, char * argv[])
{

	printf(" YDLIDAR C++ TEST\n");
    std::string port;
    std::string baudrate;
    printf("Please enter the lidar port:");
    std::cin>>port;
    printf("Please enter the lidar baud rate:");
    std::cin>>baudrate;
    const int baud = atoi(baudrate.c_str());
    const int intensities = 0;

    signal(SIGINT, Stop);
    signal(SIGTERM, Stop);
    laser.setSerialPort(port);
    laser.setSerialBaudrate(baud);
    laser.setIntensities(intensities);
    laser.setMaxRange(16.0);
    laser.setMinRange(0.26);
    laser.setMaxAngle(180);
    laser.setMinAngle(-180);
    laser.setHeartBeat(false);
    laser.setReversion(false);
    laser.setFixedResolution(false);

    laser.initialize();


    while(!running){
		bool hardError;
		LaserScan scan;

		if(laser.doProcessSimple(scan, hardError )){
            for(int i =0; i < scan.ranges.size(); i++ ){
                float angle = scan.config.min_angle + i*scan.config.ang_increment;
                float dis = scan.ranges[i];

            }
			fprintf(stderr,"Scan received: %u ranges\n",(unsigned int)scan.ranges.size());

		}
    //usleep(50*1000);

		
	}
  laser.turnOff();
  laser.disconnecting();

  return 0;


}
