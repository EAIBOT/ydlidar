#ifndef YDLIDAR_H
#define YDLIDAR_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <time.h>
#include <stdarg.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sstream>
#include <semaphore.h>
#include <malloc.h>
#include <stdbool.h>
#include <vector>
#include <asm/ioctls.h>
#include <asm/termbits.h>

#include <cstring>
#include <sstream>
#include <errno.h>
#include <exception>
#include <stdexcept>
#include <unistd.h>

#include "event.h"
#include "locker.h"

#define isDebug 0

#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))

#define THROW(exceptionClass, message) throw exceptionClass(__FILE__, \
__LINE__, (message) )


#define LIDAR_CMD_STOP                      0x65
#define LIDAR_CMD_SCAN                      0x60
#define LIDAR_CMD_FORCE_SCAN                0x61
#define LIDAR_CMD_FORCE_STOP                0x00
#define LIDAR_CMD_GET_EAI                   0x55
#define LIDAR_CMD_GET_DEVICE_INFO           0x90
#define LIDAR_CMD_GET_DEVICE_HEALTH         0x92
#define LIDAR_ANS_TYPE_DEVINFO              0x4
#define LIDAR_ANS_TYPE_DEVHEALTH            0x6
#define LIDAR_CMD_SYNC_BYTE                 0xA5
#define LIDAR_CMDFLAG_HAS_PAYLOAD           0x80
#define LIDAR_ANS_SYNC_BYTE1                0xA5
#define LIDAR_ANS_SYNC_BYTE2                0x5A
#define LIDAR_ANS_TYPE_MEASUREMENT          0x81
#define LIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2
#define LIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1<<0)
#define LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1

#ifndef min
#define min(a,b)  (((a) < (b)) ? (a) : (b))
#endif

#define PackageSampleMaxLngth 0x100
typedef enum
{
  CT_Normal = 0,
  CT_RingStart  = 1,
  CT_Tail,
}CT;
#define Node_Default_Quality (10<<2)
#define Node_Sync 1
#define Node_NotSync 2
#define PackagePaidBytes 10
#define PackageSampleBytes 2
#define PH 0x55AA

struct node_info{
    u_int8_t    sync_quality;
    u_int16_t   angle_q6_checkbit;
    u_int16_t   distance_q2;
} __attribute__((packed)) ;

struct node_package {
    u_int16_t  package_Head;
    u_int8_t   package_CT;
    u_int8_t   nowPackageNum;
    u_int16_t  packageFirstSampleAngle;
    u_int16_t  packageLastSampleAngle;
    u_int16_t  checkSum;
    u_int16_t  packageSampleDistance[PackageSampleMaxLngth];
} __attribute__((packed)) ;

struct device_info{
    u_int8_t   model;
    u_int16_t  firmware_version;
    u_int8_t   hardware_version;
    u_int8_t   serialnum[16];
} __attribute__((packed)) ;

struct device_health {
    u_int8_t   status;
    u_int16_t  error_code;
} __attribute__((packed))  ;

struct cmd_packet {
    u_int8_t syncByte;
    u_int8_t cmd_flag;
    u_int8_t size;
    u_int8_t data[0];
} __attribute__((packed)) ;

struct lidar_ans_header {
    u_int8_t  syncByte1;
    u_int8_t  syncByte2;
    u_int32_t size:30;
    u_int32_t subType:2;
    u_int8_t  type;
} __attribute__((packed));

struct scanDot {
    u_int8_t   quality;
    float angle;
    float dist;
};

using namespace std;

class Ydlidar
{
public:
    static Ydlidar* singleton()
    {
	return _impl;
    }
    static void initDriver()
    {
     	_impl = new Ydlidar;
    }
    static void done()
    {	
	if(_impl){
	    delete _impl;	
	    _impl = NULL;
	}
    }

public:
    int connect(const char * port_path, u_int32_t baudrate);
    void disconnect();
    int getHealth(device_health & health, u_int32_t timeout = DEFAULT_TIMEOUT);
    int getDeviceInfo(device_info & info, u_int32_t timeout = DEFAULT_TIMEOUT);
    int startScan(bool force = false, u_int32_t timeout = DEFAULT_TIMEOUT) ;
    int stop();
    int grabScanData(node_info * nodebuffer, size_t & count, u_int32_t timeout = DEFAULT_TIMEOUT) ;
    int ascendScanData(node_info * nodebuffer, size_t count);
    int createThread();
    u_int32_t getms();
    void simpleScanData(std::vector<scanDot> * scan_data , node_info *buffer, size_t count);

protected:
    Ydlidar();
    virtual ~Ydlidar();

    int waitPackage(node_info * node, u_int32_t timeout = DEFAULT_TIMEOUT);
    int waitScanData(node_info * nodebuffer, size_t & count, u_int32_t timeout = DEFAULT_TIMEOUT);
    int cacheScanData();
    int sendCommand(u_int8_t cmd, const void * payload = NULL, size_t payloadsize = 0);
    int waitResponseHeader(lidar_ans_header * header, u_int32_t timeout = DEFAULT_TIMEOUT);
    int waitForData(size_t data_count,u_int32_t timeout = -1, size_t * returned_size = NULL);
    int getData(unsigned char * data, size_t size);
    int sendData(const unsigned char * data, size_t size);
    void  disableDataGrabbing();
    static void* _thread_t(void* args);
    void setDTR();
    void clearDTR();
    int startMotor();
    int stopMotor();

public:
    bool     isConnected;
    bool     isScanning;
    bool     isThreadOn;
    enum {
        DEFAULT_TIMEOUT = 2000, //2000 ms
        MAX_SCAN_NODES = 2048,
    };
    node_info  scan_node_buf[2048];
    size_t     scan_node_count;
    Locker     _scanning_lock;
    Event      _dataEvt;
    Locker     _lock;

    static Ydlidar* _impl;

};


class SerialException : public std::exception
{
  // Disable copy constructors
  SerialException& operator=(const SerialException&);
  std::string e_what_;
public:
  SerialException (const char *description) {
      std::stringstream ss;
      ss << "SerialException " << description << " failed.";
      e_what_ = ss.str();
  }
  SerialException (const SerialException& other) : e_what_(other.e_what_) {}
  virtual ~SerialException() throw() {}
  virtual const char* what () const throw () {
    return e_what_.c_str();
  }
};

class IOException : public std::exception
{
  // Disable copy constructors
  IOException& operator=(const IOException&);
  std::string file_;
  int line_;
  std::string e_what_;
  int errno_;
public:
  explicit IOException (std::string file, int line, int errnum)
    : file_(file), line_(line), errno_(errnum) {
      std::stringstream ss;
#if defined(_WIN32) && !defined(__MINGW32__)
      char error_str [1024];
      strerror_s(error_str, 1024, errnum);
#else
      char * error_str = strerror(errnum);
#endif
      ss << "IO Exception (" << errno_ << "): " << error_str;
      ss << ", file " << file_ << ", line " << line_ << ".";
      e_what_ = ss.str();
  }
  explicit IOException (std::string file, int line, const char * description)
    : file_(file), line_(line), errno_(0) {
      std::stringstream ss;
      ss << "IO Exception: " << description;
      ss << ", file " << file_ << ", line " << line_ << ".";
      e_what_ = ss.str();
  }
  virtual ~IOException() throw() {}
  IOException (const IOException& other) : line_(other.line_), e_what_(other.e_what_), errno_(other.errno_) {}

  int getErrorNumber () const { return errno_; }

  virtual const char* what () const throw () {
    return e_what_.c_str();
  }
};

class PortNotOpenedException : public std::exception
{
  // Disable copy constructors
  const PortNotOpenedException& operator=(PortNotOpenedException);
  std::string e_what_;
public:
  PortNotOpenedException (const char * description)  {
      std::stringstream ss;
      ss << "PortNotOpenedException " << description << " failed.";
      e_what_ = ss.str();
  }
  PortNotOpenedException (const PortNotOpenedException& other) : e_what_(other.e_what_) {}
  virtual ~PortNotOpenedException() throw() {}
  virtual const char* what () const throw () {
    return e_what_.c_str();
  }

};

#endif // YDLIDAR_H
