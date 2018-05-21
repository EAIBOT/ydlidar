
#pragma once
#include "utils.h"
#include "ydlidar_driver.h"
#include <math.h>

#if !defined(__cplusplus)
#ifndef __cplusplus
#error "The YDLIDAR SDK requires a C++ compiler to be built"
#endif
#endif
#define PropertyBuilderByName(type, name, access_permission)\
    access_permission:\
        type m_##name;\
    public:\
    inline void set##name(type v) {\
        m_##name = v;\
    }\
    inline type get##name() {\
        return m_##name;\
}\

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifndef M_PI
#define M_PI 3.1415926
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)

class YDLIDAR_API CYdLidar
{
    PropertyBuilderByName(float,MaxRange,private)///< 设置和获取激光最大测距范围
    PropertyBuilderByName(float,MinRange,private)///< 设置和获取激光最小测距范围
    PropertyBuilderByName(float,MaxAngle,private)///< 设置和获取激光最大角度, 最大值180度
    PropertyBuilderByName(float,MinAngle,private)///< 设置和获取激光最小角度, 最小值-180度
    PropertyBuilderByName(int,ScanFrequency,private)///< 设置和获取激光扫描频率(范围5HZ~12HZ)

    PropertyBuilderByName(bool,Intensities,private)///< 设置和获取激光带信号质量(只有S4B雷达支持)
    PropertyBuilderByName(bool,FixedResolution,private)///< 设置和获取激光是否是固定角度分辨率
    PropertyBuilderByName(bool,Exposure,private)///< 设置和获取激光时候开启低光功率曝光模式 只有S4雷达支持
    PropertyBuilderByName(bool,HeartBeat,private)///< 设置和获取激光是否开启掉电保护, 之后版本号大于等于2.0.9的(G4, F4PRO, G4C)支持
    PropertyBuilderByName(bool,Reversion, private)///< 设置和获取是否旋转激光180度

    PropertyBuilderByName(int,SerialBaudrate,private)///< 设置和获取激光通讯波特率
    PropertyBuilderByName(int,SampleRate,private)///< 设置和获取激光采样频率

    PropertyBuilderByName(std::string,SerialPort,private)///< 设置和获取激光端口号
    PropertyBuilderByName(std::vector<float>,IgnoreArray,private)///< 设置和获取激光剔除点


public:
	CYdLidar(); //!< Constructor
	virtual ~CYdLidar();  //!< Destructor: turns the laser off.

    bool initialize();  //!< Attempts to connect and turns the laser on. Raises an exception on error.

    // Return true if laser data acquistion succeeds, If it's not
    bool doProcessSimple(LaserScan &outscan, bool &hardwareError);

    //Turn on the motor enable
	bool  turnOn();  //!< See base class docs
    //Turn off the motor enable and close the scan
	bool  turnOff(); //!< See base class docs

    /** Returns true if the device is in good health, If it's not*/
	bool getDeviceHealth() const;

    /** Returns true if the device information is correct, If it's not*/
    bool getDeviceInfo(int &type);

    /** Retruns true if the heartbeat function is set to heart is successful, If it's not*/
    bool checkHeartBeat() const;

    /** Retruns true if the scan frequency is set to user's frequency is successful, If it's not*/
    bool checkScanFrequency();

    //Turn off lidar connection
    void disconnecting(); //!< Closes the comms with the laser. Shouldn't have to be directly needed by the user

protected:
    /** Returns true if communication has been established with the device. If it's not,
      *  try to create a comms channel.
      * \return false on error.
      */
    bool  checkCOMMs();

    /** Returns true if health status and device information has been obtained with the device. If it's not,
      * \return false on error.
      */
    bool  checkStatus();

    /** Returns true if the normal scan runs with the device. If it's not,
      * \return false on error.
      */
    bool checkHardware();



private:
    bool isScanning;
    int node_counts ;
    double each_angle;
    int show_error;
};	// End of class

