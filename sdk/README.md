YDLIDAR SDK PACKAGE V1.3.3
=====================================================================

SDK [test](https://github.com/yangfuyuan/sdk) application for YDLIDAR

Visit EAI Website for more details about [YDLIDAR](http://www.ydlidar.com/) .

How to build YDLIDAR SDK samples
=====================================================================
    $ git clone https://github.com/yangfuyuan/sdk
    $ cd sdk
    $ git checkout master
    $ cd ..
    $ mkdir build
    $ cd build
    $ cmake ../sdk
    $ make			###linux
    $ vs open Project.sln	###windows
    
How to run YDLIDAR SDK samples
=====================================================================
    $ cd samples

linux:

    $ ./ydlidar_test
    $Please enter the lidar port:/dev/ttyUSB0
    $Please enter the lidar baud rate:230400

windows:

    $ ydlidar_test.exe
    $Please enter the lidar port:COM3
    $Please enter the lidar baud rate:230400

=====================================================================

You should see YDLIDAR's scan result in the console:

    Yd Lidar running correctly ! The health status: good
    [YDLIDAR] Connection established in [/dev/ttyUSB0]:
    Firmware version: 2.0.9
    Hardware version: 2
    Model: G4
    Serial: 2018022700000003
    [YDLIDAR INFO] Current Sampling Rate : 9K
    [YDLIDAR INFO] Current Scan Frequency : 7.400000Hz
    [YDLIDAR INFO] Now YDLIDAR is scanning ......
    Scan received: 43 ranges
    Scan received: 1361 ranges
    Scan received: 1412 ranges


Lidar point data structure
=====================================================================

data structure:

    struct node_info {

       uint8_t    sync_quality;//!intensity

       uint16_t   angle_q6_checkbit; //!angle

       uint16_t   distance_q2; //! distance

       uint64_t   stamp; //! time stamp

       uint8_t    scan_frequence;//! current_frequence = scan_frequence/10.0, If the current value equals zero, it is an invalid value
 
    } __attribute__((packed)) ;

example:

    if(data[i].scan_frequence != 0) {

        current_frequence = data[i].scan_frequence/10.0;
    }

    current_time_stamp = data[i].stamp;

    current_distance = data[i].distance_q2/4.f;

    current_angle = ((data[i].angle_q6_checkbit>>LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);

    current_intensity = (float)(data[i].sync_quality >> 2);

    ###note:current_frequence = data[0].scan_frequence/10.0.

    ###if the current_frequence value equals zero, it is an invalid value.

code:
        
        void ParseScan(node_info* data, const size_t& size) {

            double current_frequence, current_distance, current_angle, current_intensity;

            uint64_t current_time_stamp;

            for (size_t i = 0; i < size; i++ ) {

                if( data[i].scan_frequence != 0) {

                    current_frequence =  data[i].scan_frequence;//or current_frequence = data[0].scan_frequence

                }

                current_time_stamp = data[i].stamp;

                current_angle = ((data[i].angle_q6_checkbit>>LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);//LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT equals 8

                current_distance =  data[i].distance_q2/4.f;

                current_intensity = (float)(data[i].sync_quality >> 2);

            }

            if (current_frequence != 0 ) {

                printf("current lidar scan frequency: %f\n", current_frequence);

            } else {

                printf("Current lidar does not support return scan frequency\n");

            }
        }





Upgrade Log
=====================================================================
2018-05-14 version:1.3.3

   1.add the heart function constraint.

   2.add packet type with scan frequency support.

2018-04-16 version:1.3.2

   1.add multithreading support.

2018-04-16 version:1.3.1

   1.Compensate for each laser point timestamp.

