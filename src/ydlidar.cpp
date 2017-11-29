#include "ydlidar.h"
#include "event.h"
#include "locker.h"

static int serial_fd;
static pthread_t threadId;
size_t required_tx_cnt;
size_t required_rx_cnt;
u_int32_t _baudrate;
bool isSupportMotorCtrl=true;

Ydlidar* Ydlidar::_impl = NULL;

Ydlidar::Ydlidar()
{
    isConnected = false;
    isScanning = false;
    isThreadOn = false;
}

Ydlidar::~Ydlidar()
{
    {
	ScopedLocker l(_scanning_lock);
    	isScanning = false;
    }

    if(isThreadOn||threadId){
        if(threadId)
            pthread_join(threadId , NULL);
    }
}

int Ydlidar::connect(const char * port_path, u_int32_t baudrate)
{
    _baudrate = baudrate;
    if (isConnected){
        close(serial_fd);
    }

    {
        ScopedLocker l(_lock);
        if (serial_fd != -1) {
            close(serial_fd);
        }

        serial_fd = -1;
        serial_fd = open(port_path, O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd == -1) {
            return -1;
        }

        struct termios2 options, oldopt;
	if((ioctl(serial_fd,TCGETS2,&oldopt))<0){
            printf("[EAI ERROR]: error to get the termios2 , %s\n",strerror(errno));
            return -1;
        }
        bzero(&options,sizeof(struct termios2));
        options.c_ispeed =baudrate;
        options.c_ospeed =baudrate;
        options.c_cflag = BOTHER;
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IGNBRK);
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag &= ~OPOST;

        if(ioctl(serial_fd, TCFLSH, TCIFLUSH)!=0){
            fprintf(stderr , "TCIFLUSH FAILED!!!!\n");
        }

        if (fcntl(serial_fd, F_SETFL, FNDELAY)) {
            if (serial_fd != -1) {
                close(serial_fd);
            }
            serial_fd = -1;
            return -1;
        }

#if isDebug
	printf("\nSet options\n");

        printf("options.c_iflag  = 0X%8X\n", options.c_iflag );
        printf("options.c_oflag  = 0X%8X\n", options.c_oflag );
        printf("options.c_cflag  = 0X%8X\n", options.c_cflag );
        printf("options.c_lflag  = 0X%8X\n", options.c_lflag );
        printf("options.c_line  = 0X%8X\n", options.c_line );
        for(int i = 0 ; i < sizeof(struct termios2)-25; i++)
        {
                printf("options.c_cc[%d]  = 0X%2X\n", i, options.c_cc[i] );
        }
#endif
	if((ioctl(serial_fd,TCSETS2,&options))<0){
            printf("[EAI ERROR]: error to s the termios2 , %s\n",strerror(errno));
            return -1;
        }

    }

    isConnected = true;

    //clear the DTR bit to let the motor spin
    clearDTR();
    
    return serial_fd;
}


void Ydlidar::setDTR()
{
    if (!isConnected){
        return ;
    }

    u_int32_t dtr_bit = TIOCM_DTR;
    if(-1==ioctl(serial_fd,TIOCMBIS,&dtr_bit)){
	stringstream ss;
        ss << "setDTR failed on a call to ioctl(TIOCMBIS): " << errno << " " << strerror(errno);
        throw(SerialException(ss.str().c_str()));
     }
	
}

void Ydlidar::clearDTR()
{
    if (!isConnected){
        return ;
    }

    u_int32_t dtr_bit = TIOCM_DTR;
    if(-1==ioctl(serial_fd,TIOCMBIC,&dtr_bit)){
	stringstream ss;
        ss << "clearDTR failed on a call to ioctl(TIOCMBIC): " << errno << " " << strerror(errno);
        throw(SerialException(ss.str().c_str()));
     }
}

int Ydlidar::startMotor()
{
    ScopedLocker l(_lock);
    if(isSupportMotorCtrl){
        setDTR();
        usleep(500);
        return 0;
    }else{
        clearDTR();
        usleep(500);
        return 0;
    }
}

int Ydlidar::stopMotor()
{
    ScopedLocker l(_lock);
    if(isSupportMotorCtrl){
        clearDTR();
        usleep(500);
        return 0;
    }else{
        setDTR();
        usleep(500);
        return 0;
    }
}

void Ydlidar::disconnect()
{
    if (!isConnected){
        return ;
    }
    stop();
    if (serial_fd != -1) {
	int ret;
        ret = close(serial_fd);
	if(ret == 0){
	    serial_fd = -1;
	}else{
	    THROW (IOException, errno);
	}
    }
    serial_fd = -1;
    isConnected = false;
}


void Ydlidar::disableDataGrabbing()
{
    {
	ScopedLocker l(_scanning_lock);
    	isScanning = false;
    }

    if(isThreadOn){
	if(threadId){
            pthread_join(threadId , NULL);
	}
    }
}

int Ydlidar::sendCommand(u_int8_t cmd, const void * payload, size_t payloadsize)
{
    u_int8_t pkt_header[10];
    cmd_packet * header = reinterpret_cast<cmd_packet * >(pkt_header);
    u_int8_t checksum = 0;

    if (!isConnected) {
        return -2;
    }
    if (payloadsize && payload) {
        cmd |= LIDAR_CMDFLAG_HAS_PAYLOAD;
    }

    header->syncByte = LIDAR_CMD_SYNC_BYTE;
    header->cmd_flag = cmd;
    sendData(pkt_header, 2) ;

    if (cmd & LIDAR_CMDFLAG_HAS_PAYLOAD) {
        checksum ^= LIDAR_CMD_SYNC_BYTE;
        checksum ^= cmd;
        checksum ^= (payloadsize & 0xFF);

        for (size_t pos = 0; pos < payloadsize; ++pos) {
            checksum ^= ((u_int8_t *)payload)[pos];
        }

        u_int8_t sizebyte = payloadsize;
        sendData(&sizebyte, 1);

        sendData((const u_int8_t *)payload, sizebyte);

        sendData(&checksum, 1);
     }
    return 0;
}

int Ydlidar::sendData(const unsigned char * data, size_t size)
{
    if (!isConnected) {
        return 0;
    }

    if (data == NULL || size ==0) {
        return 0;
    }

    size_t tx_len = 0;
    required_tx_cnt = 0;
    do {
        int ans = write(serial_fd, data + tx_len, size-tx_len);
        if (ans == -1) {
            return tx_len;
        }
        tx_len += ans;
        required_tx_cnt = tx_len;
    }while (tx_len<size);

    return tx_len;
}

int Ydlidar::getData(unsigned char * data, size_t size)
{
    if (!isConnected) {
        return -1;
    }
    int ans = read(serial_fd, data, size);
    return ans;
}

u_int32_t Ydlidar::getms()
{
    struct timespec currentTime;
    memset(&currentTime, 0, sizeof(currentTime));
    currentTime.tv_sec = currentTime.tv_nsec = 0;
    clock_gettime(CLOCK_MONOTONIC, &currentTime);
    u_int32_t tms = currentTime.tv_sec*1000L + currentTime.tv_nsec/1000000L;
    memset(&currentTime, 0, sizeof(currentTime));
    return tms;
}

int Ydlidar::waitResponseHeader(lidar_ans_header * header, u_int32_t timeout)
{
    int  recvPos = 0;
    u_int32_t startTs = getms();
    u_int8_t  recvBuffer[sizeof(lidar_ans_header)];
    u_int8_t  *headerBuffer = reinterpret_cast<u_int8_t *>(header);
    u_int32_t waitTime;

    while ((waitTime=getms() - startTs) <= timeout) {
        size_t remainSize = sizeof(lidar_ans_header) - recvPos;
        size_t recvSize;

        int ans = waitForData(remainSize, timeout - waitTime, &recvSize);
        if (ans == -2){
            return -2;
        }else if (ans == -1){
            return -1;
        }

        if(recvSize > remainSize) recvSize = remainSize;

        ans = getData(recvBuffer, recvSize);
        if (ans == -1){
            return -2;
        }

        for (size_t pos = 0; pos < recvSize; ++pos) {
            u_int8_t currentByte = recvBuffer[pos];
            switch (recvPos) {
            case 0:
                if (currentByte != LIDAR_ANS_SYNC_BYTE1) {
                   continue;
                }
                break;
            case 1:
                if (currentByte != LIDAR_ANS_SYNC_BYTE2) {
                    recvPos = 0;
                    continue;
                }
                break;
            }
            headerBuffer[recvPos++] = currentByte;

            if (recvPos == sizeof(lidar_ans_header)) {
                return 0;
            }
        }
    }
    return -1;
}

int Ydlidar::waitForData(size_t data_count, u_int32_t timeout, size_t * returned_size)
{
    size_t length = 0;
    if (returned_size==NULL) {
        returned_size=(size_t *)&length;
    }

    int max_fd;
    fd_set input_set;
    struct timeval timeout_val;

    FD_ZERO(&input_set);
    FD_SET(serial_fd, &input_set);
    max_fd = serial_fd + 1;

    timeout_val.tv_sec = timeout / 1000;
    timeout_val.tv_usec = (timeout % 1000) * 1000;

    if ( isConnected ){
        if ( ioctl(serial_fd, FIONREAD, returned_size) == -1) {
	    THROW (IOException, errno);
            return -2;
        }
        if (*returned_size >= data_count){
            return 0;
        }
    }

    while (isConnected) {
        int n = select(max_fd, &input_set, NULL, NULL, &timeout_val);
        if (n < 0){
	    // Select was interrupted
    	   if (errno == EINTR) {
     	       return -1;
    	   }
    	   // Otherwise there was some error
    	   THROW (IOException, errno);
           return -2;
        }else if (n == 0)  {
           return -1;//timeout
        } else {
            assert (FD_ISSET(serial_fd, &input_set));
            if ( ioctl(serial_fd, FIONREAD, returned_size) == -1) {
	        THROW (IOException, errno);
                return -2;
            }

            if (*returned_size >= data_count){
                return 0;
            } else {
                int remain_timeout = timeout_val.tv_sec*1000000 + timeout_val.tv_usec;
                int expect_remain_time = (data_count - *returned_size)*1000000*8/_baudrate;
                if (remain_timeout > expect_remain_time){
                    usleep(expect_remain_time);
                }
            }
        }
    }
    THROW (IOException, errno);
    return -2;
}

int Ydlidar::getHealth(device_health & health, u_int32_t timeout)
{
    int ans;
    if (!isConnected) {
        return -2;
    }

    disableDataGrabbing();
    {
        ScopedLocker l(_lock);
        if ((ans = sendCommand(LIDAR_CMD_GET_DEVICE_HEALTH)) != 0) {
            return ans;
        }
        lidar_ans_header response_header;
        if ((ans = waitResponseHeader(&response_header, timeout)) != 0) {
            return ans;
        }

        if (response_header.type != LIDAR_ANS_TYPE_DEVHEALTH) {
            return -3;
        }

        if (response_header.size < sizeof(device_health)) {
            return -3;
        }

        if (waitForData(response_header.size, timeout) != 0) {
            return -1;
        }

        getData(reinterpret_cast<u_int8_t *>(&health), sizeof(health));
    }
    return 0;
}


int Ydlidar::getDeviceInfo(device_info & info, u_int32_t timeout)
{
    int  ans;
    if (!isConnected) {
        return -2;
    }

    disableDataGrabbing();
    {
        ScopedLocker l(_lock);
        if ((ans = sendCommand(LIDAR_CMD_GET_DEVICE_INFO)) != 0) {
            return ans;
        }

        lidar_ans_header response_header;
        if ((ans = waitResponseHeader(&response_header, timeout)) != 0) {
            return ans;
        }

        if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
            return -3;
        }

        if (response_header.size < sizeof(lidar_ans_header)) {
            return -3;
        }

        if (waitForData(response_header.size, timeout) != 0) {
            return -1;
        }
        getData(reinterpret_cast<u_int8_t *>(&info), sizeof(info));
    }
    return 0;
}

int Ydlidar::startScan(bool force, u_int32_t timeout )
{
    int ans;
    if (!isConnected) {
        return -2;
    }
    if (isScanning) {
        return 0;
    }

    stop();    
    startMotor();

    {
        ScopedLocker l(_lock);
        if ((ans = sendCommand(force?LIDAR_CMD_FORCE_SCAN:LIDAR_CMD_SCAN)) != 0) {
            return ans;
        }

        lidar_ans_header response_header;
        if ((ans = waitResponseHeader(&response_header, timeout)) != 0) {
            return ans;
        }

        if (response_header.type != LIDAR_ANS_TYPE_MEASUREMENT) {
            return -3;
        }

        if (response_header.size < sizeof(node_info)) {
            return -3;
        }
        isScanning = true;
        ans = this->createThread();
        return ans;
    }
    return 0;
}

void * Ydlidar::_thread_t(void *args)
{
    Ydlidar* pThis = static_cast<Ydlidar *>(args);
    pThis->cacheScanData();
    return NULL;
}

int Ydlidar::createThread()
{
    if(pthread_create(&threadId,NULL,_thread_t,(void *)this) != 0) {
        isThreadOn=false;
        return -2;
    }
   
    isThreadOn=true;
    return 0;
}

int Ydlidar::stop()
{
    int ans;
    node_info  local_buf[128];
    size_t     count = 128;

    disableDataGrabbing();
    {
	ScopedLocker l(_lock);
	ans = sendCommand(LIDAR_CMD_FORCE_STOP);
    	if(ans != 0)
            return ans;
    }

    stopMotor();
    return 0;
}

int Ydlidar::cacheScanData()
{
    node_info    local_buf[128];
    size_t       count = 128;
    node_info    local_scan[MAX_SCAN_NODES];
    size_t       scan_count = 0;
    int          ans;
    memset(local_scan, 0, sizeof(local_scan));

    waitScanData(local_buf, count);

    while(isScanning) {

        if ((ans=waitScanData(local_buf, count)) != 0) {
            if (ans != -1) {
		fprintf(stderr, "exit scanning thread!!");
                {
		    ScopedLocker l(_scanning_lock);
    		    isScanning = false;
    		}
                return -2;
            }
        }

        for (size_t pos = 0; pos < count; ++pos) {
            if (local_buf[pos].sync_quality & LIDAR_RESP_MEASUREMENT_SYNCBIT) {
                if ((local_scan[0].sync_quality & LIDAR_RESP_MEASUREMENT_SYNCBIT)) {
		    _lock.lock();//timeout lock, wait resource copy 
                    memcpy(scan_node_buf, local_scan, scan_count*sizeof(node_info));
                    scan_node_count = scan_count;
		    _dataEvt.set();
		    _lock.unlock();
                }
                scan_count = 0;
            }
            local_scan[scan_count++] = local_buf[pos];
            if (scan_count == _countof(local_scan)) scan_count-=1;
        }
    }
    {
	ScopedLocker l(_scanning_lock);
    	isScanning = false;
    }
    return 0;
}

int Ydlidar::waitPackage(node_info * node, u_int32_t timeout)
{
    int recvPos = 0;
    int recvPosEnd = 0;
    u_int32_t startTs = getms();
    u_int8_t  recvBuffer[sizeof(node_package)];
    u_int8_t *nodeBuffer = (u_int8_t*)node;
    u_int32_t waitTime;

    static node_package package;
    static u_int16_t package_Sample_Index = 0;
    static u_int16_t IntervalSampleAngle = 0;
    static u_int16_t IntervalSampleAngle_LastPackage = 0;
    static u_int16_t FirstSampleAngle = 0;
    static u_int16_t LastSampleAngle = 0;
    static u_int16_t CheckSun = 0;
    static u_int32_t RingNum = 0;

    static u_int16_t CheckSunCal = 0;
    static u_int16_t SampleNumlAndCTCal = 0;
    static u_int16_t LastSampleAngleCal = 0;
    static bool CheckSunResult = true;
    static u_int16_t Valu8Tou16 = 0;

    u_int8_t *packageBuffer = (u_int8_t*)&package.package_Head;
    u_int8_t  package_Sample_Num = 0;

    int32_t AngleCorrectForDistance;

    int  package_recvPos = 0;

    if(package_Sample_Index == 0) {
       recvPos = 0;
       while ((waitTime=getms() - startTs) <= timeout) {
            size_t remainSize = PackagePaidBytes - recvPos;
            size_t recvSize;
            int ans = waitForData(remainSize, timeout-waitTime, &recvSize);
            if (ans == -2 || ans == -1){
                return ans;
            }

            if (recvSize > remainSize) recvSize = remainSize;

            getData(recvBuffer, recvSize);

            for (size_t pos = 0; pos < recvSize; ++pos) {
                u_int8_t currentByte = recvBuffer[pos];
                switch (recvPos) {
                case 0:
		    if ( currentByte == (PH&0xFF) ) {

		    } else {
			continue;
		    }
                    break;
                case 1:
		    CheckSunCal = PH;
		    if ( currentByte == (PH>>8) ) {

		    } else {
		        recvPos = 0;
			continue;
		    }
                    break;
                case 2:
		    SampleNumlAndCTCal = currentByte;
		    if ((currentByte == CT_Normal) || (currentByte == CT_RingStart)){

		    } else {
			recvPos = 0;
			continue;
		    }
                    break;
                case 3:
                    SampleNumlAndCTCal += (currentByte*0x100);
                    package_Sample_Num = currentByte;
                    break;
		case 4:
                    if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
                        FirstSampleAngle = currentByte;
                    } else {
                        recvPos = 0;
                        continue;
                    }
                    break;
                case 5:
                    FirstSampleAngle += currentByte*0x100;
	            CheckSunCal ^= FirstSampleAngle;
                    FirstSampleAngle = FirstSampleAngle>>1;
                    break;
                case 6:
                    if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
                        LastSampleAngle = currentByte;
                    } else {
                        recvPos = 0;
                        continue;
                    }
                    break;
                case 7:
                    LastSampleAngle = currentByte*0x100 + LastSampleAngle;
                    LastSampleAngleCal = LastSampleAngle;
                    LastSampleAngle = LastSampleAngle>>1;
                    if(package_Sample_Num == 1){
                        IntervalSampleAngle = 0;
                    }else{
                        if(LastSampleAngle < FirstSampleAngle){
                            if((FirstSampleAngle > 270*64) && (LastSampleAngle < 90*64)){
                                IntervalSampleAngle = (360*64 + LastSampleAngle - FirstSampleAngle)/(package_Sample_Num-1);
                                IntervalSampleAngle_LastPackage = IntervalSampleAngle;
                            } else{
                                IntervalSampleAngle = IntervalSampleAngle_LastPackage;
                            }
                        } else{
                            IntervalSampleAngle = (LastSampleAngle -FirstSampleAngle)/(package_Sample_Num-1);
                            IntervalSampleAngle_LastPackage = IntervalSampleAngle;
                        }
                    }
                    break;
                case 8:
		    CheckSun = currentByte;	
		    break;
	        case 9:
	            CheckSun += (currentByte*0x100);
	            break;
                }
                packageBuffer[recvPos++] = currentByte;
            }

            if (recvPos  == PackagePaidBytes ){
                package_recvPos = recvPos;
                break;
            }
        }

        if(PackagePaidBytes == recvPos){
            startTs = getms();
            recvPos = 0;
            while ((waitTime=getms() - startTs) <= timeout) {
                size_t remainSize = package_Sample_Num*PackageSampleBytes - recvPos;
                size_t recvSize;
                int ans =waitForData(remainSize, timeout-waitTime, &recvSize);
                if (ans == -2){
                    return -2;
                }else if (ans == -1){
                    return -1;
                }

                if (recvSize > remainSize) recvSize = remainSize;

                getData(recvBuffer, recvSize);

                for(size_t pos = 0; pos < recvSize; ++pos) {
                     if(recvPos%2 == 1){
	                 Valu8Tou16 += recvBuffer[pos]*0x100;
	                 CheckSunCal ^= Valu8Tou16;
	             }else{
	                 Valu8Tou16 = recvBuffer[pos];	
	             }
                     packageBuffer[package_recvPos+recvPos] = recvBuffer[pos];
                     recvPos++;
                }        
                if(package_Sample_Num*PackageSampleBytes == recvPos){
                    package_recvPos += recvPos;
                    break;
                }
            }
            if(package_Sample_Num*PackageSampleBytes != recvPos){
                return -1;
            }
        } else {
            return -1;
        }
        CheckSunCal ^= SampleNumlAndCTCal;
        CheckSunCal ^= LastSampleAngleCal;

        if(CheckSunCal != CheckSun){	
            CheckSunResult = false;
	}else{
	    CheckSunResult = true;
        }

    }

    if(package.package_CT == CT_Normal){
        (*node).sync_quality = Node_Default_Quality + Node_NotSync;
    } else{
        (*node).sync_quality = Node_Default_Quality + Node_Sync;
    }

    if(CheckSunResult == true){
        (*node).distance_q2 = package.packageSampleDistance[package_Sample_Index];
        if((*node).distance_q2/4 != 0){
            AngleCorrectForDistance = (int32_t)(((atan(((21.8*(155.3 - ((*node).distance_q2/4)) )/155.3)/((*node).distance_q2/4)))*180.0/3.1415) * 64.0);
        }else{
            AngleCorrectForDistance = 0;		
        }
        if((FirstSampleAngle + IntervalSampleAngle*package_Sample_Index + AngleCorrectForDistance) < 0){
            (*node).angle_q6_checkbit = ((FirstSampleAngle + IntervalSampleAngle*package_Sample_Index + AngleCorrectForDistance + 360*64)<<1) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
        }else{
            if((FirstSampleAngle + IntervalSampleAngle*package_Sample_Index + AngleCorrectForDistance) > 360*64){
                (*node).angle_q6_checkbit = ((FirstSampleAngle + IntervalSampleAngle*package_Sample_Index + AngleCorrectForDistance - 360*64)<<1) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
            }else{
                (*node).angle_q6_checkbit = ((FirstSampleAngle + IntervalSampleAngle*package_Sample_Index + AngleCorrectForDistance)<<1) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
            } 
        }
    }else{
        (*node).sync_quality = Node_Default_Quality + Node_NotSync;
        (*node).angle_q6_checkbit = LIDAR_RESP_MEASUREMENT_CHECKBIT;
        (*node).distance_q2 = 0;
    }

    package_Sample_Index++;
    if(package_Sample_Index >= package.nowPackageNum){
        package_Sample_Index = 0;
    }
    return 0;
}

int Ydlidar::waitScanData(node_info * nodebuffer, size_t & count, u_int32_t timeout)
{
    if (!isConnected) {
        count = 0;
        return -2;
    }

    size_t     recvNodeCount =  0;
    u_int32_t  startTs = getms();
    u_int32_t  waitTime;
    int ans;

    while ((waitTime = getms() - startTs) <= timeout && recvNodeCount < count) {
        node_info node;
        if ((ans = this->waitPackage(&node, timeout - waitTime)) != 0) {
            return ans;
        }
        nodebuffer[recvNodeCount++] = node;

        if (recvNodeCount == count) {
            return 0;
        }
    }
    count = recvNodeCount;
    return -1;
}


int Ydlidar::grabScanData(node_info * nodebuffer, size_t & count, u_int32_t timeout)
{
    switch (_dataEvt.wait(timeout))
    {
	case Event::EVENT_TIMEOUT:
            count = 0;
       	    return -2;
	case Event::EVENT_OK:
	    {
        	if(scan_node_count == 0) {
           	    return -2;
                }
        	size_t size_to_copy = min(count, scan_node_count);
		ScopedLocker l(_lock);
        	memcpy(nodebuffer, scan_node_buf, size_to_copy*sizeof(node_info));
        	count = size_to_copy;
        	scan_node_count = 0;
	    }
	    return 0;
	default:
            count = 0;
            return -1;
    }

}

void Ydlidar::simpleScanData(std::vector<scanDot> *scan_data , node_info *buffer, size_t count)
{
    scan_data->clear();
    for (int pos = 0; pos < (int)count; ++pos) {
        scanDot dot;
        if (!buffer[pos].distance_q2) continue;
        dot.quality = (buffer[pos].sync_quality>>LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
        dot.angle = (buffer[pos].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
        dot.dist = buffer[pos].distance_q2/4.0f;
        scan_data->push_back(dot);
    }
}

int Ydlidar::ascendScanData(node_info * nodebuffer, size_t count)
{
    float     inc_origin_angle = 360.0/count;
    node_info *tmpbuffer = new node_info[count];
    int i = 0;

    for (i = 0; i < (int)count; i++) {
        if(nodebuffer[i].distance_q2 == 0) {
            continue;
        } else {
            while(i != 0) {
                i--;
                float expect_angle = (nodebuffer[i+1].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f - inc_origin_angle;
                if (expect_angle < 0.0f) expect_angle = 0.0f;
                u_int16_t checkbit = nodebuffer[i].angle_q6_checkbit & LIDAR_RESP_MEASUREMENT_CHECKBIT;
                nodebuffer[i].angle_q6_checkbit = (((u_int16_t)(expect_angle * 64.0f)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + checkbit;
            }
            break;
        }
    }

    if (i == (int)count) return -3;

    for (i = (int)count - 1; i >= 0; i--) {
        if(nodebuffer[i].distance_q2 == 0) {
            continue;
        } else {
            while(i != ((int)count - 1)) {
                i++;
                float expect_angle = (nodebuffer[i-1].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f + inc_origin_angle;
                if (expect_angle > 360.0f) expect_angle -= 360.0f;
                u_int16_t checkbit = nodebuffer[i].angle_q6_checkbit & LIDAR_RESP_MEASUREMENT_CHECKBIT;
                nodebuffer[i].angle_q6_checkbit = (((u_int16_t)(expect_angle * 64.0f)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + checkbit;
            }
            break;
        }
    }

    float frontAngle = (nodebuffer[0].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
    for (i = 1; i < (int)count; i++) {
        if(nodebuffer[i].distance_q2 == 0) {
            float expect_angle =  frontAngle + i * inc_origin_angle;
            if (expect_angle > 360.0f) expect_angle -= 360.0f;
            u_int16_t checkbit = nodebuffer[i].angle_q6_checkbit & LIDAR_RESP_MEASUREMENT_CHECKBIT;
            nodebuffer[i].angle_q6_checkbit = (((u_int16_t)(expect_angle * 64.0f)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + checkbit;
        }
    }

    size_t zero_pos = 0;
    float pre_degree = (nodebuffer[0].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;

    for (i = 1; i < (int)count ; ++i) {
        float degree = (nodebuffer[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
        if (zero_pos == 0 && (pre_degree - degree > 180)) {
            zero_pos = i;
            break;
        }
        pre_degree = degree;
    }

    for (i = (int)zero_pos; i < (int)count; i++) {
        tmpbuffer[i-zero_pos] = nodebuffer[i];
    }
    for (i = 0; i < (int)zero_pos; i++) {
        tmpbuffer[i+(int)count-zero_pos] = nodebuffer[i];
    }

    memcpy(nodebuffer, tmpbuffer, count*sizeof(node_info));
    delete[] tmpbuffer;

    return 0;
}


