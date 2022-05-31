#include <hw_interface_plugin_novatel_span/hw_interface_plugin_novatel_span_serial.hpp>

//class constructor, do required instatiation only here
    //do not setup other things as they my not have been setup by the parent classes yet
hw_interface_plugin_novatel_span::novatel_span_serial::novatel_span_serial()
{
    //A debug message
    ROS_INFO("A Wild Novatel Span Plugin Appeared!");

    //force the ROS Console level to Debug Level
    //if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    //       ros::console::notifyLoggerLevelsChanged();
    //    }

    //enable automatic class metric collection.
    enableMetrics();

    // Compute IMU scale factors once for use later
    gyroScaleFactor = DEG2RAD*IMU_RATE*pow(2.0,-21.0); // rad/s/LSB
    accelScaleFactor = IMU_RATE*pow(2.0,-22.0); // m/s^2/LSB
    
    // Set first term in orientation covariance in imu message to -1 to signify no orientation data
    imuMessage.orientation_covariance[0] = -1;
}

//class destructor
hw_interface_plugin_novatel_span::novatel_span_serial::~novatel_span_serial()
{
    // Close GPS file, if open
    if(recordGPSData)
    {
        gpsDataFile.close();
    }
}

//this is called each time the plugin is enabled, before anything else of the plugin is called
bool hw_interface_plugin_novatel_span::novatel_span_serial::subPluginInit(ros::NodeHandlePtr nhPtr)
{
    ROS_DEBUG_EXTRA("%s Plugin Init", pluginName.c_str());

    /*for Serial interfaces, 'deviceName' is an inherited member and must be defined.
        failing to define this variable will disable the plugin.
        Opening of the device port is handled automatically

        deviceName is the name and path of the port to be opened
            example: "/dev/ttyS0" */
    deviceName = "";
    ros::param::get(pluginName+"/deviceName", deviceName);

    // bind a functor to the streamCompletionChecker inherited member
    streamCompletionChecker = boost::bind(&hw_interface_plugin_novatel_span::novatel_span_serial::novatelSpanStreamMatcher, this, _1, _2);

    // enable the completion functor if the bind succeeded
    enableCompletionFunctor = !streamCompletionChecker.empty();

    //retrieve string from the ROS Parameter Server
        //of the format '<plugin_name>/<parameter>
    std::string tempString = "";

    //place that wants to write data to the device
    if(ros::param::get(pluginName+"/subscribeToTopic", tempString))
    {
	//This will create a ros subscriber to the topic from the ROS parameter server.
	    //The rosMsgCallback method will be called whenever there is a message pending.
        //rosDataSub = nh->subscribe(tempString, 1, &roboteq_drive::rosMsgCallback, this);
    }
    else
    {
        ROS_ERROR("%s:: Could not find topic subscription name", pluginName.c_str());
    }

    //place to publish the data after reading from device
    if(ros::param::get(pluginName+"/publishToTopic", tempString))
    {
        imuPublisher = nhPtr->advertise<sensor_msgs::Imu>(tempString, 1, false);
    }
    else
    {
        ROS_ERROR("%s:: Could not find topic advertisment name", pluginName.c_str());
    }

    //get GPS data file name
    if(ros::param::get(pluginName+"/recordGPSData", recordGPSData) == false)
    {
        ROS_ERROR("No recordGPSData param provided");
        exit(1);
    }
    else
    {
        std::string gpsDataFileName;
        if(ros::param::get(pluginName+"/gpsDataFileName", gpsDataFileName) == false)
        {
            ROS_ERROR("No gpsDataFileName param provided");
            exit(1);
        }
        gpsDataFile.open(gpsDataFileName, std::ios::out | std::ios::trunc | std::ios::binary);
        gpsTimePub = nhPtr->advertise<hw_interface_plugin_novatel_span::NovatelGPSTime>("/gps_time", 1);
    }

    return true;
}

/*this function is called to setup the port
    typical serial port setup uses 115200 baud rate, 8 bit character size, no flow control,
      no parity, 1 stop bit. This is typical, but no all encompassing. Change this if
      the port requires different. */
void hw_interface_plugin_novatel_span::novatel_span_serial::setInterfaceOptions()
{
	int tempBaudRate = 0;
    ros::param::get(pluginName+"/baudrate", tempBaudRate);
    setOption<boost::asio::serial_port_base::baud_rate>(
                new boost::asio::serial_port_base::baud_rate(tempBaudRate));
    //8 bits per character
    setOption<boost::asio::serial_port_base::character_size>(
    			new boost::asio::serial_port_base::character_size( 8 ));

    //flow control
    setOption<boost::asio::serial_port_base::flow_control>(
    			new boost::asio::serial_port_base::flow_control(
    										boost::asio::serial_port_base::flow_control::type::none));

    //parity
    setOption<boost::asio::serial_port_base::parity>(
    			new boost::asio::serial_port_base::parity(
    										boost::asio::serial_port_base::parity::type::none));

    //stop-bits
    setOption<boost::asio::serial_port_base::stop_bits>(
    			new boost::asio::serial_port_base::stop_bits(
    										boost::asio::serial_port_base::stop_bits::type::one));

    ROS_INFO("%s :: Device: %s :: Baudrate %d", pluginName.c_str(), deviceName.c_str(), tempBaudRate);
}

//this is called automatically when data that passes the streamMatcher is okay
    //this function is called with a data length and a position in an inherited array member
        //named 'receivedData'
//data should be published to topics from here
bool hw_interface_plugin_novatel_span::novatel_span_serial::interfaceReadHandler(const size_t &length,
                                                                            int arrayStartPos, const boost::system::error_code &ec)
{
    //ROS_INFO_EXTRA_SINGLE("Novatel Span Plugin Data Handler");

    unsigned long packetChecksum = (((unsigned long)receivedData[arrayStartPos+fullPacketLen-4]&0xFF) + (((unsigned long)receivedData[arrayStartPos+fullPacketLen-3]&0xFF)<<8) + (((unsigned long)receivedData[arrayStartPos+fullPacketLen-2]&0xFF)<<16) + (((unsigned long)receivedData[arrayStartPos+fullPacketLen-1]&0xFF)<<24));
    unsigned long computedChecksum = CalculateBlockCRC32_(length-arrayStartPos-4, &receivedData[arrayStartPos]);
    if(packetChecksum != computedChecksum)
    {      
        ROS_WARN("Checksum failed!");
    }
    else
    {
        if(((unsigned short)((receivedData[arrayStartPos+4]&0xFF) + ((receivedData[arrayStartPos+5]&0xFF)<<8))) == 325) // Check if message ID is RAWIMUS (325)
        {
            // populate IMU message accelerations and angular velocities
            imuMessage.linear_acceleration.z = accelScaleFactor*((double)((receivedData[arrayStartPos+headerLen+16]&0xFF) +
                                                    ((receivedData[arrayStartPos+headerLen+17]&0xFF)<<8) +
                                                    ((receivedData[arrayStartPos+headerLen+18]&0xFF)<<16) +
                                                    ((receivedData[arrayStartPos+headerLen+19]&0xFF)<<24))); // m/s^2

            imuMessage.linear_acceleration.y = -accelScaleFactor*((double)((receivedData[arrayStartPos+headerLen+20]&0xFF) +
                                                    ((receivedData[arrayStartPos+headerLen+21]&0xFF)<<8) +
                                                    ((receivedData[arrayStartPos+headerLen+22]&0xFF)<<16) +
                                                    ((receivedData[arrayStartPos+headerLen+23]&0xFF)<<24))); // m/s^2

            imuMessage.linear_acceleration.x = accelScaleFactor*((double)((receivedData[arrayStartPos+headerLen+24]&0xFF) +
                                                    ((receivedData[arrayStartPos+headerLen+25]&0xFF)<<8) +
                                                    ((receivedData[arrayStartPos+headerLen+26]&0xFF)<<16) +
                                                    ((receivedData[arrayStartPos+headerLen+27]&0xFF)<<24))); // m/s^2

            imuMessage.angular_velocity.z = gyroScaleFactor*((double)((receivedData[arrayStartPos+headerLen+28]&0xFF) +
                                                    ((receivedData[arrayStartPos+headerLen+29]&0xFF)<<8) +
                                                    ((receivedData[arrayStartPos+headerLen+30]&0xFF)<<16) +
                                                    ((receivedData[arrayStartPos+headerLen+31]&0xFF)<<24))); // rad/s

            imuMessage.angular_velocity.y = -gyroScaleFactor*((double)((receivedData[arrayStartPos+headerLen+32]&0xFF) +
                                                    ((receivedData[arrayStartPos+headerLen+33]&0xFF)<<8) +
                                                    ((receivedData[arrayStartPos+headerLen+34]&0xFF)<<16) +
                                                    ((receivedData[arrayStartPos+headerLen+35]&0xFF)<<24))); // rad/s

            imuMessage.angular_velocity.x = gyroScaleFactor*((double)((receivedData[arrayStartPos+headerLen+36]&0xFF) +
                                                    ((receivedData[arrayStartPos+headerLen+37]&0xFF)<<8) +
                                                    ((receivedData[arrayStartPos+headerLen+38]&0xFF)<<16) +
                                                    ((receivedData[arrayStartPos+headerLen+39]&0xFF)<<24))); // rad/s

            // populate IMU message header
            imuMessage.header.stamp = ros::Time::now();
            imuMessage.header.frame_id = "novatel_imu_link";

            // publish message
            imuPublisher.publish(imuMessage);

            // increment sequence number for next cycle
            imuMessage.header.seq++;
        }
        else if(((unsigned short)((receivedData[arrayStartPos+4]&0xFF) + ((receivedData[arrayStartPos+5]&0xFF)<<8))) == 140) // Check if message ID is RANGECMP (140)
        {
            // Extract GPS week and milliseconds
            unsigned short gpsWeek = (unsigned short)((unsigned short)(receivedData[arrayStartPos+14]&0xFF) + ((unsigned short)(receivedData[arrayStartPos+15]&0xFF)<<8));
            long gpsMilliseconds = (long)((long)(receivedData[arrayStartPos+16]&0xFF) +
                                           ((long)(receivedData[arrayStartPos+17]&0xFF)<<8) + 
                                           ((long)(receivedData[arrayStartPos+18]&0xFF)<<16) +
                                           ((long)(receivedData[arrayStartPos+19]&0xFF)<<24));
            // Publish GPS time ROS message
            gpsTimeMsg.header.stamp = ros::Time::now();
            gpsTimeMsg.week = gpsWeek;
            gpsTimeMsg.milliseconds = gpsMilliseconds;
            gpsTimePub.publish(gpsTimeMsg);

            // increment sequency number for next cycle
            gpsTimeMsg.header.seq++;

            // Copy boost shared array to local char array
            char localArray[length];
            for(unsigned int i = 0; i < length; i++)
            {
                localArray[i] = receivedData[i];
            }

            // Save GPS data to file
            gpsDataFile.write(localArray+arrayStartPos, length);
        }
    }
    
    /*ROS_INFO("Buf Pointer: 0x%p\r\n", &receivedData[arrayStartPos]);
    ROS_INFO("length = %u, arrayStartPos = %i",length, arrayStartPos);
    std::printf("Contents: ");
    for(int i = 0; i < length; i++)
    {
        std::printf("%x | ", receivedData[arrayStartPos + i]);
    }
    std::printf("\r\n");*/

    return true;
}

//automatically called to check the checksum of the packet.
    //If un-wanted/un-needed, return true.
bool hw_interface_plugin_novatel_span::novatel_span_serial::verifyChecksum()
{
    return true;
}

std::size_t hw_interface_plugin_novatel_span::novatel_span_serial::novatelSpanStreamMatcher(const boost::system::error_code &error, long totalBytesInBuffer)
{
    const long syncSeqLen = 3;
    headerLen = 0;
    fullPacketLen = 0;
    dataArrayStart = 0;
    ROS_DEBUG("START: totalBytesInBuffer = %li",totalBytesInBuffer);
    if(error != boost::asio::error::operation_aborted)
    {
        if(totalBytesInBuffer < syncSeqLen)
        {
            //ROS_DEBUG_EXTRA_NAME("Returning Early, %ld", syncSeqLen-totalBytesInBuffer);
            ROS_DEBUG("not enough bytes for sync secquence");
            return syncSeqLen-totalBytesInBuffer;
        }
        for(long i=0; i<totalBytesInBuffer; i++)
        {
            if((receivedData[i]&0xFF) == 0xAA) // Find first sync char
            {
                ROS_DEBUG("0. find first sync sequence char");
                if((totalBytesInBuffer-(i+1)) < (syncSeqLen-1)) // Not enough to find rest of rest of sync sequence, read remaining
                {
                    ROS_DEBUG("not enough to find rest of sync sequence");
                    return syncSeqLen-1;
                }
                else
                {
                    if((receivedData[i+1]&0xFF) == 0x44) // Find second sync char
                    {
                        ROS_DEBUG("1. find second sync sequence char");
                        if((totalBytesInBuffer-(i+1)-1) < (syncSeqLen-2)) // Not enough to find rest of rest of sync sequence, read remaining
                        {
                            ROS_DEBUG("not enough to find rest of sync sequence");
                            return syncSeqLen-2;
                        }
                        else
                        {
                            if(((receivedData[i+2]&0xFF) == 0x13) || ((receivedData[i+2]&0xFF) == 0x12)) // Find third sync char
                            {
                                ROS_DEBUG("2. find third sync sequence char: %X",(receivedData[i+2]&0xFF));
                                dataArrayStart = i;
                                if((receivedData[i+2]&0xFF) == 0x12 && ((totalBytesInBuffer-(i+1)-2) < 1)) // Not enough to find header length, read another byte
                                {
                                    ROS_DEBUG("not enough to find header len");
                                    return 1;
                                }
                                else
                                {
                                    ROS_DEBUG("3. find header len");
                                    if((receivedData[i+2]&0xFF) == 0x12)
                                    {
                                        headerLen = (long)(receivedData[i+3]&0xFF); // Record the length of the header
                                        ROS_DEBUG("FULL headerLen = %i",headerLen);
                                    }
                                    else
                                    {
                                        headerLen = 12;
                                        ROS_DEBUG("SHORT headerLen = %i",headerLen);
                                    }
                                    if(totalBytesInBuffer-dataArrayStart < headerLen) // Not enough to read full header, read remaining header length
                                    {
                                        ROS_DEBUG("not enough to find rest of header");
                                        return headerLen-(totalBytesInBuffer-dataArrayStart);
                                    }
                                    else
                                    {
                                        ROS_DEBUG("4. find full header");
                                        if((receivedData[i+2]&0xFF) == 0x12)
                                        {
                                            fullPacketLen = (long)((long)(receivedData[i+8]&0xFF) + ((long)(receivedData[i+9]&0xFF)<<8)) + headerLen + 4; // Record full packet length (message length + header length + 4 byte CRC)
                                            ROS_DEBUG("FULL message len = %lu, full packet len = %lu",(long)(receivedData[i+8]&0xFF + ((receivedData[i+9]&0xFF)<<8)), fullPacketLen);
                                        }
                                        else
                                        {
                                            fullPacketLen = (long)(receivedData[i+3]&0xFF) + headerLen + 4;
                                            ROS_DEBUG("SHORT message len = %lu, full packet len = %lu",((long)(receivedData[i+3]&0xFF)), fullPacketLen);
                                        }
                                        if(totalBytesInBuffer-dataArrayStart < fullPacketLen) // Not enough bytes to be full packet, read remaining
                                        {
                                            ROS_DEBUG("not enough to find rest of packet");
                                            return fullPacketLen-(totalBytesInBuffer-dataArrayStart);
                                        }
                                        else
                                        {
                                            ROS_DEBUG("5. find full packet");
                                            return 0; // Full packet length found, no need to read more
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

unsigned long hw_interface_plugin_novatel_span::novatel_span_serial::CRC32Value_(int i)
{
    int j;
    unsigned long ulCRC;
    ulCRC = i;
    for ( j = 8 ; j > 0; j-- )
    {
        if ( ulCRC & 1 )
            ulCRC = ( ulCRC >> 1 ) ^ CRC32_POLYNOMIAL;
        else
            ulCRC >>= 1;
    }
    return ulCRC;
}

unsigned long hw_interface_plugin_novatel_span::novatel_span_serial::CalculateBlockCRC32_(unsigned long ulCount, unsigned char *ucBuffer)
{
    unsigned long ulTemp1;
    unsigned long ulTemp2;
    unsigned long ulCRC = 0;
    while ( ulCount-- != 0 )
    {
        ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
        ulTemp2 = CRC32Value_( ((int) ulCRC ^ *ucBuffer++ ) & 0xff );
        ulCRC = ulTemp1 ^ ulTemp2;
    }
    return ulCRC;
}
