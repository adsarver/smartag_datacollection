#ifndef HW_INTERFACE_PLUGIN_NOVATEL_SPAN_HPP__
#define HW_INTERFACE_PLUGIN_NOVATEL_SPAN_HPP__

//always inlclude these
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <hw_interface/base_interface.hpp>

//include the header of the base type you want, Serial or UDP
#include <hw_interface/base_serial_interface.hpp>

//include ros message types
#include <sensor_msgs/Imu.h>
#include <hw_interface_plugin_novatel_span/NovatelGPSTime.h>

//file writing for GPS data
#include <fstream>

#define CRC32_POLYNOMIAL 0xEDB88320L
#define PI 3.14159265358979
#define RAD2DEG 180.0/PI
#define DEG2RAD PI/180.0
#define IMU_RATE 125.0 // Hz

namespace hw_interface_plugin_novatel_span {

    class novatel_span_serial : public base_classes::base_serial_interface
    {
    public:
        novatel_span_serial();
        ~novatel_span_serial();

    protected:

        //these methods are abstract as defined by the base_serial_interface
            //they must be defined
        bool subPluginInit(ros::NodeHandlePtr nhPtr);
        void setInterfaceOptions();
        bool interfaceReadHandler(const size_t &length, int arrayStartPos, const boost::system::error_code &ec);
        bool verifyChecksum();

        std::size_t novatelSpanStreamMatcher(const boost::system::error_code &error, long totalBytesInBuffer);

        long headerLen;
        long fullPacketLen;
        sensor_msgs::Imu imuMessage;
        ros::Publisher imuPublisher;
        hw_interface_plugin_novatel_span::NovatelGPSTime gpsTimeMsg;
        ros::Publisher gpsTimePub;
        double gyroScaleFactor;
        double accelScaleFactor;
        bool recordGPSData = false;
        std::ofstream gpsDataFile;

    private:
        unsigned long CRC32Value_(int i);
        unsigned long CalculateBlockCRC32_(unsigned long ulCount, unsigned char *ucBuffer ); // Number of bytes in the data block, Data block
    };

}

PLUGINLIB_EXPORT_CLASS(hw_interface_plugin_novatel_span::novatel_span_serial, base_classes::base_interface)



#endif //HW_INTERFACE_PLUGIN_NOVATEL_SPAN_HPP__
