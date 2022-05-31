

#ifndef BASE_UDP_INTERFACE_HPP__
#define BASE_UDP_INTERFACE_HPP__


#include <ros/ros.h>

#include <hw_interface/base_interface.hpp>

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>

#define UDP_FRAME_BUFFER_SIZE 1500
#define UDP_MAX_PKT_SIZE 65500

namespace base_classes
{
    class base_UDP_interface : public base_interface
    {
        //friend class hw_interface;

    private:

        bool interfaceReady();
        bool initPlugin(ros::NodeHandlePtr nhPtr,
                            const boost::shared_ptr<boost::asio::io_service> ioService);
        bool startWork(); //probably set some flag and start listening on port
        bool stopWork();  //probably unset some flag

        //will receive request, then run plugin readHandler(), check work flag,
            //if work flag restart async read

    protected:

        base_UDP_interface();

        boost::shared_ptr<boost::asio::ip::udp::socket> interfaceSocket;
        boost::shared_ptr<boost::asio::ip::udp::endpoint> localEndpoint;
        boost::shared_ptr<boost::asio::ip::udp::endpoint> remoteEndpoint;
        boost::asio::ip::udp::endpoint senderEndpoint;

        //needs better name
        //this function calls the plugin's function to read in ROS params,
        //subscribe to topics, publish topics. This function should fill
        //in the protected member's info
        virtual bool subPluginInit(ros::NodeHandlePtr nhPtr) = 0;

        virtual bool pluginStart()
        {
            return true;
        }

        virtual bool pluginStop()
        {
            return true;
        }

        boost::asio::ip::address localAddress;
        int localPort;
        boost::asio::ip::address remoteAddress;
        int remotePort;

        //plugin provided data handler that moves data into ROS
        virtual bool interfaceReadHandler(const size_t &bufferSize, int arrayStartPos) = 0;

        void interfaceWriteHandler(const hw_interface_support_types::shared_const_buffer &buffer);
        void postInterfaceWriteRequest(const hw_interface_support_types::shared_const_buffer &buffer);

    public:
        bool handleIORequest(const boost::system::error_code &ec, size_t bytesReceived);

    };
};
#endif //BASE_UDP_INTERFACE_HPP__
