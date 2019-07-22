/*!
 * \file novatel/novatel.h
 * \author David Hodo <david.hodo@gmail.com>
 * \version 1.0
 *
 * \section LICENSE
 *
 * The BSD License
 *
 * Copyright (c) 2011 David Hodo - Integrated Solutions for Systems (IS4S)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * \section DESCRIPTION
 *
 * This provides an interface for OEM 4, V, and 6 series of Novatel GPS receivers
 *
 * This library depends on CMake-2.4.6 or later: http://www.cmake.org/
 * This library depends on Serial: https://github.com/wjwwood/serial
 *
 */

#ifndef NOVATEL_H
#define NOVATEL_H
#pragma warning(disable: 4251)
#define _CRT_SECURE_NO_WARNINGS
#include <string>
#include <cstring> // for size_t
#include <map>
#include <memory>

// Structure definition headers
#include "novatel/novatel_enums.h"
#include "novatel/novatel_structures.h"
// Boost Headers
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

//#include <boost/condition_variable.hpp>
// Serial Headers
#include "serial/serial.h"

namespace novatel
{
    // used to convert lat and long to UTM coordinates
    #define GRAD_A_RAD(g) ((g)*0.0174532925199433)
    #define CRC32_POLYNOMIAL 0xEDB88320L

    // Constants for unpacking RANGECMP
    #define CMP_MAX_VALUE         8388608.0
    #define CMP_GPS_WAVELENGTH_L1 0.1902936727984
    #define CMP_GPS_WAVELENGTH_L2 0.2442102134246

    typedef boost::function<double()> GetTimeCallback;
    typedef boost::function<void()> HandleAcknowledgementCallback;

    // Messaging callbacks
    typedef boost::function<void(const std::string&)> LogMsgCallback;
    typedef boost::function<void(unsigned char*, size_t)> RawMsgCallback;

    typedef std::shared_ptr<BinaryMessageBase> BinaryMessagePtr;
    typedef boost::function<void(BinaryMessagePtr, double)> BinaryMessageCallback;


    class NOVATEL_EXPORT Novatel
    {
    public:
        Novatel();
        ~Novatel();

        /*!
         * Connects to the Novatel receiver given a serial port.
         *
         * @param port Defines which serial port to connect to in serial mode.
         * Examples: Linux - "/dev/ttyS0" Windows - "COM1"
         *
         * @throws ConnectionFailedException connection attempt failed.
         * @throws UnknownErrorCodeException unknown error code returned.
         */
        bool connect(const std::string&  port, int baudrate = 115200, bool search = true, bool check = true);

        /*!
         * Disconnects from the serial port
         */
        void disconnect();

        //! Indicates if a connection to the receiver has been established.
        bool isConnected();

        /*!
      
           * Pings the GPS to determine if it is properly connected
           *
           * This method sends a ping to the GPS and waits for a response.
           *
           * @param numAttempts The number of times to ping the device
           * before giving up
           *
           * @return True if the GPS was found, false if it was not.
           */
        bool ping(int numAttempts = 5);

        void setTimeHandler(const GetTimeCallback& timeHandler);

        //callbacks

        void setLogDebugCallback(const LogMsgCallback& debugCallback);;
        void setLogInfoCallback(const LogMsgCallback& infoCallback);;
        void setLogWarningCallback(const LogMsgCallback& warningCallback);;
        void setLogErrorCallback(const LogMsgCallback& errorCallback);;

        /*!
         * Request the given list of logs from the receiver.
         * Format: "[LOGNAME][MESSAGETYPE] [PORT] [LOGTYPE] [PERIOD] ..."
         * [MESSAGETYPE] - [A]=ASCII
         *               - [B]=Binary
         *               . [empty]=Abreviated ASCII
         * log_string format: "BESTUTMB ONTIME 1.0; BESTVELB ONTIME 1.0"
         */
        void configureLogs(const std::string& logString);
        void unlog(const std::string& log); //!< Stop logging a specified log
        void unlogAll(); //!< Stop logging all logs that aren't set with HOLD parameter

        /*!
         * SaveConfiguration() saves the current receiver configuration
         * in nonvolatile memory
         */
        void saveConfiguration();

        void configureInterfaceMode(const std::string& comPort,
            const std::string& rxMode, const std::string& txNode);

        void configureBaudRate(const std::string& comPort, int baudrate);

        bool sendCommand(const std::string& cmdMsg, bool waitForAck = true);
        //bool SendMessage(uint8_t* msg_ptr, size_t length);

        /*!
         * SetSvElevationCutoff
         * Sets the elevation cut-off angle. Svs below this angle
         * are not automatically searched for and are not used
         * in the position calculation. Angles < 5 deg are not
         * recommended except in specific situations
         * (Angle = +-90 deg)
         */
        bool setSvElevationAngleCutoff(float angle);

        /*!
         * Pseudrange/Delta-Phase filter (PDPFILTER)- smooths positions
         * and bridges gaps in GPS coverage. Enabled by default on
         * OEMStar receiver.
         */
        void setPDPFilterDisable();
        void setPDPFilterEnable();
        void setPDPFilterReset();
        void PDPModeConfigure(PDPMode mode, PDPDynamics dynamics);

        /*!
         * SetPositionTimeout (POSTIMEOUT) sets the timeout value for the
         * position calculation. In position logs, the position_type field
         * is set to NONE when this timeout expires (0 - 86400 sec)
         */
        void setPositionTimeout(uint32_t seconds);

        bool setInitialPosition(double latitude, double longitude, double height);
        bool setInitialTime(uint32_t gpsWeek, double gpsSeconds);
        bool injectAlmanac(Almanac almanac);
        /*!
         * SetL1CarrierSmoothing sets the amount of smoothing to be performed on
         * code measurements. L2 smoothing is available in OEMV receivers, but
         * NOT in OEMStar Firmaware receivers.
         *      l1_time_constant : 2<= time constant <= 2000 [sec]
         *      l2_time_constant : 5<= time constant <= 2000 [sec] (firmware default = 100)
         */
        bool setCarrierSmoothing(uint32_t l1TimeConstant, uint32_t l2TimeConstant);

        bool hardwareReset();
        /*!
         * HotStartReset
         * Restarts the GPS receiver, initialized with
         * Ephemeris, Almanac, Position, Time, etc.
         */
        bool hotStartReset();
        /*!
         * WarmStartReset
         * Restarts the GPS receiver, initialized with
         * Ephemeris, Almanac, NOT Position and Time info
         */
        bool warmStartReset();
        /*!
         * ColdStartReset
         * Restarts the GPS receiver, initialized without
         * any initial or aiding data.
         */
        bool coldStartReset();

        void sendRawEphemeridesToReceiver(const RawEphemerides& rawEphemerides);

        /*!
         * Requests version information from the receiver
         *
         * This requests the VERSION message from the receiver and
         * uses the result to populate the receiver capabilities
         *
         * @return True if the GPS was found, false if it was not.
         */
        bool updateVersion();

        bool convertLLaUtm(double lat, double Long, double* northing, double* easting, int* zone, bool* north);

        void readFromFile(unsigned char* buffer, unsigned int length);

        void setRawMsgCallback(RawMsgCallback handler);

        void setCallback(BINARY_LOG_TYPE typeId, const BinaryMessageCallback& callback);

        void setDefaultBinaryCallback(const BinaryMessageCallback& callback);

        void setDefaultAsciiCallback(const LogMsgCallback& callback);

        void setDefaultRtcmCallback(const RawMsgCallback& callback);

        RawEphemerides testEphems;

        void setTimeOut(uint32_t ms);

        void setEnableRawOutput(bool en = true);

        /*!
         * Starts a thread to continuously read from the serial port.
         *
         * Starts a thread that runs 'ReadSerialPort' which constatly reads
         * from the serial port.  When valid data is received, parse and then
         *  the data callback functions are called.
         *
         * @see xbow440::DataCallback, xbow440::XBOW440::ReadSerialPort, xbow440::XBOW440::StopReading
         */
        void startReading();

        /*!
         * Starts the thread that reads from the serial port
         *
         * @see xbow440::XBOW440::ReadSerialPort, xbow440::XBOW440::StartReading
         */
        void stopReading();


        //////////////////////////////////////////////////////
        // Receiver information and capabilities
        //////////////////////////////////////////////////////
        std::string protocolVersion; //!< Receiver version, OEM4, OEMV, OEM6, or UNKNOWN
        std::string serialNumber; //!< Receiver serial number
        std::string hardwareVersion; //!< Receiver hardware version
        std::string softwareVersion; //!< Receiver software version
        std::string model; //!< Receiver model number

        bool l2Capable; //!< Can the receiver handle L1 and L2 or just L1?
        bool rawCapable; //!< Can the receiver output raw measurements?
        bool rtkCapable; //!< Can the receiver compute RT2 and/or RT20 positions?
        bool glonassCapable; //!< Can the receiver receive GLONASS frequencies?
        bool spanCapable; //!< Is the receiver a SPAN unit?
    private:

        bool connect_(const std::string& port, int baudrate, bool doPing = true);

        /*!
         * Method run in a seperate thread that continuously reads from the
         * serial port.  When a complete packet is received, the parse
         * method is called to process the data
         *
         * @see xbow440::XBOW440::Parse, xbow440::XBOW440::StartReading, xbow440::XBOW440::StopReading
         */
        void readSerialPort();

        void bufferIncomingData(unsigned char* message, size_t length);
        bool checkBinaryFormat(unsigned char* msg, size_t length);
        bool checkAsciiFormat(unsigned char* msg, size_t length);
        bool checkRtcmFormat(unsigned char* msg, size_t length);
        bool checkAbbreviatedFormat(unsigned char* msg, size_t length);
        bool checkAck(unsigned char* msg, size_t length);
        bool checkReset(unsigned char* msg, size_t length);

        /*!
         * Parses a packet of data from the GPS. 
         */
        void parseBinary(unsigned char* message, size_t length, BINARY_LOG_TYPE messageId);
        /*
         * Parses a packet of data from the GPS including header [and CRC32].
         */
        void parseBinary(unsigned char* message, size_t length);
        void parseAscii(unsigned char* message, size_t length);
        void parseRtcm(unsigned char* message, size_t length);

        bool parseVersion(std::string packet);

        void unpackCompressedRangeData(const CompressedRangeData& cmp,
                                       RangeData& rng);

        double unpackCompressedPsrStd(const uint16_t& val) const;

        double unpackCompressedAccumulatedDoppler(
            const CompressedRangeData& cmp,
            const double& uncmpPsr) const;

        bool sendBinaryDataToReceiver(uint8_t* msgPtr, size_t length);

        //////////////////////////////////////////////////////
        // Serial port reading members
        //////////////////////////////////////////////////////
        //! Serial port object for communicating with sensor
        serial::Serial* serialPort;
        //! shared pointer to Boost thread for listening for data from novatel
        boost::shared_ptr<boost::thread> readThreadPtr;
        bool readingStatus; //!< True if the read thread is running, false otherwise.

        //////////////////////////////////////////////////////
        // Callbacks
        //////////////////////////////////////////////////////
        HandleAcknowledgementCallback handleAcknowledgement;
        LogMsgCallback logDebug;
        LogMsgCallback logInfo;
        LogMsgCallback logWarning;
        LogMsgCallback logError;

        GetTimeCallback timeHandler; //!< Function pointer to callback function for timestamping

        RawMsgCallback rawMsgCallback;

        std::map<BINARY_LOG_TYPE, BinaryMessageCallback> binaryCallbackMap;
        BinaryMessageCallback defaultBinaryCallback;
        LogMsgCallback defaultAsciiCallback;
        RawMsgCallback defaultRtcmCallback;

        //////////////////////////////////////////////////////
        // Incoming data buffers
        //////////////////////////////////////////////////////
        unsigned char dataBuffer[MAX_NOUT_SIZE]; //!< data currently being buffered to read
        unsigned char* dataRead = nullptr; //!< used only in BufferIncomingData - declared here for speed
        size_t bufferIndex = 0; //!< index into data_buffer_
        double readTimestamp = 0; //!< time stamp when last serial port read completed
        double parseTimestamp = 0; //!< time stamp when last parse began

        //////////////////////////////////////////////////////
        // Mutex's
        //////////////////////////////////////////////////////
        boost::condition_variable ackCondition;
        boost::mutex ackMutex;
        bool ackReceived; //!< true if an acknowledgement has been received from the GPS
        boost::condition_variable resetCondition;
        boost::mutex resetMutex;
        bool waitingForResetComplete = false; //!< true if GPS has finished resetting and is ready for input

        bool isConnectedB = false; //!< indicates if a connection to the receiver has been established

        bool enableRaw = false;
    };
}
#endif
