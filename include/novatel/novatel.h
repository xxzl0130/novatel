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
        bool Connect(const std::string&  port, int baudrate = 115200, bool search = true);

        /*!
         * Disconnects from the serial port
         */
        void Disconnect();

        //! Indicates if a connection to the receiver has been established.
        bool IsConnected() { return is_connected_; }

        /*!
      
           * Pings the GPS to determine if it is properly connected
           *
           * This method sends a ping to the GPS and waits for a response.
           *
           * @param num_attempts The number of times to ping the device
           * before giving up
           *
           * @return True if the GPS was found, false if it was not.
           */
        bool Ping(int num_attempts = 5);


        /*!
         * Pings the GPS to determine if it is properly connected
         *
         * This method sends a ping to the GPS and waits for a response.
         *
         * @return True if the GPS was found, false if it was not.
         */
        void set_time_handler(GetTimeCallback time_handler)
        {
            this->time_handler_ = time_handler;
        }

        void setLogDebugCallback(LogMsgCallback debug_callback) { log_debug_ = debug_callback; };
        void setLogInfoCallback(LogMsgCallback info_callback) { log_info_ = info_callback; };
        void setLogWarningCallback(LogMsgCallback warning_callback) { log_warning_ = warning_callback; };
        void setLogErrorCallback(LogMsgCallback error_callback) { log_error_ = error_callback; };

        /*!
         * Request the given list of logs from the receiver.
         * Format: "[LOGNAME][MESSAGETYPE] [PORT] [LOGTYPE] [PERIOD] ..."
         * [MESSAGETYPE] - [A]=ASCII
         *               - [B]=Binary
         *               . [empty]=Abreviated ASCII
         * log_string format: "BESTUTMB ONTIME 1.0; BESTVELB ONTIME 1.0"
         */
        void ConfigureLogs(const std::string& log_string);
        void Unlog(const std::string& log); //!< Stop logging a specified log
        void UnlogAll(); //!< Stop logging all logs that aren't set with HOLD parameter

        /*!
         * SaveConfiguration() saves the current receiver configuration
         * in nonvolatile memory
         */
        void SaveConfiguration();

        void ConfigureInterfaceMode(const std::string& com_port,
            const std::string& rx_mode, const std::string& tx_mode);

        void ConfigureBaudRate(const std::string& com_port, int baudrate);

        bool SendCommand(const std::string& cmd_msg, bool wait_for_ack = true);
        //bool SendMessage(uint8_t* msg_ptr, size_t length);

        /*!
         * SetSvElevationCutoff
         * Sets the elevation cut-off angle. Svs below this angle
         * are not automatically searched for and are not used
         * in the position calculation. Angles < 5 deg are not
         * recommended except in specific situations
         * (Angle = +-90 deg)
         */
        bool SetSvElevationAngleCutoff(float angle);

        /*!
         * Pseudrange/Delta-Phase filter (PDPFILTER)- smooths positions
         * and bridges gaps in GPS coverage. Enabled by default on
         * OEMStar receiver.
         */
        void PDPFilterDisable();
        void PDPFilterEnable();
        void PDPFilterReset();
        void PDPModeConfigure(PDPMode mode, PDPDynamics dynamics);

        /*!
         * SetPositionTimeout (POSTIMEOUT) sets the timeout value for the
         * position calculation. In position logs, the position_type field
         * is set to NONE when this timeout expires (0 - 86400 sec)
         */
        void SetPositionTimeout(uint32_t seconds);

        bool SetInitialPosition(double latitude, double longitude, double height);
        bool SetInitialTime(uint32_t gps_week, double gps_seconds);
        bool InjectAlmanac(Almanac almanac);
        /*!
         * SetL1CarrierSmoothing sets the amount of smoothing to be performed on
         * code measurements. L2 smoothing is available in OEMV receivers, but
         * NOT in OEMStar Firmaware receivers.
         *      l1_time_constant : 2<= time constant <= 2000 [sec]
         *      l2_time_constant : 5<= time constant <= 2000 [sec] (firmware default = 100)
         */
        bool SetCarrierSmoothing(uint32_t l1_time_constant, uint32_t l2_time_constant);

        bool HardwareReset();
        /*!
         * HotStartReset
         * Restarts the GPS receiver, initialized with
         * Ephemeris, Almanac, Position, Time, etc.
         */
        bool HotStartReset();
        /*!
         * WarmStartReset
         * Restarts the GPS receiver, initialized with
         * Ephemeris, Almanac, NOT Position and Time info
         */
        bool WarmStartReset();
        /*!
         * ColdStartReset
         * Restarts the GPS receiver, initialized without
         * any initial or aiding data.
         */
        bool ColdStartReset();

        void SendRawEphemeridesToReceiver(const RawEphemerides& raw_ephemerides);

        /*!
         * Requests version information from the receiver
         *
         * This requests the VERSION message from the receiver and
         * uses the result to populate the receiver capabilities
         *
         * @return True if the GPS was found, false if it was not.
         */
        bool UpdateVersion();

        bool ConvertLLaUTM(double Lat, double Long, double* northing, double* easting, int* zone, bool* north);

        void ReadFromFile(unsigned char* buffer, unsigned int length);

        void set_raw_msg_callback(RawMsgCallback handler)
        {
            raw_msg_callback_ = handler;
        }

        void setCallback(BINARY_LOG_TYPE typeId, const BinaryMessageCallback& callback)
        {
            binary_callback_map_[typeId] = callback;
        }
        void setDefaultBinaryCallback(const BinaryMessageCallback& callback)
        {
            defaultBinaryCallback = callback;
        }
        void setDefaultAsciiCallback(const LogMsgCallback& callback)
        {
            defaultAsciiCallback = callback;
        }
        void setDefaultRtcmCallback(const RawMsgCallback& callback)
        {
            defaultRtcmCallback = callback;
        }

        RawEphemerides test_ephems_;

        void setTimeOut(uint32_t ms);

        void setEnableRawOutput(bool en = true)
        {
            enableRaw = en;
        }
    private:

        bool Connect_(const std::string& port, int baudrate);


        /*!
         * Starts a thread to continuously read from the serial port.
         *
         * Starts a thread that runs 'ReadSerialPort' which constatly reads
         * from the serial port.  When valid data is received, parse and then
         *  the data callback functions are called.
         *
         * @see xbow440::DataCallback, xbow440::XBOW440::ReadSerialPort, xbow440::XBOW440::StopReading
         */
        void StartReading();

        /*!
         * Starts the thread that reads from the serial port
         *
         * @see xbow440::XBOW440::ReadSerialPort, xbow440::XBOW440::StartReading
         */
        void StopReading();

        /*!
         * Method run in a seperate thread that continuously reads from the
         * serial port.  When a complete packet is received, the parse
         * method is called to process the data
         *
         * @see xbow440::XBOW440::Parse, xbow440::XBOW440::StartReading, xbow440::XBOW440::StopReading
         */
        void ReadSerialPort();

        void BufferIncomingData(unsigned char* message, unsigned int length);
        bool CheckBinaryFormat(unsigned char* msg, unsigned int length);
        bool CheckAsciiFormat(unsigned char* msg, unsigned int length);
        bool CheckRtcmFormat(unsigned char* msg, unsigned int length);
        bool CheckAbbreviatedFormat(unsigned char* msg, unsigned int length);
        bool CheckACK(unsigned char* msg, unsigned int length);
        bool CheckReset(unsigned char* msg, unsigned int length);

        /*!
         * Parses a packet of data from the GPS. 
         */
        void ParseBinary(unsigned char* message, size_t length, BINARY_LOG_TYPE message_id);
        /*
         * Parses a packet of data from the GPS including header [and CRC32].
         */
        void ParseBinary(unsigned char* message, size_t length);
        void ParseAscii(unsigned char* message, size_t length);
        void ParseRtcm(unsigned char* message, size_t length);

        bool ParseVersion(std::string packet);

        void UnpackCompressedRangeData(const CompressedRangeData& cmp,
                                       RangeData& rng);

        double UnpackCompressedPsrStd(const uint16_t& val) const;

        double UnpackCompressedAccumulatedDoppler(
            const CompressedRangeData& cmp,
            const double& uncmpPsr) const;

        bool SendBinaryDataToReceiver(uint8_t* msg_ptr, size_t length);

        //////////////////////////////////////////////////////
        // Serial port reading members
        //////////////////////////////////////////////////////
        //! Serial port object for communicating with sensor
        serial::Serial* serial_port_;
        //! shared pointer to Boost thread for listening for data from novatel
        boost::shared_ptr<boost::thread> read_thread_ptr_;
        bool reading_status_; //!< True if the read thread is running, false otherwise.

        //////////////////////////////////////////////////////
        // Diagnostic Callbacks
        //////////////////////////////////////////////////////
        HandleAcknowledgementCallback handle_acknowledgement_;
        LogMsgCallback log_debug_;
        LogMsgCallback log_info_;
        LogMsgCallback log_warning_;
        LogMsgCallback log_error_;

        GetTimeCallback time_handler_; //!< Function pointer to callback function for timestamping


        //////////////////////////////////////////////////////
        // New Data Callbacks
        //////////////////////////////////////////////////////
        RawMsgCallback raw_msg_callback_;

        //////////////////////////////////////////////////////
        // Incoming data buffers
        //////////////////////////////////////////////////////
        unsigned char data_buffer_[MAX_NOUT_SIZE]; //!< data currently being buffered to read
        unsigned char* data_read_; //!< used only in BufferIncomingData - declared here for speed
        size_t bytes_remaining_; //!< bytes remaining to be read in the current message
        size_t buffer_index_; //!< index into data_buffer_
        size_t header_length_; //!< length of the current header being read
        bool reading_acknowledgement_; //!< true if an acknowledgement is being received
        bool reading_reset_complete_; //!< true if an {COM#} message confirming receiver reset if complete
        double read_timestamp_; //!< time stamp when last serial port read completed
        double parse_timestamp_; //!< time stamp when last parse began
        BINARY_LOG_TYPE message_id_; //!< message id of message currently being buffered

        //////////////////////////////////////////////////////
        // Mutex's
        //////////////////////////////////////////////////////
        boost::condition_variable ack_condition_;
        boost::mutex ack_mutex_;
        bool ack_received_; //!< true if an acknowledgement has been received from the GPS
        boost::condition_variable reset_condition_;
        boost::mutex reset_mutex_;
        bool waiting_for_reset_complete_; //!< true if GPS has finished resetting and is ready for input

        bool is_connected_; //!< indicates if a connection to the receiver has been established
        //////////////////////////////////////////////////////
        // Receiver information and capabilities
        //////////////////////////////////////////////////////
        std::string protocol_version_; //!< Receiver version, OEM4, OEMV, OEM6, or UNKNOWN
        std::string serial_number_; //!< Receiver serial number
        std::string hardware_version_; //!< Receiver hardware version
        std::string software_version_; //!< Receiver software version
        std::string model_; //!< Receiver model number

        bool l2_capable_; //!< Can the receiver handle L1 and L2 or just L1?
        bool raw_capable_; //!< Can the receiver output raw measurements?
        bool rtk_capable_; //!< Can the receiver compute RT2 and/or RT20 positions?
        bool glonass_capable_; //!< Can the receiver receive GLONASS frequencies?
        bool span_capable_; //!< Is the receiver a SPAN unit?

        std::map<BINARY_LOG_TYPE, BinaryMessageCallback> binary_callback_map_;
        BinaryMessageCallback defaultBinaryCallback;
        LogMsgCallback defaultAsciiCallback;
        RawMsgCallback defaultRtcmCallback;

        bool enableRaw = false;
    };
}
#endif
