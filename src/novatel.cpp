#include "novatel/novatel.h"
#include "crc32c/crc32c.h"

#include <cmath>
#include <iostream>
#include <valarray>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace std;
using namespace novatel;

/////////////////////////////////////////////////////
// includes for default time callback
#define WIN32_LEAN_AND_MEAN
#include "boost/date_time/posix_time/posix_time.hpp"
////////////////////////////////////////////////////


/* --------------------------------------------------------------------------
Calculate a CRC value to be used by CRC calculation functions.
-------------------------------------------------------------------------- */
unsigned long CRC32Value(int i)
{
    return crc32c::Crc32c((char*)&i,sizeof(i));
}


/* --------------------------------------------------------------------------
Calculates the CRC-32 of a block of data all at once
-------------------------------------------------------------------------- */
unsigned long CalculateBlockCRC32(unsigned long ulCount, /* Number of bytes in the data block */
                                  unsigned char* ucBuffer) /* Data block */
{
    return crc32c::Crc32c(ucBuffer, ulCount);
}

uint32_t CalculateBlockCRC24Q(unsigned long size,unsigned char *buf)
{
    const static unsigned long CRC24POLY = 0x01864CFBL;
    const static unsigned long CRC24MASK = 0x00FFFFFFL;
    const static unsigned long CRC24BIT =  0x01000000L;

    uint32_t crc = 0x00L;
    for(auto i = 0;i < size;++i)
    {
        crc ^= buf[i] << 16;
        for (i = 0; i < 8; i++) {
            crc <<= 1;
            if (crc & CRC24BIT) {
                crc ^= CRC24POLY;
            }
        }
    }
    return crc & CRC24MASK;
}


/*!
 * Default callback method for timestamping data.  Used if a
 * user callback is not set.  Returns the current time from the
 * CPU clock as the number of seconds from Jan 1, 1970
 */
inline double DefaultGetTime()
{
    boost::posix_time::ptime present_time(boost::posix_time::microsec_clock::universal_time());
    boost::posix_time::time_duration duration(present_time.time_of_day());
    return (double)(duration.total_milliseconds()) / 1000.0;
}

inline void SaveMessageToFile(unsigned char* message, size_t length, const char* filename)
{
    ofstream outfile;
    outfile.open(filename, ios::out | ios::app); // "./test_data/filename.txt"
    if (outfile.is_open())
    {
        for (int index = 0; index < length; index++)
        {
            outfile << message[index];
        }
    }
    outfile.close();
}


inline void printHex(unsigned char* data, int length)
{
    for (int i = 0; i < length; ++i)
    {
        printf("0x%X ", (unsigned)(unsigned char)data[i]);
    }
    printf("\n");
}


// stolen from: http://oopweb.com/CPP/Documents/CPPHOWTO/Volume/C++Programming-HOWTO-7.html
void Tokenize(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiters = " ")
{
    // Skip delimiters at beginning.
    std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
    // Find first "non-delimiter".
    std::string::size_type pos = str.find_first_of(delimiters, lastPos);

    while (std::string::npos != pos || std::string::npos != lastPos)
    {
        // Found a token, add it to the vector.
        tokens.push_back(str.substr(lastPos, pos - lastPos));
        // Skip delimiters.  Note the "not_of"
        lastPos = str.find_first_not_of(delimiters, pos);
        // Find next "non-delimiter"
        pos = str.find_first_of(delimiters, lastPos);
    }
}

inline void DefaultAcknowledgementHandler()
{
    ; //std::cout << "Acknowledgement received." << std::endl;
}

inline void DefaultDebugMsgCallback(const std::string& msg)
{
    ; //std::cout << "Novatel Debug: " << msg << std::endl;
}

inline void DefaultInfoMsgCallback(const std::string& msg)
{
    std::cout << "Novatel Info: " << msg << std::endl;
}

inline void DefaultWarningMsgCallback(const std::string& msg)
{
    std::cout << "Novatel Warning: " << msg << std::endl;
}

inline void DefaultErrorMsgCallback(const std::string& msg)
{
    std::cout << "Novatel Error: " << msg << std::endl;
}

inline void DefaultBestPositionCallback(BinaryMessagePtr data, double time_stamp)
{
    auto best_position = *(Position*)data.get();
    std::cout << "BESTPOS: \nGPS Week: " << best_position.header.gps_week <<
        "  GPS milliseconds: " << best_position.header.gps_millisecs << std::endl <<
        "  Latitude: " << best_position.latitude << std::endl <<
        "  Longitude: " << best_position.longitude << std::endl <<
        "  Height: " << best_position.height << std::endl << std::endl <<
        "  Solution status: " << best_position.solution_status << std::endl <<
        "  position type: " << best_position.position_type << std::endl <<
        "  number of svs tracked: " << (double)best_position.number_of_satellites << std::endl <<
        "  number of svs used: " << (double)best_position.number_of_satellites_in_solution << std::endl;
}

inline void DefaultRawEphemCallback(RawEphemeris ephemeris, double time_stamp)
{
    std::cout << "Got RAWEPHEM for PRN " << ephemeris.prn << std::endl;
}

Novatel::Novatel()
{
    serial_port_ = NULL;
    reading_status_ = false;
    time_handler_ = DefaultGetTime;
    handle_acknowledgement_ = DefaultAcknowledgementHandler;
    binary_callback_map_[BESTPOS_LOG_TYPE] = DefaultBestPositionCallback;
    log_debug_ = DefaultDebugMsgCallback;
    log_info_ = DefaultInfoMsgCallback;
    log_warning_ = DefaultWarningMsgCallback;
    log_error_ = DefaultErrorMsgCallback;
    buffer_index_ = 0;
    read_timestamp_ = 0;
    parse_timestamp_ = 0;
    ack_received_ = false;
    waiting_for_reset_complete_ = false;
    is_connected_ = false;
}

Novatel::~Novatel()
{
    Disconnect();
}

bool Novatel::Connect(const std::string& port, int baudrate, bool search)
{
    bool connected = Connect_(port, baudrate);

    if (!connected && search)
    {
        // search additional baud rates

        int bauds_to_search[9] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400};
        bool found = false;
        for (int ii = 0; ii < 9; ii++)
        {
            std::stringstream search_msg;
            search_msg << "Searching for receiver with baudrate: " << bauds_to_search[ii];
            log_info_(search_msg.str());
            if (Connect_(port, bauds_to_search[ii]))
            {
                found = true;
                break;
            }
        }

        // if the receiver was found on a different baud rate, 
        // change its setting to the selected baud rate and reconnect
        if (found)
        {
            // change baud rate to selected value
            std::stringstream cmd;
            cmd << "COM THISPORT " << baudrate << "\r\n";
            std::stringstream baud_msg;
            baud_msg << "Changing receiver baud rate to " << baudrate;
            log_info_(baud_msg.str());
            try
            {
                serial_port_->write(cmd.str());
            }
            catch (std::exception& e)
            {
                std::stringstream output;
                output << "Error changing baud rate: " << e.what();
                log_error_(output.str());
                return false;
            }
            Disconnect();
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
            connected = Connect_(port, baudrate);
        }
    }

    if (connected)
    {
        // start reading
        StartReading();
        is_connected_ = true;
        return true;
    }
    else
    {
        log_error_("Failed to connect.");
        return false;
    }
}

void Novatel::setTimeOut(uint32_t ms)
{
    auto t = serial::Timeout::simpleTimeout(ms);
    this->serial_port_->setTimeout(t);
}

bool Novatel::Connect_(const std::string& port, int baudrate = 115200)
{
    try
    {
        //serial::Timeout my_timeout(50, 200, 0, 200, 0); // 115200 working settings
        //serial_port_ = new serial::Serial(port,baudrate,my_timeout);
        if (!serial_port_)
        {
            //default timeout 200ms
            serial_port_ = new serial::Serial(port, baudrate, serial::Timeout::simpleTimeout(500));
        }

        if (!serial_port_->isOpen())
        {
            std::stringstream output;
            output << "Serial port: " << port << " failed to open." << std::endl;
            log_error_(output.str());
            delete serial_port_;
            serial_port_ = NULL;
            return false;
        }
        else
        {
            std::stringstream output;
            output << "Serial port: " << port << " opened successfully." << std::endl;
            log_info_(output.str());
        }

        // stop any incoming data and flush buffers
        //serial_port_->write("UNLOGALL\r\n");
        // wait for data to stop cominig in
        //boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        // clear serial port buffers
        serial_port_->flush();

        // look for GPS by sending ping and waiting for response
        
        if (!Ping())
        {
            std::stringstream output;
            output << "Novatel GPS not found on port: " << port << " at baudrate " << baudrate << std::endl;
            log_error_(output.str());
            delete serial_port_;
            serial_port_ = NULL;
            is_connected_ = false;
            return false;
        }
        
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error connecting to gps on com port " << port << ": " << e.what();
        log_error_(output.str());
        is_connected_ = false;
        return false;
    }

    return true;
}


void Novatel::Disconnect()
{
    log_info_("Novatel disconnecting.");
    StopReading();
    // sleep longer than the timeout period
    boost::this_thread::sleep(boost::posix_time::milliseconds(150));

    try
    {
        if ((serial_port_ != NULL) && (serial_port_->isOpen()))
        {
            log_info_("Sending UNLOGALL and closing port.");
            serial_port_->write("UNLOGALL\r\n");
            serial_port_->close();
            delete serial_port_;
            serial_port_ = NULL;
        }
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error during disconnect: " << e.what();
        log_error_(output.str());
    }
}

bool Novatel::Ping(int num_attempts)
{
    while ((num_attempts--) > 0)
    {
        std::stringstream output;
        output << "Searching for Novatel receiver..." << std::endl;
        log_info_(output.str());
        if (UpdateVersion())
        {
            std::stringstream output;
            output << "Found Novatel receiver." << std::endl;
            output << "\tModel: " << model_ << std::endl;
            output << "\tSerial Number: " << serial_number_ << std::endl;
            output << "\tHardware version: " << hardware_version_ << std::endl;
            output << "\tSoftware version: " << software_version_ << std::endl << std::endl;;
            output << "Receiver capabilities:" << std::endl;
            output << "\tL2: ";
            if (l2_capable_)
                output << "+" << std::endl;
            else
                output << "-" << std::endl;
            output << "\tRaw measurements: ";
            if (raw_capable_)
                output << "+" << std::endl;
            else
                output << "-" << std::endl;
            output << "\tRTK: ";
            if (rtk_capable_)
                output << "+" << std::endl;
            else
                output << "-" << std::endl;
            output << "\tSPAN: ";
            if (span_capable_)
                output << "+" << std::endl;
            else
                output << "-" << std::endl;
            output << "\tGLONASS: ";
            if (glonass_capable_)
                output << "+" << std::endl;
            else
                output << "-" << std::endl;
            log_info_(output.str());
            return true;
        }
    }

    // no response found
    return false;
}

void Novatel::SendRawEphemeridesToReceiver(const RawEphemerides& raw_ephemerides)
{
    try
    {
        for (uint8_t index = 0; index < MAX_NUM_SAT; index++)
        {
            cout << "SIZEOF: " << sizeof(raw_ephemerides.ephemeris[index]) << endl;
            if (sizeof(raw_ephemerides.ephemeris[index]) == 106 + HEADER_SIZE)
            {
                uint8_t* msg_ptr = (unsigned char*)&raw_ephemerides.ephemeris[index];
                bool result = SendBinaryDataToReceiver(msg_ptr, sizeof(raw_ephemerides.ephemeris[index]));
                if (result)
                    cout << "Sent RAWEPHEM for PRN " << (double)raw_ephemerides.ephemeris[index].prn << endl;
            }
        }
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error in Novatel::SendRawEphemeridesToReceiver(): " << e.what();
        log_error_(output.str());
    }
}

bool Novatel::SendBinaryDataToReceiver(uint8_t* msg_ptr, size_t length)
{
    try
    {
        stringstream output1;
        std::cout << length << std::endl;
        std::cout << "Message Pointer" << endl;
        printHex((unsigned char*)msg_ptr, length);
        size_t bytes_written;

        if ((serial_port_ != NULL) && (serial_port_->isOpen()))
        {
            bytes_written = serial_port_->write(msg_ptr, length);
        }
        else
        {
            log_error_("Unable to send message. Serial port not open.");
            return false;
        }
        // check that full message was sent to serial port
        if (bytes_written == length)
        {
            return true;
        }
        else
        {
            log_error_("Full message was not sent over serial port.");
            output1 << "Attempted to send " << length << "bytes. " << bytes_written << " bytes sent.";
            log_error_(output1.str());
            return false;
        }
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error in Novatel::SendBinaryDataToReceiver(): " << e.what();
        log_error_(output.str());
        return false;
    }
}

bool Novatel::SendCommand(const std::string& cmd_msg, bool wait_for_ack)
{
    try
    {
        // sends command to GPS receiver
        serial_port_->write(cmd_msg + "\r\n");
        // wait for acknowledgement (or 2 seconds)
        if (wait_for_ack)
        {
            boost::mutex::scoped_lock lock(ack_mutex_);
            boost::system_time const timeout = boost::get_system_time() + boost::posix_time::milliseconds(2000);
            if (ack_condition_.timed_wait(lock, timeout))
            {
                log_info_("Command `" + cmd_msg + "` sent to GPS receiver.");
                return true;
            }
            else
            {
                log_error_("Command '" + cmd_msg + "' failed.");
                return false;
            }
        }
        else
        {
            log_info_("Command `" + cmd_msg + "` sent to GPS receiver.");
            return true;
        }
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error in Novatel::SendCommand(): " << e.what();
        log_error_(output.str());
        return false;
    }
}

bool Novatel::SetSvElevationAngleCutoff(float angle)
{
    try
    {
        std::stringstream ang_cmd;
        ang_cmd << "ECUTOFF " << angle;
        return SendCommand(ang_cmd.str());
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error in Novatel::SetSvElevationCutoff(): " << e.what();
        log_error_(output.str());
        return false;
    }
}

void Novatel::PDPFilterDisable()
{
    try
    {
        std::stringstream pdp_cmd;
        pdp_cmd << "PDPFILTER DISABLE";
        bool result = SendCommand(pdp_cmd.str());
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error in Novatel::PDPFilterDisable(): " << e.what();
        log_error_(output.str());
    }
}

void Novatel::PDPFilterEnable()
{
    try
    {
        std::stringstream pdp_cmd;
        pdp_cmd << "PDPFILTER ENABLE";
        bool result = SendCommand(pdp_cmd.str());
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error in Novatel::PDPFilterEnable(): " << e.what();
        log_error_(output.str());
    }
}

void Novatel::PDPFilterReset()
{
    try
    {
        std::stringstream pdp_cmd;
        pdp_cmd << "PDPFILTER RESET";
        bool result = SendCommand(pdp_cmd.str());
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error in Novatel::PDPFilterReset(): " << e.what();
        log_error_(output.str());
    }
}

//! TODO: PROPAK DOESN"T ACCEPT, LIKES REV.1 PASSTOPASSMODE INSTEAD
void Novatel::PDPModeConfigure(PDPMode mode, PDPDynamics dynamics)
{
    try
    {
        std::stringstream pdp_cmd;

        pdp_cmd << "PDPMODE ";
        if (mode == PDP_NORMAL)
            pdp_cmd << "NORMAL ";
        else if (mode == PDP_RELATIVE)
            pdp_cmd << "RELATIVE ";
        else
        {
            log_error_("PDPModeConfigure() input 'mode'' is not valid!");
            return;
        }
        if (dynamics == AUTO)
            pdp_cmd << "AUTO";
        else if (dynamics == STATIC)
            pdp_cmd << "STATIC";
        else if (dynamics == DYNAMIC)
            pdp_cmd << "DYNAMIC";
        else
        {
            log_error_("PDPModeConfigure() input 'dynamics' is not valid!");
            return;
        }

        bool result = SendCommand(pdp_cmd.str());
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error in Novatel::PDPModeConfigure(): " << e.what();
        log_error_(output.str());
    }
}

void Novatel::SetPositionTimeout(uint32_t seconds)
{
    try
    {
        if (seconds <= 86400)
        {
            std::stringstream pdp_cmd;
            pdp_cmd << "POSTIMEOUT " << seconds;
            bool result = SendCommand(pdp_cmd.str());
        }
        else
            log_error_("Seconds is not a valid value!");
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error in Novatel::SetPositionTimeout(): " << e.what();
        log_error_(output.str());
    }
}

bool Novatel::SetInitialPosition(double latitude, double longitude, double height)
{
    std::stringstream pos_cmd;
    pos_cmd << "SETAPPROXPOS " << latitude << " " << longitude << " " << height;
    return SendCommand(pos_cmd.str());
}

bool Novatel::SetInitialTime(uint32_t gps_week, double gps_seconds)
{
    std::stringstream time_cmd;
    time_cmd << "SETAPPROXTIME " << gps_week << " " << gps_seconds;
    return SendCommand(time_cmd.str());
}

/*
uint8_t          sync1;          //!< start of packet first byte (0xAA)
uint8_t          sync2;          //!< start of packet second byte (0x44)
uint8_t          sync3;          //!< start of packet third  byte (0x12)
uint8_t          header_length; 	//!< Length of the header in bytes ( From start of packet )
uint16_t         message_id;    	//!< Message ID number
uint8_t          message_type;  	//!< Message type - binary, ascii, nmea, etc...
uint8_t          port_address;  	//!< Address of the data port the log was received on
uint16_t         message_length;	//!< Message length (Not including header or CRC)
uint16_t         sequence;      	//!< Counts down from N-1 to 0 for multiple related logs
uint8_t          idle;          	//!< Time the processor was idle in last sec between logs with same ID
uint8_t          time_status;    //!< Indicates the quality of the GPS time
uint16_t         gps_week;      	//!< GPS Week number
uint32_t         gps_millisecs; 	//!< Milliseconds into week
uint32_t         status;        	//!< Receiver status word
uint16_t         Reserved;      	//!< Reserved for internal use
uint16_t         version;       	//!< Receiver software build number (0-65535)
*/

bool Novatel::InjectAlmanac(Almanac almanac)
{
    try
    {
        MessageType type;
        type.format = BINARY;
        type.response = ORIGINAL_MESSAGE;

        almanac.header.sync1 = NOVATEL_SYNC_BYTE_1;
        almanac.header.sync2 = NOVATEL_SYNC_BYTE_2;
        almanac.header.sync3 = NOVATEL_SYNC_BYTE_3;
        almanac.header.header_length = HEADER_SIZE;
        almanac.header.message_id = ALMANAC_LOG_TYPE;
        almanac.header.message_type = type;
        almanac.header.port_address = THISPORT;
        almanac.header.message_length = 4 + almanac.number_of_prns * 112;
        almanac.header.sequence = 0;
        almanac.header.idle = 0; //!< ignored on input
        almanac.header.time_status = 0; //!< ignored on input
        almanac.header.gps_week = 0; //!< ignored on input
        almanac.header.gps_millisecs = 0; //!< ignored on input
        almanac.header.status = 0; //!< ignored on input
        almanac.header.Reserved = 0; //!< ignored on input
        almanac.header.version = 0; //!< ignored on input

        log_info_(std::string("SIZEOF: ") + std::to_string(sizeof(almanac)));
        uint8_t* msg_ptr = (unsigned char*)&almanac;
        uint32_t crc = CalculateBlockCRC32(sizeof(almanac) - 4, msg_ptr);
        memcpy(almanac.crc, &crc, sizeof(crc)); // TODO: check byte ordering for crc
        bool result = SendBinaryDataToReceiver(msg_ptr, sizeof(almanac));
        if (result)
        {
            log_info_("Sent ALMANAC.");
            return true;
        }
        return false;
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error in Novatel::InjectAlmanac(): " << e.what();
        log_error_(output.str());
        return false;
    }
}

bool Novatel::SetCarrierSmoothing(uint32_t l1_time_constant, uint32_t l2_time_constant)
{
    try
    {
        std::stringstream smooth_cmd;
        if ((2 >= l1_time_constant) || (l1_time_constant >= 2000))
        {
            log_error_("Error in SetCarrierSmoothing: l1_time_constant set to improper value.");
            return false;
        }
        else if ((5 >= l2_time_constant) || (l2_time_constant >= 2000))
        {
            log_error_("Error in SetCarrierSmoothing: l2_time_constant set to improper value.");
            return false;
        }
        else
        {
            smooth_cmd << "CSMOOTH " << l1_time_constant << " " << l2_time_constant;
        }
        return SendCommand(smooth_cmd.str());
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error in Novatel::SetCarrierSmoothing(): " << e.what();
        log_error_(output.str());
        return false;
    }
}

bool Novatel::HardwareReset()
{
    // Resets receiver to cold start, does NOT clear non-volatile memory!
    try
    {
        bool command_sent = SendCommand("RESET", false);
        if (command_sent)
        {
            boost::mutex::scoped_lock lock(reset_mutex_);
            waiting_for_reset_complete_ = true;
            boost::system_time const timeout = boost::get_system_time() + boost::posix_time::milliseconds(5000);
            if (reset_condition_.timed_wait(lock, timeout))
            {
                log_info_("Hardware Reset Complete.");
                return true;
            }
            else
            {
                log_error_("Hardware Reset never Completed.");
                waiting_for_reset_complete_ = false;
                return false;
            }
        }
        else
        {
            return false;
        }
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        waiting_for_reset_complete_ = false;
        output << "Error in Novatel::HardwareReset(): " << e.what();
        log_error_(output.str());
        return false;
    }
}

bool Novatel::HotStartReset()
{
    try
    {
        bool command_sent = SendCommand("RESET", false);
        if (command_sent)
        {
            boost::mutex::scoped_lock lock(reset_mutex_);
            waiting_for_reset_complete_ = true;
            boost::system_time const timeout = boost::get_system_time() + boost::posix_time::milliseconds(10000);
            if (reset_condition_.timed_wait(lock, timeout))
            {
                log_info_("HotStartReset Complete.");
                return true;
            }
            else
            {
                log_error_("HotStartReset never Completed.");
                waiting_for_reset_complete_ = false;
                return false;
            }
        }
        else
        {
            return false;
        }
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        waiting_for_reset_complete_ = false;
        output << "Error in Novatel::HotStartReset(): " << e.what();
        log_error_(output.str());
        return false;
    }
}

bool Novatel::WarmStartReset()
{
    try
    {
        std::stringstream rst_pos_cmd;
        std::stringstream rst_time_cmd;
        rst_pos_cmd << "FRESET " << LAST_POSITION; //!< FRESET doesn't reply with an ACK
        bool pos_reset = SendCommand(rst_pos_cmd.str(), false);
        rst_time_cmd << "FRESET " << LBAND_TCXO_OFFSET; //!< FRESET doesn't reply with an ACK
        bool time_reset = SendCommand(rst_time_cmd.str(), false);
        if (pos_reset && time_reset)
        {
            boost::mutex::scoped_lock lock(reset_mutex_);
            waiting_for_reset_complete_ = true;
            boost::system_time const timeout = boost::get_system_time() + boost::posix_time::milliseconds(10000);
            if (reset_condition_.timed_wait(lock, timeout))
            {
                log_info_("WarmStartReset Complete.");
                return true;
            }
            else
            {
                log_error_("WarmStartReset never Completed.");
                waiting_for_reset_complete_ = false;
                return false;
            }
        }
        else
        {
            return false;
        }
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        waiting_for_reset_complete_ = false;
        output << "Error in Novatel::WarmStartReset(): " << e.what();
        log_error_(output.str());
        return false;
    }
}

bool Novatel::ColdStartReset()
{
    try
    {
        bool command_sent = SendCommand("FRESET STANDARD", false); //!< FRESET doesn't reply with an ACK
        if (command_sent)
        {
            boost::mutex::scoped_lock lock(reset_mutex_);
            waiting_for_reset_complete_ = true;
            boost::system_time const timeout = boost::get_system_time() + boost::posix_time::milliseconds(10000);
            if (reset_condition_.timed_wait(lock, timeout))
            {
                log_info_("ColdStartReset Complete.");
                return true;
            }
            else
            {
                log_error_("ColdStartReset never Completed.");
                waiting_for_reset_complete_ = false;
                return false;
            }
        }
        else
        {
            return false;
        }
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        waiting_for_reset_complete_ = false;
        output << "Error in Novatel::ColdStartReset(): " << e.what();
        log_error_(output.str());
        return false;
    }
}

void Novatel::SaveConfiguration()
{
    try
    {
        bool result = SendCommand("SAVECONFIG");
        if (result)
            log_info_("Receiver configuration has been saved.");
        else
            log_error_("Failed to save receiver configuration!");
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error in Novatel::SaveConfiguration(): " << e.what();
        log_error_(output.str());
    }
}

void Novatel::ConfigureLogs(const std::string& log_string)
{
    // parse log_string on semicolons (;)
    std::vector<std::string> logs;

    Tokenize(log_string, logs, ";");

    // request each log from the receiver and wait for an ack
    for (std::vector<std::string>::iterator it = logs.begin(); it != logs.end(); ++it)
    {
        // try each command up to five times
        int ii = 0;
        while (ii < 5)
        {
            try
            {
                // send log command to gps (e.g. "LOG BESTUTMB ONTIME 1.0")
                serial_port_->write("LOG " + *it + "\r\n");
                std::stringstream cmd;
                cmd << "LOG " << *it << "\r\n";
                log_info_(cmd.str());
                // wait for acknowledgement (or 2 seconds)
                boost::mutex::scoped_lock lock(ack_mutex_);
                boost::system_time const timeout = boost::get_system_time() + boost::posix_time::milliseconds(2000);
                if (ack_condition_.timed_wait(lock, timeout))
                {
                    log_info_("Ack received for requested log: " + *it);
                    break;
                }
                else
                {
                    log_error_("No acknowledgement received for log: " + *it);
                }
                ii++;
            }
            catch (std::exception& e)
            {
                std::stringstream output;
                output << "Error configuring receiver logs: " << e.what();
                log_error_(output.str());
            }
        }
    }
}

void Novatel::Unlog(const std::string& log)
{
    try
    {
        bool result = SendCommand("UNLOG");
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error in Novatel::Unlog(): " << e.what();
        log_error_(output.str());
    }
}

void Novatel::UnlogAll()
{
    try
    {
        bool result = SendCommand("UNLOGALL THISPORT");
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error in Novatel::UnlogAll(): " << e.what();
        log_error_(output.str());
    }
}

void Novatel::ConfigureInterfaceMode(const std::string& com_port,
    const std::string& rx_mode, const std::string& tx_mode)
{
    try
    {
        // send command to set interface mode on com port
        // ex: INTERFACEMODE COM2 RX_MODE TX_MODE
        serial_port_->write("INTERFACEMODE " + com_port + " " + rx_mode + " " + tx_mode + "\r\n");
        // wait for acknowledgement (or 2 seconds)
        boost::mutex::scoped_lock lock(ack_mutex_);
        boost::system_time const timeout = boost::get_system_time() + boost::posix_time::milliseconds(2000);
        if (ack_condition_.timed_wait(lock, timeout))
        {
            log_info_("Ack received.  Interface mode for port " +
                com_port + " set to: " + rx_mode + " " + tx_mode);
        }
        else
        {
            log_error_("No acknowledgement received for interface mode command.");
        }
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error configuring interface mode: " << e.what();
        log_error_(output.str());
    }
}

void Novatel::ConfigureBaudRate(const std::string& com_port, int baudrate)
{
    try
    {
        // send command to set baud rate on GPS com port
        // ex: COM com1 9600 n 8 1 n off on
        serial_port_->write("COM " + com_port + " " + std::to_string(baudrate) + " n 8 1 n off on\r\n");
        // wait for acknowledgement (or 2 seconds)
        boost::mutex::scoped_lock lock(ack_mutex_);
        boost::system_time const timeout = boost::get_system_time() + boost::posix_time::milliseconds(2000);
        if (ack_condition_.timed_wait(lock, timeout))
        {
            std::stringstream log_out;
            log_out << "Ack received.  Baud rate on com port " <<
                com_port << " set to " << baudrate << std::endl;
            log_info_(log_out.str());
        }
        else
        {
            log_error_("No acknowledgement received for com configure command.");
        }
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error configuring baud rate: " << e.what();
        log_error_(output.str());
    }
}

bool Novatel::UpdateVersion()
{
    // request the receiver version and wait for a response
    // example response:
    //#VERSIONA,COM1,0,71.5,FINESTEERING,1362,340308.478,00000008,3681,2291;
    //    1,GPSCARD,"L12RV","DZZ06040010","OEMV2G-2.00-2T","3.000A19","3.000A9",
    //    "2006/Feb/ 9","17:14:33"*5e8df6e0

    try
    {
        // clear port
        serial_port_->flush();
        // read out any data currently in the buffer
        std::string read_data = serial_port_->read(5000);
        while (read_data.length())
            read_data = serial_port_->read(5000);

        // send request for version
        serial_port_->write("log versiona once\r\n");
        // wait for response from the receiver
        //boost::this_thread::sleep(boost::posix_time::milliseconds(200));
        // read from the serial port until a new line character is seen
        std::string gps_response = serial_port_->read(15000);

        std::vector<std::string> packets;

        Tokenize(gps_response, packets, "\n");

        // loop through all packets in file and check for version messages
        // stop when the first is found or all packets are read
        for (size_t ii = 0; ii < packets.size(); ii++)
        {
            if (ParseVersion(packets[ii]))
            {
                return true;
            }
        }
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error reading version info from receiver: " << e.what();
        log_error_(output.str());
        return false;
    }


    return false;
}

bool Novatel::ParseVersion(std::string packet)
{
    // parse the results - message should start with "#VERSIONA"
    size_t found_version = packet.find("VERSIONA");
    if (found_version == string::npos)
        return false;

    // parse version information
    // remove header
    size_t pos = packet.find(";");
    if (pos == string::npos)
    {
        log_error_("Error parsing received version."
            " End of message was not found");
        log_debug_(packet);
        return false;
    }

    // remove header from message
    std::string message = packet.substr(pos + 1, packet.length() - pos - 2);
    // parse message body by tokening on ","
    typedef boost::tokenizer<boost::char_separator<char>>
        tokenizer;
    boost::char_separator<char> sep(",");
    tokenizer tokens(message, sep);
    // set up iterator to go through token list
    tokenizer::iterator current_token = tokens.begin();
    int number_components = atoi((*tokens.begin()).c_str());
    // make sure the correct number of tokens were found
    int token_count = 0;
    for (const auto& it : tokens)
    {
        //log_debug_(*current_token);
        token_count++;
    }

    // should be 9 tokens, if not something is wrong
    if (token_count != (8 * number_components + 1))
    {
        log_error_("Error parsing received version. "
            "Incorrect number of tokens found.");
        std::stringstream err_out;
        err_out << "Found: " << token_count << "  Expected: " << (8 * number_components + 1);
        log_error_(err_out.str());
        log_debug_(packet);
        return false;
    }

    current_token = tokens.begin();
    // device type is 2nd token
    string device_type = *(++current_token);
    // model is 3rd token
    model_ = *(++current_token);
    // serial number is 4th token
    serial_number_ = *(++current_token);
    // model is 5rd token
    hardware_version_ = *(++current_token);
    // model is 6rd token
    software_version_ = *(++current_token);

    // parse the version:
    if (hardware_version_.length() > 3)
        protocol_version_ = hardware_version_.substr(1, 4);
    else
        protocol_version_ = "UNKNOWN";

    // parse model number:
    // is the receiver capable of raw measurements?
    if (model_.find("L") != string::npos)
        raw_capable_ = true;
    else
        raw_capable_ = false;

    // can the receiver receive L2?
    if (model_.find("12") != string::npos)
        l2_capable_ = true;
    else
        l2_capable_ = false;

    // can the receiver receive GLONASS?
    if (model_.find("G") != string::npos)
        glonass_capable_ = true;
    else
        glonass_capable_ = false;

    // Is this a SPAN unit?
    if ((model_.find("I") != string::npos) || (model_.find("J") != string::npos))
        span_capable_ = true;
    else
        span_capable_ = false;

    // Can the receiver process RTK?
    if (model_.find("R") != string::npos)
        rtk_capable_ = true;
    else
        rtk_capable_ = false;


    // fix for oem4 span receivers - do not use l12 notation
    // i think all oem4 spans are l1 l2 capable and raw capable
    if ((protocol_version_ == "OEM4") && (span_capable_))
    {
        l2_capable_ = true;
        raw_capable_ = true;
    }

    return true;
}

void Novatel::StartReading()
{
    if (reading_status_)
        return;
    // create thread to read from sensor
    reading_status_ = true;
    read_thread_ptr_ = boost::shared_ptr<boost::thread>
        (new boost::thread(boost::bind(&Novatel::ReadSerialPort, this)));
}

void Novatel::StopReading()
{
    reading_status_ = false;
}

void Novatel::ReadSerialPort()
{
    unsigned char buffer[MAX_NOUT_SIZE];
    size_t len;
    log_info_("Started read thread.");

    // continuously read data from serial port
    while (reading_status_)
    {
        try
        {
            // read data
            len = serial_port_->read(buffer, MAX_NOUT_SIZE);
        }
        catch (std::exception& e)
        {
            std::stringstream output;
            output << "Error reading from serial port: " << e.what();
            log_error_(output.str());
            //return;
        }
        // timestamp the read
        if (time_handler_)
            read_timestamp_ = time_handler_();
        else
            read_timestamp_ = 0;

        //std::cout << read_timestamp_ <<  "  bytes: " << len << std::endl;
        // add data to the buffer to be parsed
        BufferIncomingData(buffer, len);
    }
}

void Novatel::ReadFromFile(unsigned char* buffer, unsigned int length)
{
    BufferIncomingData(buffer, length);
}

void Novatel::BufferIncomingData(unsigned char* message, unsigned int length)
{
    // add incoming data to buffer
    if(buffer_index_ + length >= MAX_NOUT_SIZE)
    {
        memset(data_buffer_, 0, sizeof(data_buffer_));
        log_warning_("Overflowed receive buffer. Buffer cleared.");
        buffer_index_ = 0;
    }
    if(enableRaw && raw_msg_callback_)
    {
        // new data callback
        raw_msg_callback_(message, length);
    }
    memcpy(data_buffer_ + buffer_index_, message, length);
    buffer_index_ += length;
    for(auto i = 0;i < buffer_index_;++i)
    {
        if(CheckBinaryFormat(data_buffer_ + i, buffer_index_ - i))
        {
            ParseBinary(data_buffer_ + i, buffer_index_ - i);
            memset(data_buffer_, 0, sizeof(data_buffer_));
            buffer_index_ = 0;
            return;
        }
        if (CheckAsciiFormat(message + i, buffer_index_ - i))
        {
            ParseAscii(data_buffer_ + i, buffer_index_ - i);
            memset(data_buffer_, 0, sizeof(data_buffer_));
            buffer_index_ = 0;
            return;
        }
        if (CheckRtcmFormat(data_buffer_ + i, buffer_index_ - i))
        {
            ParseRtcm(data_buffer_ + i, buffer_index_ - i);
            memset(data_buffer_, 0, sizeof(data_buffer_));
            buffer_index_ = 0;
            return;
        }
        if(CheckACK(message + i, buffer_index_ - i))
        {
            memset(data_buffer_, 0, sizeof(data_buffer_));
            buffer_index_ = 0;
            boost::lock_guard<boost::mutex> lock(ack_mutex_);
            ack_received_ = true;
            ack_condition_.notify_all();

            // ACK received
            handle_acknowledgement_();
            return;
        }
        if(CheckReset(message + i,buffer_index_ - i))
        {
            boost::lock_guard<boost::mutex> lock(reset_mutex_);
            waiting_for_reset_complete_ = false;
            reset_condition_.notify_all();
            return;
        }
    }
}

bool Novatel::CheckBinaryFormat(unsigned char* msg, unsigned length)
{
    if (length < sizeof(BinaryHeader))
        return false;
    if(msg[0] != SYNC_BYTE_1 || msg[1] != SYNC_BYTE_2 || msg[2] != SYNC_BYTE_3)
        return false;
    BinaryHeader header;
    memcpy(&header, msg, sizeof(header));
    if (length < header.message_length + sizeof(BinaryHeader))
        return false;
    length = sizeof(BinaryHeader) + header.message_length;
    int32_t crcRes = crc32c::Crc32c(msg, length), crc;
    memcpy(&crc, msg + length, 4);
    return crcRes == crc;
}

bool Novatel::CheckAsciiFormat(unsigned char* msg, unsigned length)
{
    if (msg[0] != '#')
        return false;
    std::vector<std::string> data;
    Tokenize(string((char*)msg), data, "*");
    if (data.size() < 2)
        return false;
    int32_t crcRes = crc32c::Crc32c(data[0]), crc;
    sscanf(data[1].c_str(), "%x", &crc);
    return crcRes == crc;
}

bool Novatel::CheckRtcmFormat(unsigned char* msg, unsigned length)
{
    if (length < 6)
        return false;
    //check Preamble
    if (msg[0] != RTCM_SYNC_BYTE_1 || msg[1] & RTCM_SYNC_BYTE_1 != 0)
        return false;
    RTCM3Header header;
    memcpy(&header, msg, sizeof header);
    if (length < sizeof(header) + header.getLength() + 3)
        return false;
    length = sizeof(header) + header.getLength();
    auto crcRes = CalculateBlockCRC24Q(length, msg);
    auto crcValue = uint32_t(msg[length]) << 16 | uint32_t(msg[length + 1]) << 8 | msg[length + 2];
    return crcRes == crcValue;
}


bool Novatel::CheckAbbreviatedFormat(unsigned char* msg, unsigned length)
{
    return msg[0] == NOVATEL_ACK_BYTE_1;
}

bool Novatel::CheckACK(unsigned char* msg, unsigned length)
{
    if (length < 3)
        return false;
    return msg[0] == NOVATEL_ACK_BYTE_1 && msg[1] == NOVATEL_ACK_BYTE_2 && msg[2] == NOVATEL_ACK_BYTE_3;
}

bool Novatel::CheckReset(unsigned char* msg, unsigned length)
{
    if (length < 6)
        return false;
    return msg[0] == NOVATEL_RESET_BYTE_1 && msg[1] == NOVATEL_RESET_BYTE_2 && msg[2] == NOVATEL_RESET_BYTE_3
        && msg[3] == NOVATEL_RESET_BYTE_4 && msg[5] == NOVATEL_RESET_BYTE_6;
}

void Novatel::ParseBinary(unsigned char* message, size_t length, BINARY_LOG_TYPE message_id)
{
    //stringstream output;
    //output << "Parsing Log: " << message_id << endl;
    //log_debug_(output.str());
    uint16_t payload_length;
    uint16_t header_length;

    // obtain the received crc
    switch (message_id)
    {
    case BESTPOS_LOG_TYPE:
    {
        BinaryMessagePtr data(new Position);
        memcpy(data.get(), message, sizeof(Position));
        if (binary_callback_map_[BESTPOS_LOG_TYPE])
            binary_callback_map_[BESTPOS_LOG_TYPE](data, read_timestamp_);
        break;
    }
    case BESTUTM_LOG_TYPE:
    {
        BinaryMessagePtr data(new UtmPosition);
        memcpy(data.get(), message, sizeof(UtmPosition));
        if (binary_callback_map_[BESTUTM_LOG_TYPE])
            binary_callback_map_[BESTUTM_LOG_TYPE](data, read_timestamp_);
        break;
    }
    case BESTVEL_LOG_TYPE:
    {
        BinaryMessagePtr data(new Velocity);
        memcpy(data.get(), message, sizeof(Velocity));
        if (binary_callback_map_[BESTVEL_LOG_TYPE])
            binary_callback_map_[BESTVEL_LOG_TYPE](data, read_timestamp_);
        break;
    }
    case BESTXYZ_LOG_TYPE:
    {
        BinaryMessagePtr data(new PositionEcef);
        memcpy(data.get(), message, sizeof(PositionEcef));
        if (binary_callback_map_[BESTXYZ_LOG_TYPE])
            binary_callback_map_[BESTXYZ_LOG_TYPE](data, read_timestamp_);
        break;
    }
    case PSRDOP_LOG_TYPE:
    {
        BinaryMessagePtr data(new Dop);
        auto ptr = (Dop*)data.get();
        header_length = (uint16_t)*(message + 3);
        payload_length = (((uint16_t)*(message + 9)) << 8) +
            ((uint16_t)*(message + 8));

        // Copy header and unrepeated fields
        memcpy(ptr, message, header_length + 24);
        //Copy repeated fields
        memcpy(ptr->prn, message + header_length + 28, (4 * ptr->number_of_prns));
        //Copy CRC
        memcpy(ptr->crc, message + header_length + payload_length, 4);
        if (binary_callback_map_[PSRDOP_LOG_TYPE])
            binary_callback_map_[PSRDOP_LOG_TYPE](data, read_timestamp_);
        break;
    }
    case RTKDOP_LOG_TYPE:
    {
        BinaryMessagePtr data(new Dop);
        memcpy(data.get(), message, sizeof(Dop));
        if (binary_callback_map_[RTKDOP_LOG_TYPE])
            binary_callback_map_[RTKDOP_LOG_TYPE](data, read_timestamp_);
        break;
    }
    case BSLNXYZ_LOG_TYPE:
    {
        BinaryMessagePtr data(new BaselineEcef);
        memcpy(data.get(), message, sizeof(BaselineEcef));
        if (binary_callback_map_[BSLNXYZ_LOG_TYPE])
            binary_callback_map_[BSLNXYZ_LOG_TYPE](data, read_timestamp_);
        break;
    }
    case IONUTC_LOG_TYPE:
    {
        BinaryMessagePtr data(new IonosphericModel);
        memcpy(data.get(), message, sizeof(IonosphericModel));
        if (binary_callback_map_[IONUTC_LOG_TYPE])
            binary_callback_map_[IONUTC_LOG_TYPE](data, read_timestamp_);
        break;
    }
    case RANGE_LOG_TYPE:
    {
        BinaryMessagePtr data(new RangeMeasurements);
        auto ptr = (RangeMeasurements*)data.get();
        header_length = (uint16_t)*(message + 3);
        payload_length = (((uint16_t)*(message + 9)) << 8) +
            ((uint16_t)*(message + 8));

        // Copy header and #observations following
        memcpy(ptr, message, header_length + 4);

        //Copy repeated fields
        memcpy(ptr->range_data,
            message + header_length + 4,
            (44 * ptr->number_of_observations));

        //Copy CRC
        memcpy(&ptr->crc,
            message + header_length + payload_length,
            4);
        if (binary_callback_map_[RANGE_LOG_TYPE])
            binary_callback_map_[RANGE_LOG_TYPE](data, read_timestamp_);
        break;
    }
    case RANGECMP_LOG_TYPE:
    {
        BinaryMessagePtr data(new CompressedRangeMeasurements);
        header_length = (uint16_t)*(message + 3);
        payload_length = (((uint16_t)*(message + 9)) << 8) +
            ((uint16_t)*(message + 8));

        auto ptr = (CompressedRangeMeasurements*)data.get();

        //Copy header and unrepeated message block
        memcpy(ptr, message, header_length);
        memcpy(&ptr->number_of_observations,
            message + header_length,
            4);

        // Copy Repeated portion of message block)
        memcpy(&ptr->range_data,
            message + header_length + 4,
            (24 * ptr->number_of_observations));

        // Copy the CRC
        memcpy(&ptr->crc,
            message + header_length + payload_length,
            4);
        if (binary_callback_map_[RANGECMP_LOG_TYPE])
            binary_callback_map_[RANGECMP_LOG_TYPE](data, read_timestamp_);
        break;
    }
    case GPSEPHEM_LOG_TYPE:
    {
        BinaryMessagePtr data(new GpsEphemeris);
        memcpy(data.get(), message, sizeof(GpsEphemeris));
        if (binary_callback_map_[GPSEPHEM_LOG_TYPE])
            binary_callback_map_[GPSEPHEM_LOG_TYPE](data, read_timestamp_);
        break;
    }
    case RAWEPHEM_LOG_TYPE:
    {
        BinaryMessagePtr data(new RawEphemeris);
        memcpy(data.get(), message, sizeof(RawEphemeris));
        if (binary_callback_map_[RAWEPHEM_LOG_TYPE])
            binary_callback_map_[RAWEPHEM_LOG_TYPE](data, read_timestamp_);
        break;
    }
    case RAWALM_LOG_TYPE:
    {
        BinaryMessagePtr data(new RawAlmanac);
        memcpy(data.get(), message, sizeof(RawEphemeris));
        auto ptr = (RawAlmanac*)data.get();
        header_length = (uint16_t)*(message + 3);
        payload_length = (((uint16_t)*(message + 9)) << 8) + ((uint16_t)*(message + 8));

        //Copy header and unrepeated message block
        memcpy(ptr, message, header_length + 12);
        // Copy Repeated portion of message block)
        memcpy(&ptr->subframe_data, message + header_length + 12, (32 * ptr->num_of_subframes));
        // Copy the CRC
        memcpy(&ptr->crc, message + header_length + payload_length, 4);
        if (binary_callback_map_[RAWEPHEM_LOG_TYPE])
            binary_callback_map_[RAWEPHEM_LOG_TYPE](data, read_timestamp_);
        break;
    }
    case ALMANAC_LOG_TYPE:
    {
        BinaryMessagePtr data(new Almanac);
        auto ptr = (Almanac*)data.get();
        header_length = (uint16_t)*(message + 3);
        payload_length = (((uint16_t)*(message + 9)) << 8) + ((uint16_t)*(message + 8));

        //Copy header and unrepeated message block
        memcpy(ptr, message, header_length + 4);
        // Copy Repeated portion of message block)
        memcpy(&ptr->data, message + header_length + 4, (112 * ptr->number_of_prns));
        // Copy the CRC
        memcpy(&ptr->crc, message + header_length + payload_length, 4);
        if (binary_callback_map_[ALMANAC_LOG_TYPE])
            binary_callback_map_[ALMANAC_LOG_TYPE](data, read_timestamp_);
        break;
    }
    case SATXYZ2_LOG_TYPE:
    {
        BinaryMessagePtr data(new SatellitePositions);
        auto ptr = (SatellitePositions*)data.get();
        header_length = (uint16_t)*(message + 3);
        payload_length = (((uint16_t)*(message + 9)) << 8) + ((uint16_t)*(message + 8));

        memcpy(ptr, message, header_length + 12);
        //Copy repeated fields
        memcpy(ptr->data, message + header_length + 12, (68 * ptr->number_of_satellites));
        //Copy CRC
        memcpy(&ptr->crc, message + header_length + payload_length, 4);
        if (binary_callback_map_[SATXYZ2_LOG_TYPE])
            binary_callback_map_[SATXYZ2_LOG_TYPE](data, read_timestamp_);
        break;
    }
    case SATVIS_LOG_TYPE:
    {
        BinaryMessagePtr data(new SatelliteVisibility);
        auto ptr = (SatelliteVisibility*)data.get();
        header_length = (uint16_t)*(message + 3);
        payload_length = (((uint16_t)*(message + 9)) << 8) + ((uint16_t)*(message + 8));

        memcpy(ptr, message, header_length + 12);
        //Copy repeated fields
        memcpy(&ptr->data, message + header_length + 12, (40 * ptr->number_of_satellites));
        //Copy CRC
        memcpy(&ptr->crc, message + header_length + payload_length, 4);
        if (binary_callback_map_[SATVIS_LOG_TYPE])
            binary_callback_map_[SATVIS_LOG_TYPE](data, read_timestamp_);
        break;
    }
    case TIME_LOG_TYPE:
    {
        BinaryMessagePtr data(new TimeOffset);
        memcpy(data.get(), message, sizeof(TimeOffset));
        if (binary_callback_map_[TIME_LOG_TYPE])
            binary_callback_map_[TIME_LOG_TYPE](data, read_timestamp_);
        break;
    }
    case TRACKSTAT_LOG_TYPE:
    {
        BinaryMessagePtr data(new TrackStatus);
        auto ptr = (TrackStatus*)data.get();
        header_length = (uint16_t)*(message + 3);
        payload_length = (((uint16_t)*(message + 9)) << 8) + ((uint16_t)*(message + 8));

        memcpy(ptr, message, header_length + 12);
        //Copy repeated fields
        memcpy(&ptr->data, message + header_length + 12, (40 * ptr->number_of_channels));
        //Copy CRC
        memcpy(&ptr->crc, message + header_length + payload_length, 4);
        if (binary_callback_map_[TRACKSTAT_LOG_TYPE])
            binary_callback_map_[TRACKSTAT_LOG_TYPE](data, read_timestamp_);
        break;
    }
    case PSRPOS_LOG_TYPE:
    {
        BinaryMessagePtr data(new Position);
        memcpy(data.get(), message, sizeof(Position));
        if (binary_callback_map_[PSRPOS_LOG_TYPE])
            binary_callback_map_[PSRPOS_LOG_TYPE](data, read_timestamp_);
        break;
    }
    case RTKPOS_LOG_TYPE:
    {
        BinaryMessagePtr data(new Position);
        memcpy(data.get(), message, sizeof(Position));
        if (binary_callback_map_[RTKPOS_LOG_TYPE])
            binary_callback_map_[RTKPOS_LOG_TYPE](data, read_timestamp_);
        break;
    }
    default:
    {
        BinaryMessagePtr data(new GeneralData);
        auto ptr = (GeneralData*)data.get();
        header_length = (uint16_t)*(message + 3);
        payload_length = (((uint16_t)*(message + 9)) << 8) + ((uint16_t)*(message + 8));

        memcpy(ptr, message, header_length + 12);
        //Copy repeated fields
        ptr->data.reserve(payload_length);
        memcpy(ptr->data.data(), message + header_length + 12, payload_length);
        //Copy CRC
        memcpy(&ptr->crc, message + header_length + payload_length, 4);
        if (defaultBinaryCallback)
            defaultBinaryCallback(data, read_timestamp_);
        break;
    }
    }
}

void Novatel::ParseBinary(unsigned char* message, size_t length)
{
    //assume that message has been checked
    BinaryHeader header;
    int32_t id;
    memcpy(&header, message, sizeof(header));
    memcpy(&id, message + sizeof(header), sizeof(id));
    ParseBinary(message, length - sizeof(header) - sizeof(id), header.message_id);
}

void Novatel::ParseRtcm(unsigned char* message, size_t length)
{
    if(defaultRtcmCallback)
    {
        defaultRtcmCallback(message, length);
    }
}

void Novatel::ParseAscii(unsigned char* message, size_t length)
{
    if (defaultAsciiCallback)
        defaultAsciiCallback(std::string(message,message + length));
}

void Novatel::UnpackCompressedRangeData(const CompressedRangeData& cmp,
                                        RangeData& rng)
{
    rng.satellite_prn = cmp.range_record.satellite_prn;

    rng.channel_status = cmp.channel_status;

    rng.pseudorange = double(cmp.range_record.pseudorange) / 128.0;

    rng.pseudorange_standard_deviation =
        UnpackCompressedPsrStd(cmp.range_record.pseudorange_standard_deviation);

    rng.accumulated_doppler =
        UnpackCompressedAccumulatedDoppler(cmp, rng.pseudorange);

    rng.accumulated_doppler_std_deviation =
        (cmp.range_record.accumulated_doppler_std_deviation + 1.0) / 512.0;

    rng.doppler = cmp.range_record.doppler / 256.0;

    rng.locktime = cmp.range_record.locktime / 32.0;

    rng.carrier_to_noise = (float)(cmp.range_record.carrier_to_noise + 20);
}

double Novatel::UnpackCompressedPsrStd(const uint16_t& val) const
{
    if (val > 15)
        return 0;
    return std::vector<double>({ 0.050 ,0.075 ,0.113 ,0.169 ,0.253 ,0.380 ,0.570 ,0.854,
    1.281 ,2.375 ,4.750 ,9.500 ,19.000 ,38.000 ,76.000 ,152.000 })[val];
}

double Novatel::UnpackCompressedAccumulatedDoppler(
    const CompressedRangeData& cmp,
    const double& uncmpPsr) const
{
    double scaled_adr = (double)cmp.range_record.accumulated_doppler / 256.0;

    double adr_rolls = uncmpPsr;


    switch (cmp.channel_status.satellite_sys)
    {
    case 0: // GPS

        if (cmp.channel_status.signal_type == 0) // L1
        {
            adr_rolls /= CMP_GPS_WAVELENGTH_L1;
        }
        else if ((cmp.channel_status.signal_type == 5) || // L2 P
            (cmp.channel_status.signal_type == 9) || // L2 P codeless
            (cmp.channel_status.signal_type == 17)) // L2C
        {
            adr_rolls /= CMP_GPS_WAVELENGTH_L2;
        }
        else
        {
            /*      std::cout << "Unknown GPS Frequency type!" << std::endl;
                  std::cout << "PRN: "
                            << cmp.range_record.satellite_prn
                            << "\tSatellite System: "
                            << cmp.channel_status.satellite_sys
                            << "\tSignal Type: "
                            << cmp.channel_status.signal_type
                            << std::endl;*/
        }

        break;

    case 1: // GLO
        // TODO: Need to compute actual wavelengths here, this is incorrect
        if (cmp.channel_status.signal_type == 0) // L1
        {
            adr_rolls /= CMP_GPS_WAVELENGTH_L1;
        }
        else if (cmp.channel_status.signal_type == 5) // L2 P
        {
            adr_rolls /= CMP_GPS_WAVELENGTH_L2;
        }
        else
        {
            /*      std::cout << "Unknown GLO Frequency type!" << std::endl;
                  std::cout << "PRN: "
                            << cmp.range_record.satellite_prn
                            << "\tSatellite System: "
                            << cmp.channel_status.satellite_sys
                            << "\tSignal Type: "
                            << cmp.channel_status.signal_type
                            << std::endl;*/
        }
        break;

    case 2: // WAAS
        if (cmp.channel_status.signal_type == 1) // L1
        {
            adr_rolls /= CMP_GPS_WAVELENGTH_L1;
        }
        else
        {
            /*      std::cout << "Unknown WAAS Frequency type!" << std::endl;
                  std::cout << "PRN: "
                            << cmp.range_record.satellite_prn
                            << "\tSatellite System: "
                            << cmp.channel_status.satellite_sys
                            << "\tSignal Type: "
                            << cmp.channel_status.signal_type
                            << std::endl;*/
        }
        break;

    default:
        /*    std::cout << "Unknown Satellite System type!" << std::endl;
            std::cout << "PRN: "
                      << cmp.range_record.satellite_prn
                      << "\tSatellite System: "
                      << cmp.channel_status.satellite_sys
                      << "\tSignal Type: "
                      << cmp.channel_status.signal_type
                      << std::endl;*/
        break;
    }


    adr_rolls = (adr_rolls + scaled_adr) / CMP_MAX_VALUE;

    if (adr_rolls <= 0)
    {
        adr_rolls -= 0.5;
    }
    else
    {
        adr_rolls += 0.5;
    }

    return (scaled_adr - (CMP_MAX_VALUE * (int)adr_rolls));
}

// this functions matches the conversion done by the Novatel receivers
bool Novatel::ConvertLLaUTM(double Lat, double Long, double* northing, double* easting, int* zone, bool* north)
{
    const double a = 6378137.0;
    const double ee = 0.00669437999;
    const double k0 = 0.9996;
    const double e2 = ee / (1 - ee);

    double LongTemp = (Long + 180) - int((Long + 180) / 360) * 360 - 180; // -180.00 .. 179.9;
    double LatRad = GRAD_A_RAD(Lat);
    double LongRad = GRAD_A_RAD(LongTemp);
    double LongOriginRad;

    double N, T, C, A, M;

    //Make sure the longitude is between -180.00 .. 179.9
    *zone = int((LongTemp + 180) / 6.0) + 1;
    if (Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0)
        *zone = 32;

    // Special zones for Svalbard
    if (Lat >= 72.0 && Lat < 84.0)
    {
        if (LongTemp >= 0.0 && LongTemp < 9.0)
            *zone = 31;
        else if (LongTemp >= 9.0 && LongTemp < 21.0)
            *zone = 33;
        else if (LongTemp >= 21.0 && LongTemp < 33.0)
            *zone = 35;
        else if (LongTemp >= 33.0 && LongTemp < 42.0)
            *zone = 37;
    }
    LongOriginRad = GRAD_A_RAD((*zone-1)*6 - 180 + 3);

    N = a / sqrt(1 - ee * sin(LatRad) * sin(LatRad));
    T = tan(LatRad) * tan(LatRad);
    C = e2 * cos(LatRad) * cos(LatRad);
    A = cos(LatRad) * (LongRad - LongOriginRad);
    M = a * ((1 - ee / 4 - 3 * ee * ee / 64 - 5 * ee * ee * ee / 256) * LatRad
        - (3 * ee / 8 + 3 * ee * ee / 32 + 45 * ee * ee * ee / 1024) * sin(2 * LatRad)
        + (15 * ee * ee / 256 + 45 * ee * ee * ee / 1024) * sin(4 * LatRad)
        - (35 * ee * ee * ee / 3072) * sin(6 * LatRad));

    *easting = (double)(k0 * N * (A + (1 - T + C) * A * A * A / 6
        + (5 - 18 * T + T * T + 72 * C - 58 * e2) * A * A * A * A * A / 120) + 500000.0);
    *northing = (double)(k0 * (M + N * tan(LatRad) * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24
        + (61 - 58 * T + T * T + 600 * C - 330 * e2) * A * A * A * A * A * A / 720)));

    if (Lat < 0)
    {
        *northing += 10000000; //10000000 meter offset for southern hemisphere
        *north = false;
    }
    else
        *north = true;

    return true;
}


