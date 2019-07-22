#include "novatel/novatel.h"

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
    int j;
    unsigned long ulCRC;
    ulCRC = i;
    for (j = 8; j > 0; j--)
    {
        if (ulCRC & 1)
            ulCRC = (ulCRC >> 1) ^ CRC32_POLYNOMIAL;
        else
            ulCRC >>= 1;
    }
    return ulCRC;
}


/* --------------------------------------------------------------------------
Calculates the CRC-32 of a block of data all at once
-------------------------------------------------------------------------- */
unsigned long CalculateBlockCRC32(unsigned long ulCount, /* Number of bytes in the data block */
                                  unsigned char* ucBuffer) /* Data block */
{
    unsigned long ulTemp1;
    unsigned long ulTemp2;
    unsigned long ulCRC = 0;
    while (ulCount-- != 0)
    {
        ulTemp1 = (ulCRC >> 8) & 0x00FFFFFFL;
        ulTemp2 = CRC32Value(((int)ulCRC ^ *ucBuffer++) & 0xff);
        ulCRC = ulTemp1 ^ ulTemp2;
    }
    return(ulCRC);
}

uint32_t CalculateBlockCRC24Q(unsigned long size,unsigned char *buf)
{
    const static unsigned long CRC24POLY = 0x01864CFBL;
    const static unsigned long CRC24MASK = 0x00FFFFFFL;
    const static unsigned long CRC24BIT =  0x01000000L;

    uint32_t crc = 0x00L;
    for(auto i = 0u;i < size;++i)
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
    std::cout << "BESTPOS: \nGPS Week: " << best_position.header.gpsWeek <<
        "  GPS milliseconds: " << best_position.header.gpsMillisecs << std::endl <<
        "  Latitude: " << best_position.latitude << std::endl <<
        "  Longitude: " << best_position.longitude << std::endl <<
        "  Height: " << best_position.height << std::endl << std::endl <<
        "  Solution status: " << best_position.solutionStatus << std::endl <<
        "  position type: " << best_position.positionType << std::endl <<
        "  number of svs tracked: " << (double)best_position.numberOfSatellites << std::endl <<
        "  number of svs used: " << (double)best_position.numberOfSatellitesInSolution << std::endl;
}

inline void DefaultRawEphemCallback(RawEphemeris ephemeris, double time_stamp)
{
    std::cout << "Got RAWEPHEM for PRN " << ephemeris.prn << std::endl;
}

Novatel::Novatel()
{
    serialPort = NULL;
    readingStatus = false;
    timeHandler = DefaultGetTime;
    handleAcknowledgement = DefaultAcknowledgementHandler;
    binaryCallbackMap[BESTPOS_LOG_TYPE] = DefaultBestPositionCallback;
    logDebug = DefaultDebugMsgCallback;
    logInfo = DefaultInfoMsgCallback;
    logWarning = DefaultWarningMsgCallback;
    logError = DefaultErrorMsgCallback;
    bufferIndex = 0;
    readTimestamp = 0;
    parseTimestamp = 0;
    ackReceived = false;
    waitingForResetComplete = false;
    isConnectedB = false;
}

Novatel::~Novatel()
{
    disconnect();
}

bool Novatel::connect(const std::string& port, int baudrate, bool search, bool check)
{
    bool connected = connect_(port, baudrate, check);

    if (!connected && search)
    {
        // search additional baud rates

        int bauds_to_search[9] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400};
        bool found = false;
        for (int ii = 0; ii < 9; ii++)
        {
            std::stringstream search_msg;
            search_msg << "Searching for receiver with baudrate: " << bauds_to_search[ii];
            logInfo(search_msg.str());
            if (connect_(port, bauds_to_search[ii], check))
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
            logInfo(baud_msg.str());
            try
            {
                serialPort->write(cmd.str());
            }
            catch (std::exception& e)
            {
                std::stringstream output;
                output << "Error changing baud rate: " << e.what();
                logError(output.str());
                return false;
            }
            disconnect();
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
            connected = connect_(port, baudrate);
        }
    }

    if (connected)
    {
        // start reading
        startReading();
        isConnectedB = true;
        return true;
    }
    else
    {
        logError("Failed to connect.");
        return false;
    }
}

void Novatel::setTimeOut(uint32_t ms)
{
    auto t = serial::Timeout::simpleTimeout(ms);
    this->serialPort->setTimeout(t);
}

void Novatel::setEnableRawOutput(bool en)
{
    enableRaw = en;
}

bool Novatel::connect_(const std::string& port, int baudrate = 115200, bool doPing)
{
    try
    {
        //serial::Timeout my_timeout(50, 200, 0, 200, 0); // 115200 working settings
        //serial_port_ = new serial::Serial(port,baudrate,my_timeout);
        if (!serialPort)
        {
            //default timeout 200ms
            serialPort = new serial::Serial(port, baudrate, serial::Timeout::simpleTimeout(500));
        }

        if (!serialPort->isOpen())
        {
            std::stringstream output;
            output << "Serial port: " << port << " failed to open." << std::endl;
            logError(output.str());
            delete serialPort;
            serialPort = NULL;
            return false;
        }
        else
        {
            std::stringstream output;
            output << "Serial port: " << port << " opened successfully." << std::endl;
            logInfo(output.str());
        }

        // stop any incoming data and flush buffers
        //serial_port_->write("UNLOGALL\r\n");
        // wait for data to stop cominig in
        //boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        // clear serial port buffers
        serialPort->flush();

        // look for GPS by sending ping and waiting for response
        
        if (doPing && !ping())
        {
            std::stringstream output;
            output << "Novatel GPS not found on port: " << port << " at baudrate " << baudrate << std::endl;
            logError(output.str());
            delete serialPort;
            serialPort = NULL;
            isConnectedB = false;
            return false;
        }
        
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error connecting to gps on com port " << port << ": " << e.what();
        logError(output.str());
        isConnectedB = false;
        return false;
    }

    return true;
}


void Novatel::disconnect()
{
    logInfo("Novatel disconnecting.");
    stopReading();
    // sleep longer than the timeout period
    boost::this_thread::sleep(boost::posix_time::milliseconds(150));

    try
    {
        if ((serialPort != NULL) && (serialPort->isOpen()))
        {
            logInfo("Sending UNLOGALL and closing port.");
            serialPort->write("UNLOGALL\r\n");
            serialPort->close();
            delete serialPort;
            serialPort = NULL;
        }
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error during disconnect: " << e.what();
        logError(output.str());
    }
}

bool Novatel::isConnected()
{
    return isConnectedB;
}

void Novatel::setTimeHandler(const GetTimeCallback& timeHandler)
{
    this->timeHandler = timeHandler;
}

void Novatel::setLogDebugCallback(const LogMsgCallback& debugCallback)
{
    logDebug = debugCallback;
}

void Novatel::setLogInfoCallback(const LogMsgCallback& infoCallback)
{
    logInfo = infoCallback;
}

void Novatel::setLogWarningCallback(const LogMsgCallback& warningCallback)
{
    logWarning = warningCallback;
}

void Novatel::setLogErrorCallback(const LogMsgCallback& errorCallback)
{
    logError = errorCallback;
}

bool Novatel::ping(int numAttempts)
{
    while ((numAttempts--) > 0)
    {
        std::stringstream output;
        output << "Searching for Novatel receiver..." << std::endl;
        logInfo(output.str());
        if (updateVersion())
        {
            std::stringstream output;
            output << "Found Novatel receiver." << std::endl;
            output << "\tModel: " << model << std::endl;
            output << "\tSerial Number: " << serialNumber << std::endl;
            output << "\tHardware version: " << hardwareVersion << std::endl;
            output << "\tSoftware version: " << softwareVersion << std::endl << std::endl;;
            output << "Receiver capabilities:" << std::endl;
            output << "\tL2: ";
            if (l2Capable)
                output << "+" << std::endl;
            else
                output << "-" << std::endl;
            output << "\tRaw measurements: ";
            if (rawCapable)
                output << "+" << std::endl;
            else
                output << "-" << std::endl;
            output << "\tRTK: ";
            if (rtkCapable)
                output << "+" << std::endl;
            else
                output << "-" << std::endl;
            output << "\tSPAN: ";
            if (spanCapable)
                output << "+" << std::endl;
            else
                output << "-" << std::endl;
            output << "\tGLONASS: ";
            if (glonassCapable)
                output << "+" << std::endl;
            else
                output << "-" << std::endl;
            logInfo(output.str());
            return true;
        }
    }

    // no response found
    return false;
}

void Novatel::sendRawEphemeridesToReceiver(const RawEphemerides& rawEphemerides)
{
    try
    {
        for (uint8_t index = 0; index < MAX_NUM_SAT; index++)
        {
            cout << "SIZEOF: " << sizeof(rawEphemerides.ephemeris[index]) << endl;
            if (sizeof(rawEphemerides.ephemeris[index]) == 106 + HEADER_SIZE)
            {
                uint8_t* msg_ptr = (unsigned char*)&rawEphemerides.ephemeris[index];
                bool result = sendBinaryDataToReceiver(msg_ptr, sizeof(rawEphemerides.ephemeris[index]));
                if (result)
                    cout << "Sent RAWEPHEM for PRN " << (double)rawEphemerides.ephemeris[index].prn << endl;
            }
        }
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error in Novatel::SendRawEphemeridesToReceiver(): " << e.what();
        logError(output.str());
    }
}

bool Novatel::sendBinaryDataToReceiver(uint8_t* msgPtr, size_t length)
{
    try
    {
        stringstream output1;
        std::cout << length << std::endl;
        std::cout << "Message Pointer" << endl;
        printHex((unsigned char*)msgPtr, length);
        size_t bytes_written;

        if ((serialPort != NULL) && (serialPort->isOpen()))
        {
            bytes_written = serialPort->write(msgPtr, length);
        }
        else
        {
            logError("Unable to send message. Serial port not open.");
            return false;
        }
        // check that full message was sent to serial port
        if (bytes_written == length)
        {
            return true;
        }
        else
        {
            logError("Full message was not sent over serial port.");
            output1 << "Attempted to send " << length << "bytes. " << bytes_written << " bytes sent.";
            logError(output1.str());
            return false;
        }
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error in Novatel::SendBinaryDataToReceiver(): " << e.what();
        logError(output.str());
        return false;
    }
}

bool Novatel::sendCommand(const std::string& cmdMsg, bool waitForAck)
{
    try
    {
        // sends command to GPS receiver
        serialPort->write(cmdMsg + "\r\n");
        // wait for acknowledgement (or 2 seconds)
        if (waitForAck)
        {
            boost::mutex::scoped_lock lock(ackMutex);
            boost::system_time const timeout = boost::get_system_time() + boost::posix_time::milliseconds(2000);
            if (ackCondition.timed_wait(lock, timeout))
            {
                logInfo("Command `" + cmdMsg + "` sent to GPS receiver.");
                return true;
            }
            else
            {
                logError("Command '" + cmdMsg + "' failed.");
                return false;
            }
        }
        else
        {
            logInfo("Command `" + cmdMsg + "` sent to GPS receiver.");
            return true;
        }
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error in Novatel::SendCommand(): " << e.what();
        logError(output.str());
        return false;
    }
}

bool Novatel::setSvElevationAngleCutoff(float angle)
{
    try
    {
        std::stringstream ang_cmd;
        ang_cmd << "ECUTOFF " << angle;
        return sendCommand(ang_cmd.str());
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error in Novatel::SetSvElevationCutoff(): " << e.what();
        logError(output.str());
        return false;
    }
}

void Novatel::setPDPFilterDisable()
{
    try
    {
        std::stringstream pdp_cmd;
        pdp_cmd << "PDPFILTER DISABLE";
        bool result = sendCommand(pdp_cmd.str());
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error in Novatel::PDPFilterDisable(): " << e.what();
        logError(output.str());
    }
}

void Novatel::setPDPFilterEnable()
{
    try
    {
        std::stringstream pdp_cmd;
        pdp_cmd << "PDPFILTER ENABLE";
        bool result = sendCommand(pdp_cmd.str());
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error in Novatel::PDPFilterEnable(): " << e.what();
        logError(output.str());
    }
}

void Novatel::setPDPFilterReset()
{
    try
    {
        std::stringstream pdp_cmd;
        pdp_cmd << "PDPFILTER RESET";
        bool result = sendCommand(pdp_cmd.str());
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error in Novatel::PDPFilterReset(): " << e.what();
        logError(output.str());
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
            logError("PDPModeConfigure() input 'mode'' is not valid!");
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
            logError("PDPModeConfigure() input 'dynamics' is not valid!");
            return;
        }

        bool result = sendCommand(pdp_cmd.str());
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error in Novatel::PDPModeConfigure(): " << e.what();
        logError(output.str());
    }
}

void Novatel::setPositionTimeout(uint32_t seconds)
{
    try
    {
        if (seconds <= 86400)
        {
            std::stringstream pdp_cmd;
            pdp_cmd << "POSTIMEOUT " << seconds;
            bool result = sendCommand(pdp_cmd.str());
        }
        else
            logError("Seconds is not a valid value!");
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error in Novatel::SetPositionTimeout(): " << e.what();
        logError(output.str());
    }
}

bool Novatel::setInitialPosition(double latitude, double longitude, double height)
{
    std::stringstream pos_cmd;
    pos_cmd << "SETAPPROXPOS " << latitude << " " << longitude << " " << height;
    return sendCommand(pos_cmd.str());
}

bool Novatel::setInitialTime(uint32_t gpsWeek, double gpsSeconds)
{
    std::stringstream time_cmd;
    time_cmd << "SETAPPROXTIME " << gpsWeek << " " << gpsSeconds;
    return sendCommand(time_cmd.str());
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

bool Novatel::injectAlmanac(Almanac almanac)
{
    try
    {
        MessageType type;
        type.format = BINARY;
        type.response = ORIGINAL_MESSAGE;

        almanac.header.sync1 = NOVATEL_SYNC_BYTE_1;
        almanac.header.sync2 = NOVATEL_SYNC_BYTE_2;
        almanac.header.sync3 = NOVATEL_SYNC_BYTE_3;
        almanac.header.headerLength = HEADER_SIZE;
        almanac.header.messageId = ALMANAC_LOG_TYPE;
        almanac.header.messageType = type;
        almanac.header.portAddress = THISPORT;
        almanac.header.messageLength = 4 + almanac.numberOfPrns * 112;
        almanac.header.sequence = 0;
        almanac.header.idle = 0; //!< ignored on input
        almanac.header.timeStatus = 0; //!< ignored on input
        almanac.header.gpsWeek = 0; //!< ignored on input
        almanac.header.gpsMillisecs = 0; //!< ignored on input
        almanac.header.status = 0; //!< ignored on input
        almanac.header.reserved = 0; //!< ignored on input
        almanac.header.version = 0; //!< ignored on input

        logInfo(std::string("SIZEOF: ") + std::to_string(sizeof(almanac)));
        uint8_t* msg_ptr = (unsigned char*)&almanac;
        uint32_t crc = CalculateBlockCRC32(sizeof(almanac) - 4, msg_ptr);
        memcpy(almanac.crc, &crc, sizeof(crc)); // TODO: check byte ordering for crc
        bool result = sendBinaryDataToReceiver(msg_ptr, sizeof(almanac));
        if (result)
        {
            logInfo("Sent ALMANAC.");
            return true;
        }
        return false;
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error in Novatel::InjectAlmanac(): " << e.what();
        logError(output.str());
        return false;
    }
}

bool Novatel::setCarrierSmoothing(uint32_t l1TimeConstant, uint32_t l2TimeConstant)
{
    try
    {
        std::stringstream smooth_cmd;
        if ((2 >= l1TimeConstant) || (l1TimeConstant >= 2000))
        {
            logError("Error in SetCarrierSmoothing: l1_time_constant set to improper value.");
            return false;
        }
        else if ((5 >= l2TimeConstant) || (l2TimeConstant >= 2000))
        {
            logError("Error in SetCarrierSmoothing: l2_time_constant set to improper value.");
            return false;
        }
        else
        {
            smooth_cmd << "CSMOOTH " << l1TimeConstant << " " << l2TimeConstant;
        }
        return sendCommand(smooth_cmd.str());
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error in Novatel::SetCarrierSmoothing(): " << e.what();
        logError(output.str());
        return false;
    }
}

bool Novatel::hardwareReset()
{
    // Resets receiver to cold start, does NOT clear non-volatile memory!
    try
    {
        bool command_sent = sendCommand("RESET", false);
        if (command_sent)
        {
            boost::mutex::scoped_lock lock(resetMutex);
            waitingForResetComplete = true;
            boost::system_time const timeout = boost::get_system_time() + boost::posix_time::milliseconds(5000);
            if (resetCondition.timed_wait(lock, timeout))
            {
                logInfo("Hardware Reset Complete.");
                return true;
            }
            else
            {
                logError("Hardware Reset never Completed.");
                waitingForResetComplete = false;
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
        waitingForResetComplete = false;
        output << "Error in Novatel::HardwareReset(): " << e.what();
        logError(output.str());
        return false;
    }
}

bool Novatel::hotStartReset()
{
    try
    {
        bool command_sent = sendCommand("RESET", false);
        if (command_sent)
        {
            boost::mutex::scoped_lock lock(resetMutex);
            waitingForResetComplete = true;
            boost::system_time const timeout = boost::get_system_time() + boost::posix_time::milliseconds(10000);
            if (resetCondition.timed_wait(lock, timeout))
            {
                logInfo("HotStartReset Complete.");
                return true;
            }
            else
            {
                logError("HotStartReset never Completed.");
                waitingForResetComplete = false;
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
        waitingForResetComplete = false;
        output << "Error in Novatel::HotStartReset(): " << e.what();
        logError(output.str());
        return false;
    }
}

bool Novatel::warmStartReset()
{
    try
    {
        std::stringstream rst_pos_cmd;
        std::stringstream rst_time_cmd;
        rst_pos_cmd << "FRESET " << LAST_POSITION; //!< FRESET doesn't reply with an ACK
        bool pos_reset = sendCommand(rst_pos_cmd.str(), false);
        rst_time_cmd << "FRESET " << LBAND_TCXO_OFFSET; //!< FRESET doesn't reply with an ACK
        bool time_reset = sendCommand(rst_time_cmd.str(), false);
        if (pos_reset && time_reset)
        {
            boost::mutex::scoped_lock lock(resetMutex);
            waitingForResetComplete = true;
            boost::system_time const timeout = boost::get_system_time() + boost::posix_time::milliseconds(10000);
            if (resetCondition.timed_wait(lock, timeout))
            {
                logInfo("WarmStartReset Complete.");
                return true;
            }
            else
            {
                logError("WarmStartReset never Completed.");
                waitingForResetComplete = false;
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
        waitingForResetComplete = false;
        output << "Error in Novatel::WarmStartReset(): " << e.what();
        logError(output.str());
        return false;
    }
}

bool Novatel::coldStartReset()
{
    try
    {
        bool command_sent = sendCommand("FRESET STANDARD", false); //!< FRESET doesn't reply with an ACK
        if (command_sent)
        {
            boost::mutex::scoped_lock lock(resetMutex);
            waitingForResetComplete = true;
            boost::system_time const timeout = boost::get_system_time() + boost::posix_time::milliseconds(10000);
            if (resetCondition.timed_wait(lock, timeout))
            {
                logInfo("ColdStartReset Complete.");
                return true;
            }
            else
            {
                logError("ColdStartReset never Completed.");
                waitingForResetComplete = false;
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
        waitingForResetComplete = false;
        output << "Error in Novatel::ColdStartReset(): " << e.what();
        logError(output.str());
        return false;
    }
}

void Novatel::saveConfiguration()
{
    try
    {
        bool result = sendCommand("SAVECONFIG");
        if (result)
            logInfo("Receiver configuration has been saved.");
        else
            logError("Failed to save receiver configuration!");
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error in Novatel::SaveConfiguration(): " << e.what();
        logError(output.str());
    }
}

void Novatel::configureLogs(const std::string& logString)
{
    // parse log_string on semicolons (;)
    std::vector<std::string> logs;

    Tokenize(logString, logs, ";");

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
                serialPort->write("LOG " + *it + "\r\n");
                std::stringstream cmd;
                cmd << "LOG " << *it << "\r\n";
                logInfo(cmd.str());
                // wait for acknowledgement (or 2 seconds)
                boost::mutex::scoped_lock lock(ackMutex);
                boost::system_time const timeout = boost::get_system_time() + boost::posix_time::milliseconds(2000);
                if (ackCondition.timed_wait(lock, timeout))
                {
                    logInfo("Ack received for requested log: " + *it);
                    break;
                }
                else
                {
                    logError("No acknowledgement received for log: " + *it);
                }
                ii++;
            }
            catch (std::exception& e)
            {
                std::stringstream output;
                output << "Error configuring receiver logs: " << e.what();
                logError(output.str());
            }
        }
    }
}

void Novatel::unlog(const std::string& log)
{
    try
    {
        bool result = sendCommand("UNLOG");
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error in Novatel::Unlog(): " << e.what();
        logError(output.str());
    }
}

void Novatel::unlogAll()
{
    try
    {
        bool result = sendCommand("UNLOGALL THISPORT");
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error in Novatel::UnlogAll(): " << e.what();
        logError(output.str());
    }
}

void Novatel::configureInterfaceMode(const std::string& comPort,
    const std::string& rxMode, const std::string& txNode)
{
    try
    {
        // send command to set interface mode on com port
        // ex: INTERFACEMODE COM2 RX_MODE TX_MODE
        serialPort->write("INTERFACEMODE " + comPort + " " + rxMode + " " + txNode + "\r\n");
        // wait for acknowledgement (or 2 seconds)
        boost::mutex::scoped_lock lock(ackMutex);
        boost::system_time const timeout = boost::get_system_time() + boost::posix_time::milliseconds(2000);
        if (ackCondition.timed_wait(lock, timeout))
        {
            logInfo("Ack received.  Interface mode for port " +
                comPort + " set to: " + rxMode + " " + txNode);
        }
        else
        {
            logError("No acknowledgement received for interface mode command.");
        }
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error configuring interface mode: " << e.what();
        logError(output.str());
    }
}

void Novatel::configureBaudRate(const std::string& comPort, int baudrate)
{
    try
    {
        // send command to set baud rate on GPS com port
        // ex: COM com1 9600 n 8 1 n off on
        serialPort->write("COM " + comPort + " " + std::to_string(baudrate) + " n 8 1 n off on\r\n");
        // wait for acknowledgement (or 2 seconds)
        boost::mutex::scoped_lock lock(ackMutex);
        boost::system_time const timeout = boost::get_system_time() + boost::posix_time::milliseconds(2000);
        if (ackCondition.timed_wait(lock, timeout))
        {
            std::stringstream log_out;
            log_out << "Ack received.  Baud rate on com port " <<
                comPort << " set to " << baudrate << std::endl;
            logInfo(log_out.str());
        }
        else
        {
            logError("No acknowledgement received for com configure command.");
        }
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error configuring baud rate: " << e.what();
        logError(output.str());
    }
}

bool Novatel::updateVersion()
{
    // request the receiver version and wait for a response
    // example response:
    //#VERSIONA,COM1,0,71.5,FINESTEERING,1362,340308.478,00000008,3681,2291;
    //    1,GPSCARD,"L12RV","DZZ06040010","OEMV2G-2.00-2T","3.000A19","3.000A9",
    //    "2006/Feb/ 9","17:14:33"*5e8df6e0

    try
    {
        // clear port
        serialPort->flush();
        // read out any data currently in the buffer
        std::string read_data = serialPort->read(5000);
        while (read_data.length())
            read_data = serialPort->read(5000);

        // send request for version
        serialPort->write("log versiona once\r\n");
        // wait for response from the receiver
        //boost::this_thread::sleep(boost::posix_time::milliseconds(200));
        // read from the serial port until a new line character is seen
        std::string gps_response = serialPort->read(15000);

        std::vector<std::string> packets;

        Tokenize(gps_response, packets, "\n");

        // loop through all packets in file and check for version messages
        // stop when the first is found or all packets are read
        for (size_t ii = 0; ii < packets.size(); ii++)
        {
            if (parseVersion(packets[ii]))
            {
                return true;
            }
        }
    }
    catch (std::exception& e)
    {
        std::stringstream output;
        output << "Error reading version info from receiver: " << e.what();
        logError(output.str());
        return false;
    }


    return false;
}

bool Novatel::parseVersion(std::string packet)
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
        logError("Error parsing received version."
            " End of message was not found");
        logDebug(packet);
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
        logError("Error parsing received version. "
            "Incorrect number of tokens found.");
        std::stringstream err_out;
        err_out << "Found: " << token_count << "  Expected: " << (8 * number_components + 1);
        logError(err_out.str());
        logDebug(packet);
        return false;
    }

    current_token = tokens.begin();
    // device type is 2nd token
    string device_type = *(++current_token);
    // model is 3rd token
    model = *(++current_token);
    // serial number is 4th token
    serialNumber = *(++current_token);
    // model is 5rd token
    hardwareVersion = *(++current_token);
    // model is 6rd token
    softwareVersion = *(++current_token);

    // parse the version:
    if (hardwareVersion.length() > 3)
        protocolVersion = hardwareVersion.substr(1, 4);
    else
        protocolVersion = "UNKNOWN";

    // parse model number:
    // is the receiver capable of raw measurements?
    if (model.find("L") != string::npos)
        rawCapable = true;
    else
        rawCapable = false;

    // can the receiver receive L2?
    if (model.find("12") != string::npos)
        l2Capable = true;
    else
        l2Capable = false;

    // can the receiver receive GLONASS?
    if (model.find("G") != string::npos)
        glonassCapable = true;
    else
        glonassCapable = false;

    // Is this a SPAN unit?
    if ((model.find("I") != string::npos) || (model.find("J") != string::npos))
        spanCapable = true;
    else
        spanCapable = false;

    // Can the receiver process RTK?
    if (model.find("R") != string::npos)
        rtkCapable = true;
    else
        rtkCapable = false;


    // fix for oem4 span receivers - do not use l12 notation
    // i think all oem4 spans are l1 l2 capable and raw capable
    if ((protocolVersion == "OEM4") && (spanCapable))
    {
        l2Capable = true;
        rawCapable = true;
    }

    return true;
}

void Novatel::startReading()
{
    if (readingStatus)
        return;
    // create thread to read from sensor
    readingStatus = true;
    readThreadPtr = boost::shared_ptr<boost::thread>
        (new boost::thread(boost::bind(&Novatel::readSerialPort, this)));
}

void Novatel::stopReading()
{
    readingStatus = false;
}

void Novatel::readSerialPort()
{
    if(dataRead)
    {
        delete[] dataRead;
    }
    dataRead = new unsigned char[MAX_NOUT_SIZE];
    size_t len;
    logInfo("Started read thread.");

    // continuously read data from serial port
    while (readingStatus)
    {
        try
        {
            // read data
            len = serialPort->read(dataRead, MAX_NOUT_SIZE);
        }
        catch (std::exception& e)
        {
            std::stringstream output;
            output << "Error reading from serial port: " << e.what();
            logError(output.str());
            //return;
        }
        // timestamp the read
        if (timeHandler)
            readTimestamp = timeHandler();
        else
            readTimestamp = 0;

        //std::cout << read_timestamp_ <<  "  bytes: " << len << std::endl;
        // add data to the buffer to be parsed
        if(len)
            bufferIncomingData(dataRead, len);
    }
    delete[] dataRead;
    dataRead = nullptr;
}

void Novatel::readFromFile(unsigned char* buffer, unsigned int length)
{
    bufferIncomingData(buffer, length);
}

void Novatel::bufferIncomingData(unsigned char* message, size_t length)
{
    // add incoming data to buffer
    if(bufferIndex + length >= MAX_NOUT_SIZE)
    {
        memset(dataBuffer, 0, sizeof(dataBuffer));
        logWarning("Overflowed receive buffer. Buffer cleared.");
        bufferIndex = 0;
    }
    if(enableRaw && rawMsgCallback)
    {
        // new data callback
        rawMsgCallback(message, length);
    }
    memcpy(dataBuffer + bufferIndex, message, length);
    bufferIndex += length;
    for(auto i = 0;i < bufferIndex;++i)
    {
        if(checkBinaryFormat(dataBuffer + i, bufferIndex - i))
        {
            parseBinary(dataBuffer + i, bufferIndex - i);
            memset(dataBuffer, 0, sizeof(dataBuffer));
            bufferIndex = 0;
            return;
        }
        if (checkAsciiFormat(message + i, bufferIndex - i))
        {
            parseAscii(dataBuffer + i, bufferIndex - i);
            memset(dataBuffer, 0, sizeof(dataBuffer));
            bufferIndex = 0;
            return;
        }
        if (checkRtcmFormat(dataBuffer + i, bufferIndex - i))
        {
            parseRtcm(dataBuffer + i, bufferIndex - i);
            memset(dataBuffer, 0, sizeof(dataBuffer));
            bufferIndex = 0;
            return;
        }
        if(checkAck(message + i, bufferIndex - i))
        {
            memset(dataBuffer, 0, sizeof(dataBuffer));
            bufferIndex = 0;
            boost::lock_guard<boost::mutex> lock(ackMutex);
            ackReceived = true;
            ackCondition.notify_all();

            // ACK received
            handleAcknowledgement();
            return;
        }
        if(checkReset(message + i,bufferIndex - i))
        {
            boost::lock_guard<boost::mutex> lock(resetMutex);
            waitingForResetComplete = false;
            resetCondition.notify_all();
            return;
        }
    }
}

bool Novatel::checkBinaryFormat(unsigned char* msg, size_t length)
{
    if (length < sizeof(BinaryHeader))
        return false;
    if(msg[0] != SYNC_BYTE_1 || msg[1] != SYNC_BYTE_2 || msg[2] != SYNC_BYTE_3)
        return false;
    BinaryHeader header;
    memcpy(&header, msg, sizeof(header));
    if (length < header.messageLength + sizeof(BinaryHeader))
        return false;
    length = sizeof(BinaryHeader) + header.messageLength;
    int32_t crcRes = CalculateBlockCRC32(length, msg), crc;
    memcpy(&crc, msg + length, 4);
    return crcRes == crc;
}

bool Novatel::checkAsciiFormat(unsigned char* msg, size_t length)
{
    if (msg[0] != '#')
        return false;
    auto starPtr = strchr((char*)msg, '*');
    if (starPtr == NULL)
        return false;
    uint32_t crcRes = CalculateBlockCRC32(starPtr - (char*)msg - 1, msg + 1), crc;
    sscanf(starPtr + 1, "%x", &crc);
    return crcRes == crc;
}

bool Novatel::checkRtcmFormat(unsigned char* msg, size_t length)
{
    if (length < 6)
        return false;
    //check Preamble
    if (msg[0] != RTCM_SYNC_BYTE_1 || (msg[1] & RTCM_SYNC_BYTE_1) != 0)
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


bool Novatel::checkAbbreviatedFormat(unsigned char* msg, size_t length)
{
    return msg[0] == NOVATEL_ACK_BYTE_1;
}

bool Novatel::checkAck(unsigned char* msg, size_t length)
{
    if (length < 3)
        return false;
    return msg[0] == NOVATEL_ACK_BYTE_1 && msg[1] == NOVATEL_ACK_BYTE_2 && msg[2] == NOVATEL_ACK_BYTE_3;
}

bool Novatel::checkReset(unsigned char* msg, size_t length)
{
    if (length < 6)
        return false;
    return msg[0] == NOVATEL_RESET_BYTE_1 && msg[1] == NOVATEL_RESET_BYTE_2 && msg[2] == NOVATEL_RESET_BYTE_3
        && msg[3] == NOVATEL_RESET_BYTE_4 && msg[5] == NOVATEL_RESET_BYTE_6;
}

void Novatel::parseBinary(unsigned char* message, size_t length, BINARY_LOG_TYPE messageId)
{
    //stringstream output;
    //output << "Parsing Log: " << message_id << endl;
    //log_debug_(output.str());
    uint16_t payload_length;
    uint16_t header_length;

    // obtain the received crc
    switch (messageId)
    {
    case BESTPOS_LOG_TYPE:
    {
        BinaryMessagePtr data(new Position);
        memcpy(data.get(), message, sizeof(Position));
        if (binaryCallbackMap[BESTPOS_LOG_TYPE])
            binaryCallbackMap[BESTPOS_LOG_TYPE](data, readTimestamp);
        break;
    }
    case HEADING_LOG_TYPE:
    {
        BinaryMessagePtr data(new Heading);
        memcpy(data.get(), message, sizeof(Heading));
        if (binaryCallbackMap[HEADING_LOG_TYPE])
            binaryCallbackMap[HEADING_LOG_TYPE](data, readTimestamp);
        break;
    }
    case BESTUTM_LOG_TYPE:
    {
        BinaryMessagePtr data(new UtmPosition);
        memcpy(data.get(), message, sizeof(UtmPosition));
        if (binaryCallbackMap[BESTUTM_LOG_TYPE])
            binaryCallbackMap[BESTUTM_LOG_TYPE](data, readTimestamp);
        break;
    }
    case BESTVEL_LOG_TYPE:
    {
        BinaryMessagePtr data(new Velocity);
        memcpy(data.get(), message, sizeof(Velocity));
        if (binaryCallbackMap[BESTVEL_LOG_TYPE])
            binaryCallbackMap[BESTVEL_LOG_TYPE](data, readTimestamp);
        break;
    }
    case BESTXYZ_LOG_TYPE:
    {
        BinaryMessagePtr data(new PositionEcef);
        memcpy(data.get(), message, sizeof(PositionEcef));
        if (binaryCallbackMap[BESTXYZ_LOG_TYPE])
            binaryCallbackMap[BESTXYZ_LOG_TYPE](data, readTimestamp);
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
        memcpy(ptr->prn, message + header_length + 28, (4 * ptr->numberOfPrns));
        //Copy CRC
        memcpy(ptr->crc, message + header_length + payload_length, 4);
        if (binaryCallbackMap[PSRDOP_LOG_TYPE])
            binaryCallbackMap[PSRDOP_LOG_TYPE](data, readTimestamp);
        break;
    }
    case RTKDOP_LOG_TYPE:
    {
        BinaryMessagePtr data(new Dop);
        memcpy(data.get(), message, sizeof(Dop));
        if (binaryCallbackMap[RTKDOP_LOG_TYPE])
            binaryCallbackMap[RTKDOP_LOG_TYPE](data, readTimestamp);
        break;
    }
    case BSLNXYZ_LOG_TYPE:
    {
        BinaryMessagePtr data(new BaselineEcef);
        memcpy(data.get(), message, sizeof(BaselineEcef));
        if (binaryCallbackMap[BSLNXYZ_LOG_TYPE])
            binaryCallbackMap[BSLNXYZ_LOG_TYPE](data, readTimestamp);
        break;
    }
    case IONUTC_LOG_TYPE:
    {
        BinaryMessagePtr data(new IonosphericModel);
        memcpy(data.get(), message, sizeof(IonosphericModel));
        if (binaryCallbackMap[IONUTC_LOG_TYPE])
            binaryCallbackMap[IONUTC_LOG_TYPE](data, readTimestamp);
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
        memcpy(ptr->rangeData,
            message + header_length + 4,
            (44 * ptr->numberOfObservations));

        //Copy CRC
        memcpy(&ptr->crc,
            message + header_length + payload_length,
            4);
        if (binaryCallbackMap[RANGE_LOG_TYPE])
            binaryCallbackMap[RANGE_LOG_TYPE](data, readTimestamp);
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
        memcpy(&ptr->numberOfObservations,
            message + header_length,
            4);

        // Copy Repeated portion of message block)
        memcpy(&ptr->rangeData,
            message + header_length + 4,
            (24 * ptr->numberOfObservations));

        // Copy the CRC
        memcpy(&ptr->crc,
            message + header_length + payload_length,
            4);
        if (binaryCallbackMap[RANGECMP_LOG_TYPE])
            binaryCallbackMap[RANGECMP_LOG_TYPE](data, readTimestamp);
        break;
    }
    case GPSEPHEM_LOG_TYPE:
    {
        BinaryMessagePtr data(new GpsEphemeris);
        memcpy(data.get(), message, sizeof(GpsEphemeris));
        if (binaryCallbackMap[GPSEPHEM_LOG_TYPE])
            binaryCallbackMap[GPSEPHEM_LOG_TYPE](data, readTimestamp);
        break;
    }
    case RAWEPHEM_LOG_TYPE:
    {
        BinaryMessagePtr data(new RawEphemeris);
        memcpy(data.get(), message, sizeof(RawEphemeris));
        if (binaryCallbackMap[RAWEPHEM_LOG_TYPE])
            binaryCallbackMap[RAWEPHEM_LOG_TYPE](data, readTimestamp);
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
        memcpy(&ptr->subframeData, message + header_length + 12, (32 * ptr->numOfSubframes));
        // Copy the CRC
        memcpy(&ptr->crc, message + header_length + payload_length, 4);
        if (binaryCallbackMap[RAWEPHEM_LOG_TYPE])
            binaryCallbackMap[RAWEPHEM_LOG_TYPE](data, readTimestamp);
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
        memcpy(&ptr->data, message + header_length + 4, (112 * ptr->numberOfPrns));
        // Copy the CRC
        memcpy(&ptr->crc, message + header_length + payload_length, 4);
        if (binaryCallbackMap[ALMANAC_LOG_TYPE])
            binaryCallbackMap[ALMANAC_LOG_TYPE](data, readTimestamp);
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
        memcpy(ptr->data, message + header_length + 12, (68 * ptr->numberOfSatellites));
        //Copy CRC
        memcpy(&ptr->crc, message + header_length + payload_length, 4);
        if (binaryCallbackMap[SATXYZ2_LOG_TYPE])
            binaryCallbackMap[SATXYZ2_LOG_TYPE](data, readTimestamp);
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
        memcpy(&ptr->data, message + header_length + 12, (40 * ptr->numberOfSatellites));
        //Copy CRC
        memcpy(&ptr->crc, message + header_length + payload_length, 4);
        if (binaryCallbackMap[SATVIS_LOG_TYPE])
            binaryCallbackMap[SATVIS_LOG_TYPE](data, readTimestamp);
        break;
    }
    case TIME_LOG_TYPE:
    {
        BinaryMessagePtr data(new TimeOffset);
        memcpy(data.get(), message, sizeof(TimeOffset));
        if (binaryCallbackMap[TIME_LOG_TYPE])
            binaryCallbackMap[TIME_LOG_TYPE](data, readTimestamp);
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
        memcpy(&ptr->data, message + header_length + 12, (40 * ptr->numberOfChannels));
        //Copy CRC
        memcpy(&ptr->crc, message + header_length + payload_length, 4);
        if (binaryCallbackMap[TRACKSTAT_LOG_TYPE])
            binaryCallbackMap[TRACKSTAT_LOG_TYPE](data, readTimestamp);
        break;
    }
    case PSRPOS_LOG_TYPE:
    {
        BinaryMessagePtr data(new Position);
        memcpy(data.get(), message, sizeof(Position));
        if (binaryCallbackMap[PSRPOS_LOG_TYPE])
            binaryCallbackMap[PSRPOS_LOG_TYPE](data, readTimestamp);
        break;
    }
    case RTKPOS_LOG_TYPE:
    {
        BinaryMessagePtr data(new Position);
        memcpy(data.get(), message, sizeof(Position));
        if (binaryCallbackMap[RTKPOS_LOG_TYPE])
            binaryCallbackMap[RTKPOS_LOG_TYPE](data, readTimestamp);
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
            defaultBinaryCallback(data, readTimestamp);
        break;
    }
    }
}

void Novatel::parseBinary(unsigned char* message, size_t length)
{
    //assume that message has been checked
    BinaryHeader header;
    int32_t id;
    memcpy(&header, message, sizeof(header));
    memcpy(&id, message + sizeof(header), sizeof(id));
    parseBinary(message, length - sizeof(header) - sizeof(id), header.messageId);
}

void Novatel::parseRtcm(unsigned char* message, size_t length)
{
    if(defaultRtcmCallback)
    {
        defaultRtcmCallback(message, length);
    }
}

void Novatel::parseAscii(unsigned char* message, size_t length)
{
    if (defaultAsciiCallback)
        defaultAsciiCallback(std::string(message,message + length));
}

void Novatel::unpackCompressedRangeData(const CompressedRangeData& cmp,
                                        RangeData& rng)
{
    rng.satellitePrn = cmp.rangeRecord.satellitePrn;

    rng.channel_status = cmp.channelStatus;

    rng.pseudorange = double(cmp.rangeRecord.pseudorange) / 128.0;

    rng.pseudorangeStandardDeviation =
        unpackCompressedPsrStd(cmp.rangeRecord.pseudorangeStandardDeviation);

    rng.accumulatedDoppler =
        unpackCompressedAccumulatedDoppler(cmp, rng.pseudorange);

    rng.accumulatedDopplerStdDeviation =
        (cmp.rangeRecord.accumulatedDopplerStdDeviation + 1.0) / 512.0;

    rng.doppler = cmp.rangeRecord.doppler / 256.0;

    rng.locktime = cmp.rangeRecord.locktime / 32.0;

    rng.carrierToNoise = (float)(cmp.rangeRecord.carrierToNoise + 20);
}

double Novatel::unpackCompressedPsrStd(const uint16_t& val) const
{
    if (val > 15)
        return 0;
    return std::vector<double>({ 0.050 ,0.075 ,0.113 ,0.169 ,0.253 ,0.380 ,0.570 ,0.854,
    1.281 ,2.375 ,4.750 ,9.500 ,19.000 ,38.000 ,76.000 ,152.000 })[val];
}

double Novatel::unpackCompressedAccumulatedDoppler(
    const CompressedRangeData& cmp,
    const double& uncmpPsr) const
{
    double scaled_adr = (double)cmp.rangeRecord.accumulatedDoppler / 256.0;

    double adr_rolls = uncmpPsr;


    switch (cmp.channelStatus.satelliteSys)
    {
    case 0: // GPS

        if (cmp.channelStatus.signalType == 0) // L1
        {
            adr_rolls /= CMP_GPS_WAVELENGTH_L1;
        }
        else if ((cmp.channelStatus.signalType == 5) || // L2 P
            (cmp.channelStatus.signalType == 9) || // L2 P codeless
            (cmp.channelStatus.signalType == 17)) // L2C
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
        if (cmp.channelStatus.signalType == 0) // L1
        {
            adr_rolls /= CMP_GPS_WAVELENGTH_L1;
        }
        else if (cmp.channelStatus.signalType == 5) // L2 P
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
        if (cmp.channelStatus.signalType == 1) // L1
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
bool Novatel::convertLLaUtm(double lat, double Long, double* northing, double* easting, int* zone, bool* north)
{
    const double a = 6378137.0;
    const double ee = 0.00669437999;
    const double k0 = 0.9996;
    const double e2 = ee / (1 - ee);

    double LongTemp = (Long + 180) - int((Long + 180) / 360) * 360 - 180; // -180.00 .. 179.9;
    double LatRad = GRAD_A_RAD(lat);
    double LongRad = GRAD_A_RAD(LongTemp);
    double LongOriginRad;

    double N, T, C, A, M;

    //Make sure the longitude is between -180.00 .. 179.9
    *zone = int((LongTemp + 180) / 6.0) + 1;
    if (lat >= 56.0 && lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0)
        *zone = 32;

    // Special zones for Svalbard
    if (lat >= 72.0 && lat < 84.0)
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

    if (lat < 0)
    {
        *northing += 10000000; //10000000 meter offset for southern hemisphere
        *north = false;
    }
    else
        *north = true;

    return true;
}

void Novatel::setRawMsgCallback(RawMsgCallback handler)
{
    rawMsgCallback = handler;
}

void Novatel::setCallback(BINARY_LOG_TYPE typeId, const BinaryMessageCallback& callback)
{
    binaryCallbackMap[typeId] = callback;
}

void Novatel::setDefaultBinaryCallback(const BinaryMessageCallback& callback)
{
    defaultBinaryCallback = callback;
}

void Novatel::setDefaultAsciiCallback(const LogMsgCallback& callback)
{
    defaultAsciiCallback = callback;
}

void Novatel::setDefaultRtcmCallback(const RawMsgCallback& callback)
{
    defaultRtcmCallback = callback;
}


