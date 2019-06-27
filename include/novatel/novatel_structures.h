/*!
 * \file novatel/novatel_structures.h
 * \author David Hodo <david.hodo@is4s.com>
 * Portions based on previous code by William Travis and Scott Martin
 * \version 1.1
 *
 * \section LICENSE
 *
 * The BSD License
 *
 * Copyright (c) 2011 David Hodo - Integrated Solutions for Systems
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
 * This provides structure definitions for messages output by a Novatel GPS.
 *
 */


#ifndef NOVATELSTRUCTURES_H
#define NOVATELSTRUCTURES_H

#include "novatel_enums.h"
#include <stdint.h>  // use fixed size integer types, rather than standard c++ types
#include <vector>

namespace novatel
{
#define MAX_NOUT_SIZE 8192 // Maximum size of a NovAtel log buffer (ALMANACA logs are big!)
#define EPH_CHAN 33
#define NUMSAT 14
#define MAX_CHAN	54  // Maximum number of signal channels
#define MAX_NUM_SAT 28	// Maximum number of satellites with information in the RTKDATA log
#define HEADER_SIZE 28 // Binary header size for OEM 4, V, and 6 receivers
#define SHORT_HEADER_SIZE 12 // short binary header size
#define CHECKSUM_SIZE 4  // size of the message CRC


#define SYNC_BYTE_1 0xAA
#define SYNC_BYTE_2 0x44
#define SYNC_BYTE_3 0x12

#define SYNC_1_IDX 0 	// first sync byte location
#define SYNC_2_IDX 1 	// second sync byte location
#define SYNC_3_IDX 2 	// third sync byte location
#define HEADER_LEN_IDX 3 // header length location
#define MSG_ID_END_IDX 5	// Message ID location
#define MSG_LENGTH_END_IDX 9 // message length index

#define NOVATEL_SYNC_BYTE_1 0xAA
#define NOVATEL_SYNC_BYTE_2 0x44
#define NOVATEL_SYNC_BYTE_3 0x12
#define NOVATEL_ACK_BYTE_1 '<'
#define NOVATEL_ACK_BYTE_2 'O'
#define NOVATEL_ACK_BYTE_3 'K'
#define NOVATEL_RESET_BYTE_1 0X5B
#define NOVATEL_RESET_BYTE_2 'C'
#define NOVATEL_RESET_BYTE_3 'O'
#define NOVATEL_RESET_BYTE_4 'M'
#define NOVATEL_RESET_BYTE_6 0X5D

#define RTCM_SYNC_BYTE_1    0xD3
#define RTCM_SYNC_BYTE_2    (~0x03)

// IMU Constants
// scale factor between integer counts and change in velocity in m/s for AG11 and AG58
#define VELOCITY_CHANGE_SCALE_FACTOR_11_58 (0.3048/((double)134217728.0))
// scale factor between integer counts and change in velocity in m/s for AG17 and AG62
#define VELOCITY_CHANGE_SCALE_FACTOR_17_62 (0.3048/((double)67108864.0))
// scale factor between integer counts and change in angle in rad for AG11, AG17, AG58, and AG62
#define ANGULAR_CHANGE_SCALE_FACTOR (1.0/((double)8589934592.0))

// define macro to pack structures correctly with both GCC and MSVC compilers
#ifdef _MSC_VER // using MSVC
#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#else
	#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif


//*******************************************************************************
// BASE STRUCTURES
//*******************************************************************************
/*!
* Message Type
*/
PACK(
struct NOVATEL_EXPORT MessageType {
    unsigned source:5;
    MessageFormat format:2;
    ResponseBit response:1;
});

//! Header prepended to OEM6 binary messages
PACK(
struct NOVATEL_EXPORT BinaryHeader
{
   uint8_t sync1; //!< start of packet first byte (0xAA)
   uint8_t sync2; //!< start of packet second byte (0x44)
   uint8_t sync3; //!< start of packet third  byte (0x12)
   uint8_t headerLength; //!< Length of the header in bytes ( From start of packet )
   BINARY_LOG_TYPE messageId; //!< Message ID number
   MessageType messageType; //!< Message type - binary, ascii, nmea, etc...
   uint8_t portAddress; //!< Address of the data port the log was received on
   uint16_t messageLength; //!< Message length (Not including header or CRC)
   uint16_t sequence; //!< Counts down from N-1 to 0 for multiple related logs
   uint8_t idle; //!< Time the processor was idle in last sec between logs with same ID
   uint8_t timeStatus; //!< Indicates the quality of the GPS time
   uint16_t gpsWeek; //!< GPS Week number
   uint32_t gpsMillisecs; //!< Milliseconds into week
   uint32_t status; //!< Receiver status word
   uint16_t reserved; //!< Reserved for internal use
   uint16_t version; //!< Receiver software build number (0-65535)
});

PACK(
struct NOVATEL_EXPORT CRC32{
    int8_t data[4];
});

PACK(
struct NOVATEL_EXPORT BinaryMessageBase
{
    BinaryHeader header; //!< Message header
});


//*******************************************************************************
// GENERIC GPS STRUCTURES
//*******************************************************************************

/*!
* Position Message Structure
* This log contains the a position received from the receiver
* in latitude and longitude as well as status information such as
* the solution type and the number of satellites used.
*
* This structure represents the format of the following messages:
*  - BESTPOS
*  - RTKPOS
*  - PSRPOS
*/
PACK(
struct NOVATEL_EXPORT Position : BinaryMessageBase
{
	SolutionStatus solutionStatus; //!< Solution status
	PositionType positionType; //!< Position type
	double latitude; //!< latitude (deg)
    double longitude; //!< longitude (deg)
	double height; //!< height above mean sea level (m)
	float undulation; //!< Undulation - the relationship between the geoid and the ellipsoid (m)
	DatumID datumId; //!< datum id number
	float latitudeStandardDeviation; //!< latitude standard deviation (m)
	float longitudeStandardDeviation; //!< longitude standard deviation (m)
	float heightStandardDeviation; //!< height standard deviation (m)
	uint8_t baseStationId[4]; //!< base station id
	float differentialAge; //!< differential position age (sec)
	float solutionAge; //!< solution age (sec)
	uint8_t numberOfSatellites; //!< number of satellites tracked
	uint8_t numberOfSatellitesInSolution; //!< number of satellites used in solution
	uint8_t numGpsPlusGlonassL1; //!< number of GPS plus GLONASS L1 satellites used in solution
	uint8_t numGpsPlusGlonassL2; //!< number of GPS plus GLONASS L2 satellites used in solution
	uint8_t reserved; //!< reserved
	uint8_t extendedSolutionStatus; //!< extended solution status - OEMV and greater only
	uint8_t reserved2; //!< reserved
	uint8_t signalsUsedMask; //!< signals used mask - OEMV and greater only
    uint8_t crc[4]; //!< 32-bit cyclic redundancy check (CRC)
});

/*!
* ECEF Position Message Structure
* This log contains the receiver’s best
* available position and velocity in ECEF
* coordinates. The position and velocity status
* fields indicate whether or not the corresponding
* data is valid.
*
* This structure represents the format of the following messages:
*  - BESTXYZ
*  - RTKXYZ
*  - PSRXYZ
*/
PACK(
struct NOVATEL_EXPORT PositionEcef : BinaryMessageBase
{
    SolutionStatus solutionStatus; //!< Solution status
    PositionType positionType; //!< Position type
    double xPosition; //!< x coordinate in ECEF (m)
    double yPosition; //!< x coordinate in ECEF (m)
    double zPosition; //!< x coordinate in ECEF (m)
    float xStandardDeviation; //!< Standard deviation of x coordinate (m)
    float yStandardDeviation; //!< Standard deviation of y coordinate (m)
    float zStandardDeviation; //!< Standard deviation of z coordinate (m)
    SolutionStatus velocityStatus; //!< Velocity solution status
    PositionType velocityType; //!< Velocity solution type
    double xVelocity; //Velocity in x (m/s)
    double yVelocity; //Velocity in y (m/s)
    double zVelocity; //Velocity in z (m/s)
    float xVelocityStandardDeviation; //!< Standard deviation of velcoity in x (m/s)
    float yVelocityStandardDeviation; //!< Standard deviation of velcoity in y (m/s)
    float zVelocityStandardDeviation; //!< Standard deviation of velcoity in z (m/s)
    uint8_t baseStationId[4]; //!< Base station ID
    float velocityLatency; //!< Latency in velocity time tag (s)
    float differentialAge; //!< differential position age (sec)
    float solutionAge; //!< solution age (sec)
    uint8_t numberOfSatellites; //!< number of satellites tracked
    uint8_t numberOfSatellitesInSolution; //!< number of satellites used in solution
    uint8_t reserved[3]; //!< Reserved
    uint8_t extendedSolutionStatus; //!< extended solution status - OEMV and greater only
    uint8_t reserved2; //!< reserved
    uint8_t signalsUsedMask; //!< signals used mask - OEMV and greater only
    uint8_t crc[4]; //!< 32-bit cyclic redundancy check (CRC)
});


/*!
* Velocity Message Structure
* This log contains the best available velocity
* information computed by the receiver. In addition,
* it reports a velocity status indicator, which is
* useful in indicating whether or not the corresponding
* data is valid. The velocity measurements sometimes
* have a latency associated with them. The time of validity
* is the time tag in the log minus the latency value.
*
* This structure represents the format of the following messages:
*  - BESTVEL
*  - RTKVEL
*  - PSRVEL
*/
PACK(
struct NOVATEL_EXPORT Velocity : BinaryMessageBase
{
	SolutionStatus solutionStatus; //!< Solution status
	PositionType positionType; //!< Position type
	float latency; //!< measure of the latency of the velocity time tag in seconds
	float age; //!< differential age in seconds
	double horizontalSpeed; //!< horizontal speed in m/s
	double trackOverGround; //!< direction of travel in degrees
	double verticalSpeed; //!< vertical speed in m/s
	float reserved;
    uint8_t crc[4]; //!< 32-bit cyclic redundancy check (CRC)
});


/*!
* DOP Message Structure
* The dilution of precision data is calculated
* using the geometry of only those satellites that are
* currently being tracked and used in the position
* solution by the receiver. This log is updated once
* every 60 seconds or whenever a change in the satellite
* constellation occurs. Therefore, the total number of
* data fields output by the log is variable and depends
* on the number of SVs that are being tracked.
*
* This structure represents the format of the following messages:
*  - PSRDOP
*  - RTKDOP
*/
PACK(
struct NOVATEL_EXPORT Dop : BinaryMessageBase
{
    float geometricDop; //!< Geometric DOP
    float positionDop; //!< Position DOP
    float horizontalDop; //!< Horizontal DOP
    float horizontalPositionTimeDop; //!< Horizontal position and time DOP
    float timeDop; //!< Time DOP
    float elevationCutoffAngle; //!< Elevation cutoff angle
    int32_t numberOfPrns; //!< Number of PRNs to follow
    uint32_t prn[MAX_CHAN]; //!< PRNof each satellite used
    uint8_t crc[4]; //!< 32-bit cyclic redundancy check (CRC)
});

//*******************************************************************************
// MESSAGE SPECIFIC GPS STRUCTURES
//*******************************************************************************

/*!
* BSLNXYZ Message Structure
* This log contains the receiver’s RTK baseline
* in ECEF coordinates. The position status field
* indicates whether or not the corresponding data
* is valid.
*/
PACK(
struct NOVATEL_EXPORT BaselineEcef : BinaryMessageBase
{
    SolutionStatus solutionStatus; //!< Solution status
    PositionType positionType; //!< Position type
    double xBaseline; //!< Baseline x coordinate (m)
    double yBaseline; //!< Baseline y coordinate (m)
    double zBaseline; //!< Baseline z coordinate (m)
    float xBaselineStandardDeviation; //!< Standard deviation of baseline x coordinate (m)
    float yBaselineStandardDeviation; //!< Standard deviation of baseline y coordinate (m)
    float zBaselineStandardDeviation; //!< Standard deviation of baseline z coordinate (m)
    uint8_t baseStationId[4]; //!< Base station ID
    uint8_t numberOfSatellites; //!< number of satellites tracked
    uint8_t numberOfSatellitesInSolution; //!< number of satellites used in solution
    uint8_t numGpsPlusGlonassL1; //!< number of GPS plus GLONASS L1 satellites used in solution
    uint8_t numGpsPlusGlonassL2; //!< number of GPS plus GLONASS L2 satellites used in solution
    uint8_t reserved; //!< reserved
    uint8_t extendedSolutionStatus; //!< extended solution status - OEMV and greater only
    uint8_t reserved2; //!< reserved
    uint8_t signalsUsedMask; //!< signals used mask - OEMV and greater only
    uint8_t crc[4]; //!< 32-bit cyclic redundancy check (CRC)
});


/*!
* UTM Position Message Structure
* This log contains the best available position
* computed by the receiver in UTM coordinates.
*
* The latitude limits of the UTM System are 80°S to
* 84°N. If your position is outside this range, the
* BESTUTM log outputs a northing, easting and height
* of 0.0, along with a zone letter of ‘*’and a zone
* number of 0, so that it is obvious that the data
* in the log is unusable.
*/
PACK(
struct NOVATEL_EXPORT UtmPosition : BinaryMessageBase
{
    SolutionStatus solutionStatus; //!< Solution status
    PositionType positionType; //!< Position type
    uint32_t longitudeZoneNumber; //!< longitude utm zone number
    uint32_t latitudeZoneLetter; //!< latitude utm zone letter
    double northing; //!< northing (m)
    double easting; //!< easting (m)
    double height; //!< height above mean sea level (m)
    float undulation; //!< Undulation - the relationship between the geoid and the ellipsoid (m)
    DatumID datumId; //!< datum id number
    float northingStandardDeviation; //!< northing standard deviation (m)
    float eastingStandardDeviation; //!< easting standard deviation (m)
    float heightStandardDeviation; //!< height standard deviation (m)
    int8_t baseStationId[4]; //!< base station id
    float differentialAge; //!< differential position age (sec)
    float solutionAge; //!< solution age (sec)
    uint8_t numberOfSatellites; //!< number of satellites tracked
    uint8_t numberOfSatellitesInSolution; //!< number of satellites used in solution
    uint8_t numGpsPlusGlonassL1; //!< number of GPS plus GLONASS L1 satellites used in solution
    uint8_t numGpsPlusGlonassL2; //!< number of GPS plus GLONASS L2 satellites used in solution
    uint8_t reserved; //!< reserved
    uint8_t extendedSolutionStatus; //!< extended solution status - OEMV and greater only
    uint8_t reserved2; //!< reserved
    uint8_t signalsUsedMask; //!< signals used mask - OEMV and greater only
    uint8_t crc[4]; //!< 32-bit cyclic redundancy check (CRC)
});

/*!
* IONUTC Message Structure
* This log contains The Ionospheric Model
* parametres (ION) and the Universal Time
* Coordinated parametres (UTC)
*/
PACK(
struct NOVATEL_EXPORT IonosphericModel : BinaryMessageBase
{
    double a0; //!< alpha parameter constant term
    double a1; //!< alpha parameter 1st order term
    double a2; //!< alpha parameter 2nd order term
    double a3; //!< alpha parameter 3rd order term
    double b0; //!< beta parameter constant term
    double b1; //!< beta parameter 1st order term
    double b2; //!< beta parameter 2nd order term
    double b3; //!< beta parameter 3rd order term
    uint32_t numWk; //!< UTC reference week number
    uint32_t tot; //!< reference time of UTC parameters
    double A0; //!< UTC constant term
    double A1; //!< UTC 1st order term
    uint32_t futWk; //!< future week number
    uint32_t numDay; //!< day number
    int32_t dells; //!< delta time due to leap seconds
    int32_t futDells; //!< future delta time due to leap seconds
    uint32_t delutc; //!< time difference
    uint8_t crc[4]; //!< 32-bit cyclic redundancy check (CRC)
});

/*!
* Channel Tracking Status
* Used in logs RANGE and TRACKSTAT
*/
PACK(
struct NOVATEL_EXPORT ChannelStatus 
{
	unsigned int trackingState : 5;
	unsigned int svChanNum : 5;
	unsigned int phaseLockFlag : 1;
	unsigned int parityKnownFlag : 1;
	unsigned int codeLockedFlag : 1;
	unsigned int correlatorType : 3;
	unsigned int satelliteSys : 3;
	unsigned int reserved1 : 1;
	unsigned int grouping : 1;
	unsigned int signalType : 5;
	unsigned int forwardErrCorrection : 1;
	unsigned int primaryL1Chan : 1;
	unsigned int carrierPhaseMeas : 1;
	unsigned int reserved2 : 1;
	unsigned int prnLockFlag : 1;
	unsigned int channelAssignment : 1;
});

/*!
* Pseudorange Message Structure
* This log contains the pseudorange information for a
* single channel. Used in the RangeMeasurements structure.
*/
PACK(
struct NOVATEL_EXPORT RangeData 
{
    uint16_t satellitePrn; //!< SV PRN number
    uint16_t glonassFrequency; //!< Frequency number of GLONASS SV (0 for GPS)
    double pseudorange; //!<  pseudorange [m]
    float pseudorangeStandardDeviation; //!< pseudorange standard deviation [m]
    double accumulatedDoppler; //!< accumulated doppler [cycles]
    float accumulatedDopplerStdDeviation; //!< accumulated doppler standard deviation [cycles]
    float doppler; //!< Doppler frequency [Hz]
    float carrierToNoise; //!< Signal/Noise [dB-Hz]
    float locktime; //!< Number of seconds of continuous tracking [sec]
    ChannelStatus channel_status; //!< channel tracking status
});


/*!
* RANGE Message Structure
* This log contains the channel measurements for the
* currently tracked satellites. When using this log,
* please keep in mind the constraints noted along with
* the description.
*/
PACK(
struct NOVATEL_EXPORT RangeMeasurements : BinaryMessageBase
{
    int32_t numberOfObservations; //!< Number of ranges observations in the following message
    RangeData rangeData[MAX_CHAN]; //!< Range data for each available channel
    uint8_t crc[4]; //!< 32-bit cyclic redundancy check (CRC)
});


/*!
* Compressed Pseudorange Message Structure
* This log contains the pseudorange information for a
* single channel. Used in the RangeMeasurements structure.
*/
PACK(
struct NOVATEL_EXPORT CompressedRangeRecord 
{
    int64_t doppler:28; //!< Doppler frequency [Hz]; SF = 1/256
    uint64_t pseudorange:36; //!<  pseudorange [m]; SF = 1/128
    int32_t accumulatedDoppler:32; //!< accumulated doppler [cycles]; SF = 1/256
    uint16_t pseudorangeStandardDeviation:4; //!< pseudorange standard deviation [m]
    uint16_t accumulatedDopplerStdDeviation:4; //!< accumulated doppler standard deviation [cycles]
    uint16_t satellitePrn:8; //!< SV PRN number
    uint32_t locktime:21; //!< Number of seconds of continuous tracking [sec]
    uint32_t carrierToNoise:5; //!< Signal/Noise [dB-Hz]
    uint32_t reserved:6;
    uint16_t reservedb:16;
} //;
);

PACK(
struct NOVATEL_EXPORT CompressedRangeData 
{
    ChannelStatus channelStatus; //!< channel tracking status
    CompressedRangeRecord rangeRecord;
} //;
);

/*!
* RANGECMP Message Structure
* This log contains the compressed version of the RANGE log.
*/
PACK(
struct NOVATEL_EXPORT CompressedRangeMeasurements : BinaryMessageBase
{
    int32_t numberOfObservations; //!< Number of ranges observations in the following message
    CompressedRangeData rangeData[MAX_CHAN]; //!< Range data for each available channel
    uint8_t crc[4]; //!< 32-bit cyclic redundancy check (CRC)
});

//*******************************************************************************
// SATELLITE INFORMATION GPS STRUCTURES
//*******************************************************************************


/*!
* GPSEPHEM Message Structure
* This log contains a single set of
* GPS ephemeris parametres.
*/
PACK(
struct NOVATEL_EXPORT GpsEphemeris : BinaryMessageBase
{
    uint32_t prn; //!< PRN number
    double timeOfWeek; //!< time stamp of subframe 0 (s)
    uint32_t health; //!< health status, defined in ICD-GPS-200
    uint32_t issueOfEphemeris1; //!< issue of ephemeris data 1
    uint32_t issueOfEphemeris2; //!< issue of ephemeris data 2
    uint32_t gpsWeek; //!< GPS week number
    uint32_t zCountWeek; //!< z count week number
    double timeOfEphemeris; //!< reference time for ephemeris (s)
    double semiMajorAxis; //!< semi major axis (m)
    double meanMotionDifference; //!< Mean motion difference (rad/s)
    double anomolyReferenceTime; //!< mean anomoly reference time (rad)
    double eccentricity; //!< eccentricity
    double omega; //!< arguement of perigee (rad)
    double latitudeCosine; //!< arugument of latitude - cos (rad)
    double latitudeSine; //!< argument of latitude - sine (rad)
    double orbitRadiusCosine; //!< orbit radius - cos (rad)
    double orbitRadiusSine; //!< orbit radius - sine (rad)
    double inclinationCosine; //!< inclination - cos (rad)
    double inclinationSine; //!< inclination - sine (rad)
    double inclinationAngle; //!< inclination angle (rad)
    double inclinationAngleRate; //!< rate of inclination angle (rad/s)
    double rightAscension; //!< right ascension (rad)
    double rightAscensionRate; //!< rate of right ascension (rad/s)
    uint32_t issueOfDataClock; //!< issue of data clock
    double svClockCorrection; //!< SV clock correction term (s)
    double groupDelayDifference; //!< estimated group delay difference
    double clockAligningParam0; //!< clock aging parameter 0
    double clockAligningParam1; //!< clock aging parameter 1
    double clockAligningParam2; //!< clock aging parameter 2
    yes_no antiSpoofing; //!< anti spoofing on
    double correctedMeanMotion; //!< corrected mean motion
    double rangeAccuracyVariance; //!< user range accuracy variance
    uint8_t crc[4]; //!< 32-bit cyclic redundancy check (CRC)
});

/*!
* RAWEPHEM Message Structure
* contains the raw binary information for subframes one, two
* and three from the satellite with the parity information removed.
* Ephemeris older than 6 hours is not output
*/
PACK(
struct NOVATEL_EXPORT RawEphemeris : BinaryMessageBase
{
    uint32_t prn; //!< Satellite PRN number
    uint32_t ephemReferenceWeekNum; //!< Ephemeris reference week number
    uint32_t ephemReferenceSeconds; //!< Ephemeris reference time [sec]
    uint8_t subframe1[30]; //!< Subframe 1 data
    uint8_t subframe2[30]; //!< Subframe 2 data
    uint8_t subframe3[30]; //!< Subframe 3 data
    uint8_t crc[4]; //!< 32-bit cyclic redundancy check (CRC)
});

PACK(
struct NOVATEL_EXPORT RawEphemerides 
{
    RawEphemeris ephemeris[MAX_NUM_SAT];
});

/*!
* RAWALM Message Structure
* Contains the undecoded almanac subframes as received from the satellite
*/
PACK(
struct NOVATEL_EXPORT RawAlmanacData
{
	uint16_t svid;
    uint8_t subframe[30]; // 30 bytes of subframe page data
});

PACK(
struct NOVATEL_EXPORT RawAlmanac : BinaryMessageBase
{
	uint32_t refWeek;
	uint32_t refTime; // [sec]
	uint32_t numOfSubframes; // numbers of subframes to follow
	RawAlmanacData subframeData;
	uint8_t crc[4];
});

/*!
* ALMANAC
* Contains decoded almanac parameters from Subframes 4 and 5 with parity 
* info removed.
*/
PACK(
struct NOVATEL_EXPORT AlmanacData 
{
	uint32_t prn;
	uint32_t refWeek;
	double refTime; //!< [sec]
	double eccentricity;
	double rightAscensionRate; //!< [rad/sec]
	double rightAscension; //!< [rad]
	double perigee; //!< [rad]
	double meanAnomolyOfRefTime; //!< [rad]
	double clockAgingParam0; //!< [sec]
	double clockAgingParam1; //!< [sec/sec]
	double correctedMeanMotion; //!< [rad/sec]
	double semiMajorAxis; //!< [m]
	double inclinationAngle; //!< [rad] Angle of inclination relative to .3*pi
	uint32_t svConfiguration; //!< 
	uint32_t svHealth; //!< (6 bits) From Page 25 of subframe 4 or 5
	uint32_t svHealthFromAlmanac; //!< (8 bits) 
	true_false antiSpoofing; //!< 
});

PACK(
struct NOVATEL_EXPORT Almanac : BinaryMessageBase
{
	int32_t numberOfPrns;
	AlmanacData data[MAX_NUM_SAT];
	uint8_t crc[4];
});

/*!
* Satellite Position Structure
* Contains the position of one satellite in ECEF coordinates.
* Also contains satellite correction information.  Used to make
* up the SatellitePositions structure
*/
PACK(
struct NOVATEL_EXPORT SatellitePositionData 
{
    uint32_t satellitePrn; //!< SV PRN number
    double xPosition; //!< SV X coordinate [m]
    double yPosition; //!< SV Y coordinate [m]
    double zPosition; //!< SV Z coordinate [m]
    double clockCorrection; //!< SV clock correction [m]
    double ionosphericCorrection; //!< ionospheric correction [m]
    double troposphericCorrection; //!< tropospheric correction [m]
    double dReserved1; //!< reserved
    double dReserved2; //!< reserved
});

/*!
* SATXYZ Message Structure
* When combined with a RANGE log, this data set contains
* the decoded satellite information necessary to compute the
* solution: satellite coordinates (ECEF WGS84), satellite clock
* correction, ionospheric corrections and tropospheric corrections.
*/
PACK(
struct NOVATEL_EXPORT SatellitePositions : BinaryMessageBase
{
    double dReserved1; //!< Reserved
    uint32_t numberOfSatellites; //!< Number of satellites in following message
    SatellitePositionData data[MAX_CHAN]; //!< Position data for each satellite
    uint8_t crc[4]; //!< 32-bit cyclic redundancy check (CRC)
});

/*!
* SATVIS Message Structure
* The SATVIS log is meant to provide a brief overview. The satellite positions
* and velocities used in the computation of this log are based on Almanac
* orbital parameters, not the higher precision Ephemeris parameters
*/
PACK(
struct NOVATEL_EXPORT SatelliteVisibilityData 
{
    int16_t satellite_prn; //!< SV PRN number
    //!< GPS 1-32
    //!< SBAS 120-138
    //!< GLONASS
    int16_t glonassFrequency; //!< GLONASS frequency +7
    uint32_t health; //!< Satellite Health
    double elevation; //!< SV elevation [deg]
    double azimuth; //!< SV azimuth [deg]
    double theoreticalDoppler; //!< Theoretical Doppler frequency of SV [Hz]
    double apparentDoppler; //!< Theoretical Doppler with clock drift correction added [Hz]
});

PACK(
struct NOVATEL_EXPORT SatelliteVisibility : BinaryMessageBase
{
    true_false satVis; //!< Reserved
    true_false completeAlmanacUsed; //!< Was Complete almanac used
    uint32_t numberOfSatellites; //!< Number of satellites in following message
    SatelliteVisibilityData data[MAX_CHAN]; //!< Position data for each satellite
    uint8_t crc[4]; //!< 32-bit cyclic redundancy check (CRC)
});


/*!
* TIME Message Structure
* This log provides several time related pieces of information
* including receiver clock offset and UTC time and offset. It
* can also be used to determine any offset in the PPS signal
* relative to GPS time. To find any offset in the PPS signal,
* log the TIME log 'ontime' at the same rate as the PPS output.
*
* GPS time = receiver time - offset
* UTC time = GPS time + offset + UTC offset
*/
PACK(
struct NOVATEL_EXPORT TimeOffset : BinaryMessageBase
{
    uint32_t clockModelStatus; //!< ClockModelStatus
    double offset; //!< Receiver Offset in seconds from GPS time
    double offsetStandardDeviation; //!< Instantaneous Standard Deviation of Receiver Clock Offset
    double gpsToUtcOffset; //!< Offset in seconds of GPS time from UTC time
    int32_t utcYear; //!< UTC Year
    uint8_t utcMonth; //!< UTC Month
    uint8_t utcDay; //!< UTC Day
    uint8_t utcHour; //!< UTC Hour
    uint8_t utcMinute; //!< UTC Minutes
    int32_t utcMillisecond; //!< UTC Milliseconds
    int32_t utcStatus; //!< UTC Status
    uint8_t crc[4]; //!< 32-bit cyclic redundancy check (CRC)
});

/*!
* TRACKSTAT Message Structure
* This log provides the Tracking Status information for each
* receiver channel
*/
struct NOVATEL_EXPORT TrackStatusData
{
    uint16_t prn; //!< SV prn
    int16_t glonassFrequency; //!< GLONASS frequency +7
    ChannelStatus channelTrackStatus; //!< Channel tracking status
    double pseudorange; //!< Pseudorange
    float dopplerFrequency; //!< Doppler frequency [Hz]
    float cnr; //!< Carrier to noise density ratio [dB-Hz]
    float lockTime; //!< Number of seconds of continuous tracking (no cycle slips)
    float pseudorangeResidual; //!< Pseudorange residual from pseudorange filter [m]
    RangeRejectCode rangeRejectCode; //!< Range reject code from pseudorange filter
    float pseudorangeWeight; //!< Pseudorange filter weighting
};

struct NOVATEL_EXPORT TrackStatus : BinaryMessageBase
{
    SolutionStatus solutionStatus; //!< Solution status
    PositionType positionType; //!< Position type
    float elevationCutoffAngle; //!< Tracking elevation cutoff angle
    int32_t numberOfChannels; //!< Number of channels with information following
    TrackStatusData data[MAX_CHAN]; //!< Tracking Status data repeated per channe
    uint8_t crc[4];
};

//*******************************************************************************
// RTK GPS STRUCTURES
//*******************************************************************************

//TODO: CONVERT AND CLEANUP THESE STRUCTURES

//********************
//RTKDATAB
PACK(
struct NOVATEL_EXPORT rtkdatab_header 
{
    uint32_t rtkinfo; //RTK information
    uint8_t numObs; //Number of observations tracked
    uint8_t gpsL1Ranges; // Number of GPS L1 ranges used
    uint8_t gpsL1MaskRanges; // Number of GPS L1 ranges above the RTK mask angle used
    uint8_t gpsL2MaskRanges; // Number of GPS L2 ranges above the RTK mask angle used
    uint8_t reserved[4];
    SEARCHER_TYPE searcherType; // Searcher type
    uint32_t numLaneCombs; // Number of possible lane combinations
    float Cxx; // ECEF position covariance matrix
    float Cxy;
    float Cxz;
    float Cyx;
    float Cyy;
    float Cyz;
    float Czx;
    float Czy;
    float Czz;
    double deltaX; // Float solution baseline in ECEF - x
    double deltaY; // Float solution baseline in ECEF - y
    double deltaZ; // Float solution baseline in ECEF - z
    float sdX; // Standard deviation of float solution baseline in ECEF - x
    float sdY; // Standard deviation of float solution baseline in ECEF - y
    float sdZ; // Standard deviation of float solution baseline in ECEF - z
    uint32_t refPrn; // Reference PRN
    int32_t numSvs; // The number of SVs in data portion
});

PACK(
struct NOVATEL_EXPORT rtkdatab_data 
{
    uint32_t prn; // GPS satellite PRN
    AMBIGUITY_TYPE ambiguityType; // Type of ambiguity
    float residual; // Satellite health
});

PACK(
struct NOVATEL_EXPORT rtkdatab_log : BinaryMessageBase
{
    SolutionStatus solutionStatus; //Solution status
    PositionType positionType; //Position type
    rtkdatab_header header;
    rtkdatab_data data[MAX_NUM_SAT];
});

//********************


//********************
//RTCADATA1B

PACK(
struct NOVATEL_EXPORT rtcadata1b_header 
{
    double zcount; //Week number from subframe one of the ephemeris
    uint8_t aeb; //Acceleration error bound
    uint32_t num_prn; //Number of satellite corrections with info to follow
});

PACK(
struct NOVATEL_EXPORT rtcadata1b_data 
{
    uint32_t prn; //PRN number of range measurement
    double range; //pseudorange correction (m)
    uint8_t iode; //Issue of ephemeris data
    double rrate; //pseudorange rate correction (m/s)
    float udre; //user differential range error
});

PACK(
struct NOVATEL_EXPORT rtcadata1b_log : BinaryMessageBase
{
    rtcadata1b_header info;
    rtcadata1b_data data[MAX_NUM_SAT];
});

//********************


//********************
//RTCADATAEPHEMB

PACK(
struct NOVATEL_EXPORT rtcadataephemb_data 
{
    uint8_t des; //Novatel designator
    uint8_t subtype; //RTCA message subtype
    uint32_t week; //GPS week number
    uint32_t sec; //Seconds into week
    uint32_t prn; //PRN number
    int8_t reserved[4];
    int8_t ephem[92]; //Raw ephemeris data
});

PACK(
struct NOVATEL_EXPORT rtcadataephemb_log : BinaryMessageBase
{
    rtcadataephemb_data data;
});

//********************


//********************
//RTCADATAOBSB - CHECK
PACK(
struct NOVATEL_EXPORT rtcadataobsb_header 
{
    uint8_t des; //Novatel designator
    uint8_t subtype; //RTCA message subtype
    double minPsr; //minimum pseudorange
    float sec; //seconds into GPS week
    int8_t reserved[4];
    uint32_t num_ids; //Number of transmitter ids with info to follow
});

PACK(
struct NOVATEL_EXPORT rtcadataobsb_data //Structure for RTCADATAEPHEM message
{
    uint8_t transId; //Transmitter ID
    uint8_t l1Lock; //L1 lock flag
    uint8_t l2Lock; //L2 lock flag
    double l1Psr; //L1 pseudorange offset
    double l2Psr; //L2 pseudorange offset
    float l1Adr; //L1 carrier phase offset, accumulated doppler range
    float l2Adr; //L2 carrier phase offset, accumulated doppler range
    yes_no l2Encrypt; //If L2 is encrypted
    int8_t reserved[4];
});

PACK(
struct NOVATEL_EXPORT rtcadataobsb_log : BinaryMessageBase
{
    rtcadataobsb_header info;
    rtcadataobsb_data data[MAX_NUM_SAT]; //WT:  This is probably too many... need to verify how many id's can be sent.
});

//********************


//********************
//RTCADATAREFB
PACK(
struct NOVATEL_EXPORT rtcadatarefb_data 
{
    uint8_t des; //Novatel designator
    uint8_t subtype; //RTCA message subtype
    double posX; //base station X coordinate position (mm)
    double posY; //base station Y coordinate position (mm)
    double posZ; //base station Z coordinate position (mm)
    int8_t reserved[4];
});

PACK(
struct NOVATEL_EXPORT rtcadatarefb_log : BinaryMessageBase
{
    rtcadatarefb_data data;
});

PACK(
struct NOVATEL_EXPORT GeneralData : BinaryMessageBase
{
    std::vector<uint8_t> data;
    uint8_t crc[4];
});


//********************


//********************
//RTCM DATA TYPE

PACK(
    struct RTCM3Header
{
    uint8_t  preamble : 8;      // must be 11010011b
    uint8_t lengthHigh : 2;     // Message length in bytes
    uint8_t  reserved : 6;      // must be 000000b
    uint8_t lengthLow;
    uint16_t getLength()
    {
        return uint16_t(lengthHigh) << 8 | lengthLow;
    }
});

//Contents of the Message Header, Types 1001, 1002, 1003, 1004: GPS RTK Messages
PACK(
struct NOVATEL_EXPORT RtkMessageHeader
{
    uint16_t number         : 12;   // Message Number
    uint16_t stationID      : 12;   // Reference Station ID
    uint32_t TOW            : 30;   // GPS Epoch Time (TOW)
    uint8_t  flag           : 1;    // Synchronous GNSS Flag
    uint8_t  noGps          : 5;    // No. of GPS Satellite Signals Processed
    uint8_t  indicatior     : 1;    // GPS Divergence-free Smoothing Indicator
    uint8_t  interval       : 3;    // GPS Smoothing Interval
});

//*******************************************************************************
// STATUS STRUCTURES
//*******************************************************************************


/*!
* VERSION Message Structure
* This log contains the version information for
* all components of a system. When using a standard
* receiver, there is only one component in the log. A
* component may be hardware (for example, a receiver
* or data collector) or firmware in the form of
* applications or data (for example, data blocks for
* height models or user applications).
*
*/
PACK(
struct NOVATEL_EXPORT Version : BinaryMessageBase
{
	int32_t number_of_components; //!< Number of components (cards, etc..)
	int32_t component_type; //!< Component type
	char model[16]; //!< Base model name
	char serial_number[16]; //!< Product serial number
	char hardware_version[16]; //!< Hardware version number
	char software_version[16]; //!< firmware version number
	char boost_version[16]; //!< boot code version
	char compile_date[12]; //!< Firmware compile date
	char compile_time[12]; //!< Firmware compile time
	int8_t crc[4];
});


struct NOVATEL_EXPORT ReceiverError
{
    int32_t DRAMStatus :1;
    int32_t invalidFirmware : 1;
    int32_t ROMStatus : 1;
    int32_t reserved1 : 1;
    int32_t ESNaccessStatus : 1;
    int32_t authorizationCodeStatus : 1;
    int32_t slowADCStatus : 1;
    int32_t supplyVoltageStatus : 1;
    int32_t thermometerStatus : 1;
    int32_t temperatusStatus : 1;
    int32_t MINOS4Status : 1;
    int32_t PLLRf1Status : 1;
    int32_t PLLRf2Status : 1;
    int32_t RF1Status : 1;
    int32_t RF2Status : 1;
    int32_t NVMStatus : 1;
    int32_t softwareResourceLimit : 1;
    int32_t reserved2 : 3;
    int32_t remoteLoadingBegun : 1;
    int32_t exportRestriction : 1;
    int32_t reserved3 : 9;
    int32_t componentHardwareFailure : 1;
};

struct NOVATEL_EXPORT ReceiverStatus
{
    int32_t errorFlag : 1;
    int32_t temperatureStatus : 1;
    int32_t voltageSupplyStatus : 1;
    int32_t antennaPowerStatus : 1;
    int32_t reserved1 : 1;
    int32_t antennaOpenFlag : 1;
    int32_t antennaShortedFlag : 1;
    int32_t CPUoverloadFlag : 1;
    int32_t COM1overrunFlag : 1;
    int32_t COM2overrunFlag : 1;
    int32_t COM3overrunFlag : 1;
    int32_t USBoverrun : 1;
    int32_t reserved2 : 3;
    int32_t RF1AGCStatus : 1;
    int32_t reserved3 : 1;
    int32_t RF2AGCStatus : 1;
    int32_t almanacFlag : 1;
    int32_t positionSolutionFlag : 1;
    int32_t positionFixedFlag : 1;
    int32_t clockSteeringStatus : 1;
    int32_t clockModelFlag : 1;
    int32_t extOscillatorFlag : 1;
    int32_t softwarerResource : 1;
    int32_t reserved4 : 4;
    int32_t AUX3statusEventFlag : 1;
    int32_t AUX2statusEventFlag : 1;
    int32_t AUX1statusEventFlag : 1;
};


/*!
* RXSTATUS Message Structure
* This log conveys various status parameters
* of the GNSS receiver system. These include
* the Receiver Status and Error words which contain
* several flags specifying status and error conditions.
* If an error occurs (shown in the Receiver Error word)
* the receiver idles all channels, turns off the antenna,
* anddisables the RF hardware as these conditions are
* considered to be fatal errors. The log contains a variable
* number of status words to allow for maximum flexibility and
* future expansion.
*/
PACK(
struct NOVATEL_EXPORT RXStatus : BinaryMessageBase
{
	ReceiverError error; //!< receiver error field
	uint32_t numStats; //!< number of status messages
	ReceiverStatus rxStat; //!< receiver status word
	uint32_t rxStatPri;
	uint32_t rxStatSet;
	uint32_t rxStatClear;
	uint32_t aux1Stat; //!< auxiliary 1 status field
	uint32_t aux1Pri;
	uint32_t aux1Set;
	uint32_t aux1Clear;
	uint32_t aux2Stat;
	uint32_t aux2Pri;
	uint32_t aux2Set;
	uint32_t aux2Clear;
	uint32_t aux3Stat;
	uint32_t aux3Pri;
	uint32_t aux3Set;
	uint32_t aux3Clear;
	int8_t crc[4];
});


struct NOVATEL_EXPORT RXStatusEvent : BinaryMessageBase
{
    StatusWord status; // the status word that generated the event message
    uint32_t bitPosition; // location of the bit in the status word (Table 81, pg 303
    EventType type; // event type (Table 86, pg 306)
    int8_t description[32]; // text description of the event or error
};

/*!
* RXHWLEVELS Message Structure
* This log contains the receiver environmental and voltage parametres.
*/
struct NOVATEL_EXPORT ReceiverHardwareStatus : BinaryMessageBase
{
    float board_temperature; //!< board temperature in degrees celcius
    float antenna_current; //!< antenna current (A)
    float core_voltage; //!< CPU core voltage (V)
    float supply_voltage; //!< supply voltage(V)
    float rf_voltage; //!< 5V RF supply voltage(V)
    float lna_voltage; //!< internal LNA voltage (V)
    float GPAI; //!< general purpose analog input
    float reserved1; //!< Reserved
    float reserved2; //!< Reserved
    float lna_card_voltage; //!< LNA voltage (V) at GPSCard output
    int8_t crc[4]; //!< 32-bit crc
};
}


#endif
