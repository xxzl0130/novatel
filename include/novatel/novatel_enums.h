// Novatel OEM4 enumerations
#ifndef NOVATELENUMS_H
#define NOVATELENUMS_H

#ifdef NOVATEL_DLL
#define NOVATEL_EXPORT __declspec(dllexport)
#else
#define NOVATEL_EXPORT __declspec(dllimport)
#endif

#include <stdint.h>  // use fixed size integer types, rather than standard c++ types

namespace novatel
{
#define THISPORT 0xc0

    //*******************************************************************************
    // USER-DEFINED ENUMS
    //*******************************************************************************

    enum true_false : uint8_t
    {
        FALSE_FLAG = 0,
        TRUE_FLAG = 1
    };

    enum return_type
    {
        success,
        fail
    };

    enum base_type
    {
        stationary,
        dynamic
    };

    enum yes_no : uint8_t
    {
        no,
        yes
    };

    enum rec_type
    {
        stand_alone,
        rover_with_static_base,
        static_base_station,
        rover_with_dynamic_base,
        dynamic_base_station
    };


    //*******************************************************************************
    // NOVATEL ENUMS
    //*******************************************************************************

    enum RangeRejectCode : uint32_t
    {
        //!< Used in TRACKSTAT
        GOOD = 0,
        //!< Observation is good
        BADHEALTH = 1,
        //!< Bad SV health indicated by ephemeris
        OLDEPHEMERIS = 2,
        //!< Ephemeris not updated during the las 3 hours
        ECCENTRICANOMALY = 3,
        //!< Eccentric anomaly error during computation of SV position
        TRUEANOMALY = 4,
        //!< True anomaly error during computation of SV position
        SATCOORDINATEERROR = 5,
        //!< SV coordinate error during computation of SV position
        ELEVATIONERROR = 6,
        //!< Elevation error due to SV below the cut-off angle
        MISCLOSURE = 7,
        //!< Misclosure too large due to excessive gap between estimated and actual positions
        NODIFFCORR = 8,
        //!< No compatible differential correction is available for this particular satellite
        NOEPHEMERIS = 9,
        //!< Ephemeris data not yet received for this SV
        INVALIDIODE = 10,
        //!< Invalide IODE (Issue of Data Ephemeris)
        LOCKEDOUT = 11,
        //!< SV is excluded by the user (LOCKOUT command)
        LOWPOWER = 12,
        //!< SV rejected due to low C/No ratio
        NOIONOCORR = 16,
        //!< No ionospheric correction available for this SV
        NOTUSED = 17,
        //!< Observation ignored and not used in solution
        NA = 99,
        //!< No obseration
        BAD_INTEGRITY = 100,
        //!< Integrity of the pseudorange is bad
    };

    enum AMBIGUITY_TYPE
    {
        UNDEFINED = 0,
        FLOAT_L1 = 1,
        FLOAT_IONOFREE = 2,
        FLOAT_NARROW = 3,
        NLF_FROM_WL1 = 4,
        INT_L1 = 5,
        INT_WIDE = 6,
        INT_NARROW = 7,
        IONOFREE_DISCRETE = 8
    };

    enum SEARCHER_TYPE
    {
        NONE_REQUESTED = 0,
        BUFFERING_MEASUREMENTS = 1,
        SEARCHING = 2,
        COMPLETE = 3,
        HANDOFF_COMPLETE = 4
    };


    enum TIME_STATUS
    {
        GPSTIME_UNKNOWN = 20,
        GPSTIME_APPROXIMATE =60,
        GPSTIME_COARSEADJUSTING=80,
        GPSTIME_COARSE=100,
        GPSTIME_COARSESTEERING=120,
        GPSTIME_FREEWHEELING=130,
        GPSTIME_FINEADJUSTING=140,
        GPSTIME_FINE=160,
        GPSTIME_FINESTEERING=180,
        GPSTIME_SATTIME=200,
    };


    enum LogMode
    {
        ONNEW,
        //!< does not output current message, but outputs when message is updated
        ONCHANGED,
        //!< outputs the current message and then continues to output when the message is changed
        ONTIME,
        //!< output on a time interval
        ONNEXT,
        //!< output only the next message
        ONCE,
        //!< output only the current message
        ONMARK,
        //!< output when a pulse is detected on the mark1 input, MK1I
        STOPPED //!< unlog message
    };


    enum SolutionStatus : uint32_t
    {
        SOL_COMPUTED,
        //!< solution computed
        INSUFFICIENT_OBS,
        //!< insufficient observations
        NO_CONVERGENCE,
        //!< noconvergence
        SINGULARITY,
        //!< singularity at parameters matrix
        COV_TRACE,
        //!< covariance trace exceeds maximum (trace>1000m)
        TEST_DIST,
        //!< test distance exceeded (max of 3 rejections if distance > 10km)
        COLD_START,
        //!< not yet converged from cold start
        V_H_LIMIT,
        //!< height or velocity limits exceeded 
        VARIANCE,
        //!< variance exceeds limits
        RESIDUALS,
        //!< residuals are too large
        DELTA_POS,
        //!< delta position is too large
        NEGATIVE_VAR,
        //!< negative variance
        INTEGRITY_WARNING=13,
        //!< large residuals make position unreliable
        INS_INACTIVE,
        //!< ins has not started yet
        INS_ALIGNING,
        //!< ins doing its coarse alignment
        INS_BAD,
        //!< ins position is bad
        IMU_UNPLUGGED,
        //!< no imu detected
        PENDING,
        //!< when a fix position command is entered, the receiver computes its own position and determines if the fixed position is valid
        INVALID_FIX,
        //!< the fixed position entered using the fix position command is not valid
        UNAUTHORIZED
    };

    enum MessageFormat : uint8_t //!< Bits 5-6 of MessageType struct
    {
        BINARY = 0b00,
        ASCII = 0b01,
        ABREVIATED_ASCII = 0b10,
        NMEA = 0b11,
    };

    enum ResponseBit : uint8_t //!< Last bit (7) of MessageType struct
    {
        ORIGINAL_MESSAGE = 0b0,
        RESPONSE_MESSAGE = 0b1,
    };

    enum PositionType : uint32_t
    {
        NONE = 0,
        FIXEDPOS = 1,
        FIXEDHEIGHT = 2,
        Reserved = 3,
        FLOATCONV = 4,
        WIDELANE = 5,
        NARROWLANE = 6,
        DOPPLER_VELOCITY = 8,
        SINGLE = 16,
        PSRDIFF = 17,
        WAAS = 18,
        PROPOGATED = 19,
        OMNISTAR = 20,
        L1_FLOAT = 32,
        IONOFREE_FLOAT = 33,
        NARROW_FLOAT = 34,
        L1_INT = 48,
        WIDE_INT = 49,
        NARROW_INT = 50,
        RTK_DIRECT_INS = 51,
        INS = 52,
        INS_PSRSP = 53,
        INS_PSRDIFF = 54,
        INS_RTKFLOAT = 55,
        INS_RTKFIXED = 56,
        OMNISTAR_HP = 64,
        OMNISTAR_XP = 65,
        CDGPS = 66,
    };

    enum DatumID : uint32_t
    {
        ADIND=1,
        ARC50,
        ARC60,
        AGD66,
        AGD84,
        BUKIT,
        ASTRO,
        CHATM,
        CARTH,
        CAPE,
        DJAKA,
        EGYPT,
        ED50,
        ED79,
        GUNSG,
        GEO49,
        GRB36,
        GUAM,
        HAWAII,
        KAUAI,
        MAUI,
        OAHU,
        HERAT,
        HJORS,
        HONGK,
        HUTZU,
        INDIA,
        IRE65,
        KERTA,
        KANDA,
        LIBER,
        LUZON,
        MINDA,
        MERCH,
        NAHR,
        NAD83,
        CANADA,
        ALASKA,
        NAD27,
        CARIBB,
        MEXICO,
        CAMER,
        MINNA,
        OMAN,
        PUERTO,
        QORNO,
        ROME,
        CHUA,
        SAM56,
        SAM69,
        CAMPO,
        SACOR,
        YACAR,
        TANAN,
        TIMBA,
        TOKYO,
        TRIST,
        VITI,
        WAK60,
        WGS72,
        WGS84,
        ZANDE,
        USER,
        CSRS,
        ADIM,
        ARSM,
        ENW,
        HTN,
        INDB,
        INDI,
        IRL,
        LUZA,
        LUZB,
        NAHC,
        NASP,
        OGBM,
        OHAA,
        OHAB,
        OHAC,
        OHAD,
        OHIA,
        OHIB,
        OHIC,
        OHID,
        TIL,
        TOYM
    };

    enum StatusWord
    {
        RCV_ERROR,
        //!< Receiver error word
        RCV_STATUS,
        //!< receiver status word
        AUX1,
        //!< auxillary 1 status word
        AUX2,
        //!< auxillary 2 status word
        AUX3 //!< auxillary 3 status word
    };

    enum EventType
    {
        CLEAR=0,
        //!< bit was cleared
        SET=1 //!< bit was set
    };

    enum PDPSwitch //!< Used in PDPFILTER Command
    {
        DISABLE = 0,
        ENABLE = 1,
        RESET = 2,
    };

    enum PDPMode
    {
        PDP_NORMAL = 0,
        PDP_RELATIVE = 1,
    };

    enum PDPDynamics
    {
        AUTO = 0,
        //!< Autodetect dynamics mode
        STATIC = 1,
        //!< Static Mode
        DYNAMIC = 2,
        //!< Dynamic Mode
    };

    enum FRESET_TARGET
    {
        STANDARD = 0,
        //!< [DEFAULT] Clears commands, ephemeris, and almanac
        COMMAND = 1,
        //!< Clears saved configuration
        GPSALMANAC = 2,
        //!< Clears stored GPS almanac
        GPSEPHEM = 3,
        //!< Clears stored GPS ephemeris
        GLOEPHEM = 4,
        //!< Clears stored GLONASS ephemeris
        MODEL = 5,
        //!< Clears the currently selected model
        CLKCALIBRATION = 11,
        //!< Clears parameters entered using CLOCKCALIBRATE command
        SBASALMANAC = 20,
        //!< Clears stored SBAS almanac
        LAST_POSITION = 21,
        //!< Resets the position using the last stored position
        GLOALMANAC = 31,
        //!< Clears the stored GLONASS almanac
        LBAND_TCXO_OFFSET = 38,
        //!< Removes the TCXO offset information from NVM (not in OEMStar firmware)
    };

    enum BINARY_LOG_TYPE : int16_t
    {
        // OEM6 logs
        LOGLIST_LOG_TYPE = 5,	// A list of system logs
        GPSEPHEM_LOG_TYPE = 7,	// GPS ephemeris data
        IONUTC_LOG_TYPE = 8,	// Ionospheric and UTC model information
        CLOCKMODEL_LOG_TYPE = 16,	// Current clock model matrices
        RAWGPSSUBFRAME_LOG_TYPE = 25,	// Raw subframe data
        CLOCKSTEERING_LOG_TYPE = 26,	// Clock steering status
        VERSION_LOG_TYPE = 37,	// Receiver hardware and software version numbers
        RAWEPHEM_LOG_TYPE = 41,	// Raw ephemeris
        BESTPOS_LOG_TYPE = 42,	// Best position data
        RANGE_LOG_TYPE = 43,	// Satellite range information
        PSRPOS_LOG_TYPE = 47,	// Pseudorange position information
        SATVIS_LOG_TYPE = 48,	// Satellite visibility
        PORTSTATS_LOG_TYPE = 72,	// COM or USB port statistics
        ALMANAC_LOG_TYPE = 73,	// Current almanac information
        RAWALM_LOG_TYPE = 74,	// Raw almanac
        TRACKSTAT_LOG_TYPE = 83,	// Satellite tracking status
        RXSTATUS_LOG_TYPE = 93,	// Self-test status
        RXSTATUSEVENT_LOG_TYPE = 94,	// Status event indicator
        MATCHEDPOS_LOG_TYPE = 96,	// RTK Computed Position ¨C Time Matched
        BESTVEL_LOG_TYPE = 99,	// Velocity data
        PSRVEL_LOG_TYPE = 100,	// Pseudorange velocity information
        TIME_LOG_TYPE = 101,	// Receiver time information
        RXCONFIG_LOG_TYPE = 128,	// Receiver configuration status
        RANGECMP_LOG_TYPE = 140,	// Compressed version of the RANGE log
        RTKPOS_LOG_TYPE = 141,	// RTK low latency position data
        DIRENT_LOG_TYPE = 159,	// Onboard memory file list
        NAVIGATE_LOG_TYPE = 161,	// Navigation waypoint status
        AVEPOS_LOG_TYPE = 172,	// Position averaging
        PSRDOP_LOG_TYPE = 174,	// DOP of SVs currently tracking
        REFSTATION_LOG_TYPE = 175,	// Base station position and health
        MARKPOS_LOG_TYPE = 181,	// Position at time of mark1 input event
        VALIDMODELS_LOG_TYPE = 206,	// Model and expiry date information for receiver
        RTKVEL_LOG_TYPE = 216,	// RTK velocity
        MARKTIME_LOG_TYPE = 231,	// Time of mark1 input event
        PASSCOM1_LOG_TYPE = 233,	// Pass-through logs
        PASSCOM2_LOG_TYPE = 234,	// Pass-through logs
        PASSCOM3_LOG_TYPE = 235,	// Pass-through logs
        BESTXYZ_LOG_TYPE = 241,	// Cartesian coordinate position data
        MATCHEDXYZ_LOG_TYPE = 242,	// RTK Time Matched cartesian coordinate position data
        PSRXYZ_LOG_TYPE = 243,	// Pseudorange cartesian coordinate position information
        RTKXYZ_LOG_TYPE = 244,	// RTK cartesian coordinate position data
        PASSXCOM1_LOG_TYPE = 405,	// Pass-through logs
        PASSXCOM2_LOG_TYPE = 406,	// Pass-through logs
        RAWGPSWORD_LOG_TYPE = 407,	// Raw navigation word
        PDPPOS_LOG_TYPE = 469,	// PDP filter position
        PDPVEL_LOG_TYPE = 470,	// PDP filter velocity
        PDPXYZ_LOG_TYPE = 471,	// PDP filter Cartesian position and velocity
        TIMESYNC_LOG_TYPE = 492,	// Synchronize time between receivers
        OMNIHPPOS_LOG_TYPE = 495,	// OmniSTAR HP/XP/G2 position data
        APPLICATIONSTATUS_LOG_TYPE = 520,	// Provides application status information
        PASSUSB1_LOG_TYPE = 607,	// Pass-through logs (for receivers that support USB)
        PASSUSB2_LOG_TYPE = 608,	// Pass-through logs (for receivers that support USB)
        PASSUSB3_LOG_TYPE = 609,	// Pass-through logs (for receivers that support USB)
        MARK2POS_LOG_TYPE = 615,	// Time of mark input2 event
        MARK2TIME_LOG_TYPE = 616,	// Position at time of mark2 input event
        RANGEGPSL1_LOG_TYPE = 631,	// L1 version of the RANGE log
        BSLNXYZ_LOG_TYPE = 686,	// RTK XYZ baseline
        PASSAUX_LOG_TYPE = 690,	// Pass-through log for AUX port
        GLOALMANAC_LOG_TYPE = 718,	// GLONASS almanac data
        GLOCLOCK_LOG_TYPE = 719,	// GLONASS clock information
        GLORAWALM_LOG_TYPE = 720,	// Raw GLONASS almanac data
        GLORAWFRAME_LOG_TYPE = 721,	// Raw GLONASS frame data
        GLORAWSTRING_LOG_TYPE = 722,	// Raw GLONASS string data
        GLOEPHEMERIS_LOG_TYPE = 723,	// GLONASS ephemeris data
        BESTUTM_LOG_TYPE = 726,	// Best available UTM data
        LBANDINFO_LOG_TYPE = 730,	// L-Band configuration information
        LBANDSTAT_LOG_TYPE = 731,	// L-Band status information
        RAWLBANDFRAME_LOG_TYPE = 732,	// Raw L-Band frame data
        RAWLBANDPACKET_LOG_TYPE = 733,	// Raw L-Band data packet
        GLORAWEPHEM_LOG_TYPE = 792,	// Raw GLONASS ephemeris data
        PASSXCOM3_LOG_TYPE = 795,	// Pass through log
        OMNIVIS_LOG_TYPE = 860,	// OmniSTAR satellite visibility list
        PSRTIME_LOG_TYPE = 881,	// Time offsets from the pseudorange filter
        RTKDOP_LOG_TYPE = 952,	// Values from the RTK fast filter
        HWMONITOR_LOG_TYPE = 963,	// Monitor Hardware Levels
        HEADING_LOG_TYPE = 971,	// Heading information with the ALIGN feature
        RAWSBASFRAME_LOG_TYPE = 973,	// Raw SBAS frame data
        SBAS0_LOG_TYPE = 976,	// Remove PRN from the solution
        SBAS1_LOG_TYPE = 977,	// PRN mask assignments
        SBAS10_LOG_TYPE = 978,	// Degradation factor
        SBAS12_LOG_TYPE = 979,	// SBAS network time and UTC
        SBAS17_LOG_TYPE = 980,	// GEO almanac message
        SBAS18_LOG_TYPE = 981,	// IGP mask
        SBAS2_LOG_TYPE = 982,	// Fast correction slots 0-12
        SBAS24_LOG_TYPE = 983,	// Mixed fast/slow corrections
        SBAS25_LOG_TYPE = 984,	// Long term slow satellite corrections
        SBAS26_LOG_TYPE = 985,	// Ionospheric delay corrections
        SBAS27_LOG_TYPE = 986,	// SBAS service message
        SBAS3_LOG_TYPE = 987,	// Fast correction slots 13-25
        SBAS32_LOG_TYPE = 988,	// CDGPS Fast Corrections slots 0-10
        SBAS33_LOG_TYPE = 989,	// CDGPS Fast Corrections slots 11-21
        SBAS34_LOG_TYPE = 990,	// CDGPS Fast Corrections slots 22-32
        SBAS35_LOG_TYPE = 991,	// CDGPS Fast Corrections slots 32-43
        SBAS4_LOG_TYPE = 992,	// Fast correction slots 26-38
        SBAS45_LOG_TYPE = 993,	// CDGPS Slow Corrections
        SBAS5_LOG_TYPE = 994,	// Fast corrections slots 39-50
        SBAS6_LOG_TYPE = 995,	// Integrity Message
        SBAS7_LOG_TYPE = 996,	// Fast Correction Degradation
        SBAS9_LOG_TYPE = 997,	// Geo Nav Message
        SBASCORR_LOG_TYPE = 998,	// SBAS range corrections used
        LBANDTRACKSTAT_LOG_TYPE = 1201,	// L-Band Tracking Status
        SATVIS2_LOG_TYPE = 1043,	// Satellite visibility
        MASTERPOS_LOG_TYPE = 1051,	// Displays the master position with the ALIGN feature
        ROVERPOS_LOG_TYPE = 1052,	// Displays the rover position with the ALIGN feature
        RAWCNAVFRAME_LOG_TYPE = 1066,	// Raw L2C frame data
        MARK3TIME_LOG_TYPE = 1075,	// Position at time of mark3 input event
        MARK4TIME_LOG_TYPE = 1076,	// Position at time of mark4 input event
        MARK1COUNT_LOG_TYPE = 1093,	// Count for the Mark1 input
        MARK2COUNT_LOG_TYPE = 1094,	// Count for the Mark2 input
        MARK3COUNT_LOG_TYPE = 1095,	// Count for the Mark3 input
        MARK4COUNT_LOG_TYPE = 1096,	// Count for the Mark4 input
        GALALMANAC_LOG_TYPE = 1120,	// Decoded Galileo almanac parameters from Galileo navigation messages
        GALCLOCK_LOG_TYPE = 1121,	// Galileo time information
        GALEPHEMERIS_LOG_TYPE = 1122,	// Galileo ephemeris information is available through the GALEPHEMERIS log
        GALIONO_LOG_TYPE = 1127,	// Decoded Galileo ionospheric corrections
        LOGFILESTATUS_LOG_TYPE = 1146,	// Current state of file and recording
        CHANCONFIGLIST_LOG_TYPE = 1148,	// Channel configuration list
        PSRSATS_LOG_TYPE = 1162,	// Satellites used in PSRPOS solution
        PSRDOP2_LOG_TYPE = 1163,	// Pseudorange least squares DOP
        RTKDOP2_LOG_TYPE = 1172,	// Values from the RTK Fast Filter
        RTKSATS_LOG_TYPE = 1174,	// Satellites used in RTKPOS solution
        MATCHEDSATS_LOG_TYPE = 1176,	// Lists the used and unused satellites for the corresponding MATCHEDPOS solution
        BESTSATS_LOG_TYPE = 1194,	// Satellites used in BESTPOS
        OMNIHPSATS_LOG_TYPE = 1197,	// Satellites used in the OMNIHPPOS solution
        PASSETH1_LOG_TYPE = 1209,	// Pass through log
        PDPSATS_LOG_TYPE = 1234,	// Satellites used in PDPPOS solution
        SOFTLOADSTATUS_LOG_TYPE = 1235,	// Describes the status of the SoftLoad process
        PASSICOM1_LOG_TYPE = 1250,	// Pass through log
        PASSICOM2_LOG_TYPE = 1251,	// Pass through log
        PASSICOM3_LOG_TYPE = 1252,	// Pass through log
        PASSNCOM1_LOG_TYPE = 1253,	// Pass through log
        PASSNCOM2_LOG_TYPE = 1254,	// Pass through log
        PASSNCOM3_LOG_TYPE = 1255,	// Pass through log
        RANGECMP2_LOG_TYPE = 1273,	// RANGE data compressed to handle more channels and types
        RAIMSTATUS_LOG_TYPE = 1286,	// RAIM status
        ETHSTATUS_LOG_TYPE = 1288,	// Current Ethernet status
        IPSTATUS_LOG_TYPE = 1289,	// Current network configuration status
        GALINAVEPHEMERIS_LOG_TYPE = 1309,	// Decoded Galileo INAV ephemeris
        GALFNAVEPHEMERIS_LOG_TYPE = 1301,	// Decoded Galileo FNAV ephemeris
        ALIGNBSLNXYZ_LOG_TYPE = 1314,	// Outputs the RTK quality XYZ baselines from ALIGN
        ALIGNBSLNENU_LOG_TYPE = 1315,	// Outputs the RTK quality ENU baselines from ALIGN
        HEADINGSATS_LOG_TYPE = 1316,	// Outputs the satellite information from ALIGN filter
        REFSTATIONINFO_LOG_TYPE = 1325,	// Reference station position and health information
        MODELFEATURES_LOG_TYPE = 1329,	// States features available for current loaded model
        QZSSRAWEPHEM_LOG_TYPE = 1330,	// Contains the raw binary information for subframes one, two and three from the satellite with the parity information removed
        QZSSRAWSUBFRAME_LOG_TYPE = 1330,	// A raw QZSS subframe is 300 bits in total, includes the parity bits which are interspersed with the raw data ten times, in six bit chunks, for a total of 60 parity bits 
        ALIGNDOP_LOG_TYPE = 1332,	// Outputs the DOP computed using the satellites used in solution
        HEADING2_LOG_TYPE = 1335,	// Outputs same information as HEADING log with an additional Rover ID field
        QZSSEPHEMERIS_LOG_TYPE = 1336,	// Single set of QZSS ephemeris parameters
        RTCAOBS3_LOG_TYPE = 1340,	// Proprietary message that carries dual-frequency GPS and GLO measurements and is used in ALIGN. Also carries SBAS measurements if the Master receiver is single-frequency (L1-only) receiver to enable SBAS-ALIGN at the L1-only ALIGN Rover
        PASSTHROUGH_LOG_TYPE = 1342,	// Outputs pass-through data from all receiver ports
        SOURCETABLE_LOG_TYPE = 1344,	// Outputs the NTRIP source table entries from the NTRIPCASTER set by the NTRIPSOURCETABLE command
        QZSSRAWALMANAC_LOG_TYPE = 1345,	// Contains the undecoded almanac subframes as received from the QZSS satellite
        QZSSALMANAC_LOG_TYPE = 1346,	// Contains the decoded almanac parameters as received from the satellite with the parity information removed and appropriate scaling applied
        QZSSIONUTC_LOG_TYPE = 1347,	// Ionospheric Model parameters (ION) and the Universal Time Coordinated parameters (UTC) for QZSS are provided
        AUTHCODES_LOG_TYPE = 1348,	// Contains all authorization codes (auth codes) entered into the system since the last complete firmware reload
        PASSCOM4_LOG_TYPE = 1384,	// Pass through log
        PROFILEINFO_LOG_TYPE = 1412,	// Outputs a list of Profiles
        GALFNAVRAWPAGE_LOG_TYPE = 1413,	// Contains the raw Galileo F/Nav page data
        GALINAVRAWWORD_LOG_TYPE = 1414,	// Contains the raw Galileo I/Nav word data
        SBASALMANAC_LOG_TYPE = 1425,	// A collection of all current SBAS almanacs decoded by the receiver
        SATXYZ2_LOG_TYPE = 1451,	// Combined with a RANGE log, this data set contains the decoded satellite information necessary to compute the solution
        PPPPOS_LOG_TYPE = 1538,	// PPP filter position
        PPPSATS_LOG_TYPE = 1541,	// Satellites used in the PPPPOS solution
        PASSCOM5_LOG_TYPE = 1576,	// Pass through log
        PASSCOM6_LOG_TYPE = 1577,	// Pass through log
        BDSALMANAC_LOG_TYPE = 1584,	// Decoded almanac parameters as received from the satellite, with the parity information removed and appropriate scaling applied
        BDSIONO_LOG_TYPE = 1590,	// Contains the Klobuchar ionosphere model parameters transmitted by the BeiDou satellites
        BDSCLOCK_LOG_TYPE = 1607,	// Time parameters transmitted by the BeiDou satellites
        BLUETOOTHSTATUS_LOG_TYPE = 1608,	// Bluetooth radio module status
        WIFICLISTATUS_LOG_TYPE = 1613,	// Wi-Fi client connection status
        WIFICLISCANRESULTS_LOG_TYPE = 1616,	// Wi-Fi AP scan results
        NOVATELXOBS_LOG_TYPE = 1618,	// NovAtel proprietary RTK correction
        NOVATELXREF_LOG_TYPE = 1620,	// NovAtel proprietary reference station message for use in ALIGN
        WIFIAPSTATUS_LOG_TYPE = 1666,	// Wi-Fi Access Point Status
        IPSTATS_LOG_TYPE = 1669,	// IP statistics
        CELLULARSTATUS_LOG_TYPE = 1685,	// Cellular modem and network status information
        CELLULARINFO_LOG_TYPE = 1686,	// Cellular modem and network information
        BDSRAWNAVSUBFRAME_LOG_TYPE = 1695,	// Log contains single set of BDS ephemeris parameters
        BDSEPHEMERIS_LOG_TYPE = 1696,	// A single set of BDS ephemeris parameters
        HEADINGRATE_LOG_TYPE = 1698,	// Provides rate of change for the heading parameters
        PASSCOM7_LOG_TYPE = 1701,	// Pass through log (ProPak6 only via expansion cable)
        PASSCOM8_LOG_TYPE = 1702,	// Pass through log (ProPak6 only via expansion cable)
        PASSCOM9_LOG_TYPE = 1703,	// Pass through log (ProPak6 only via expansion cable)
        PASSCOM10_LOG_TYPE = 1704,	// Pass through log (ProPak6 only via expansion cable)
        LBANDBEAMTABLE_LOG_TYPE = 1718,	// List of L-Band Beams
        TERRASTARINFO_LOG_TYPE = 1719,	// TerraStar Subscription Information
        VERIPOSINFO_LOG_TYPE = 1728,	// Veripos Subscription Information
        TERRASTARSTATUS_LOG_TYPE = 1729,	// TerraStar Decoder and Subscription Status
        VERIPOSSTATUS_LOG_TYPE = 1730,	// Veripos Decoder and Subscription Status
        MARK3POS_LOG_TYPE = 1738,	// Position at time of Mark3 input event
        MARK4POS_LOG_TYPE = 1739,	// Position at time of Mark4 input event

        //CMR FORMAT LOGS
        CMROBS_LOG_TYPE = 103,	// Base station satellite observation information
        CMRREF_LOG_TYPE = 105,	// Base station position information
        CMRDESC_LOG_TYPE = 310,	// Base station description information
        CMRPLUS_LOG_TYPE = 717,	// Base station position information (low rate)
        CMRGLOOBS_LOG_TYPE = 882,	// CMR Type 3 GLONASS observations

        //RTCA FORMAT LOGS
        RTCAOBS_LOG_TYPE = 6,	// Type 7 Base Station observations
        RTCA1_LOG_TYPE = 10,	// Type 1 Differential GPS corrections
        RTCAREF_LOG_TYPE = 11,	// Type 7 Base Station parameters
        RTCAEPHEM_LOG_TYPE = 347,	// Type 7 Ephemeris and time information
        RTCAOBS2_LOG_TYPE = 805,	// Type 7 Base Station observations 2
        RTCAREFEXT_LOG_TYPE = 1049,	// Type 7 Extended Base Station parameters

        //RTCM FORMAT LOGS
        RTCM1_LOG_TYPE = 107,	// Type 1 Differential GPS corrections
        RTCM59_LOG_TYPE = 116,	// Type 59N-0 NovAtel Proprietary: RT20 differential
        RTCM3_LOG_TYPE = 117,	// Type 3 Base Station parameters
        RTCM22_LOG_TYPE = 118,	// Type 22 Extended Base Station parameters
        RTCM16_LOG_TYPE = 129,	// Type16 Special message
        RTCM16T_LOG_TYPE = 131,	// Type16T Special text message
        RTCM1819_LOG_TYPE = 260,	// Type18 and Type 19 raw measurements
        RTCM9_LOG_TYPE = 275,	// Type 9 Partial Differential GPS Corrections
        RTCM15_LOG_TYPE = 307,	// Type 15 Ionospheric Corrections
        RTCM2021_LOG_TYPE = 374,	// Type 20 and Type 21 Measurement Corrections
        RTCM23_LOG_TYPE = 665,	// Type 22 Extended Base Station parameters
        RTCM24_LOG_TYPE = 667,	// Type 23 Antenna Type Definition
        RTCM31_LOG_TYPE = 864,	// Type 31 Differential GLONASS Corrections
        RTCM32_LOG_TYPE = 873,	// Type 32 GLONASS Base Station parameters
        RTCM36_LOG_TYPE = 875,	// Type 36 Special Message
        RTCM36T_LOG_TYPE = 877,	// Type 36T Special Text Message
        RTCM59GLO_LOG_TYPE = 903,	// NovAtel proprietary GLONASS differential NovAtel proprietary GLONASS differential
        RTCMOMNI1_LOG_TYPE = 957,	// RTCM1 from OmniSTAR

        //RTCMV3 FORMAT LOGS
        RTCM1001_LOG_TYPE = 772,	// L1-Only GPS RTK Observables
        RTCM1002_LOG_TYPE = 774,	// Extended L1-Only GPS RTK Observables
        RTCM1003_LOG_TYPE = 776,	// L1/L2 GPS RTK Observables
        RTCM1004_LOG_TYPE = 770,	// Extended L1/L2 GPS RTK Observables
        RTCM1005_LOG_TYPE = 765,	// RTK Base Station ARP
        RTCM1006_LOG_TYPE = 768,	// RTK Base Station ARP with Antenna Height
        RTCM1007_LOG_TYPE = 852,	// Extended Antenna Descriptor and Setup
        RTCM1008_LOG_TYPE = 854,	// Extended Antenna Reference Station Description and Serial Number
        RTCM1009_LOG_TYPE = 885,	// GLONASS L1-Only RTK
        RTCM1010_LOG_TYPE = 887,	// Extended GLONASS L1-Only RTK
        RTCM1011_LOG_TYPE = 889,	// GLONASS L1/L2 RTK
        RTCM1012_LOG_TYPE = 891,	// Extended GLONASS L1/L2 RTK
        RTCM1019_LOG_TYPE = 893,	// GPS Ephemerides
        RTCM1020_LOG_TYPE = 895,	// GLONASS Ephemerides
        RTCM1033_LOG_TYPE = 1097,	// Receiver and antenna descriptors
        RTCM1071_LOG_TYPE = 1472,	// MSM1, GPS Code Measurements
        RTCM1072_LOG_TYPE = 1473,	// MSM2, GPS Phase Measurements
        RTCM1073_LOG_TYPE = 1474,	// MSM3, GPS Code and Phase Measurements
        RTCM1074_LOG_TYPE = 1475,	// MSM4, GPS Code, Phase and CNR Measurements
        RTCM1075_LOG_TYPE = 1476,	// MSM5, GPS Code, Phase, CNR and Doppler Measurements
        RTCM1076_LOG_TYPE = 1477,	// MSM6, Extended GPS Code, Phase and CNR Measurements
        RTCM1077_LOG_TYPE = 1478,	// MSM7, Extended GPS Code, Phase, CNR and Doppler Measurements
        RTCM1081_LOG_TYPE = 1479,	// MSM1, GLONASS Code Measurements
        RTCM1082_LOG_TYPE = 1480,	// MSM2, GLONASS Phase Measurements
        RTCM1083_LOG_TYPE = 1481,	// MSM3, GLONASS Code and Phase Measurements
        RTCM1084_LOG_TYPE = 1482,	// MSM4, GLONASS Code, Phase and CNR Measurements
        RTCM1085_LOG_TYPE = 1483,	// MSM5, GLONASS Code, Phase, CNR and Doppler Measurements
        RTCM1086_LOG_TYPE = 1484,	// MSM6, Extended GLONASS Code, Phase and CNR Measurements
        RTCM1087_LOG_TYPE = 1485,	// MSM7, Extended GLONASS Code, Phase, CNR and Doppler Measurements
        RTCM1091_LOG_TYPE = 1486,	// MSM1, Galileo Code Measurements
        RTCM1092_LOG_TYPE = 1487,	// MSM2, Galileo Phase Measurements
        RTCM1093_LOG_TYPE = 1488,	// MSM3, Galileo Code and Phase Measurements
        RTCM1094_LOG_TYPE = 1489,	// MSM4, Galileo Code, Phase and CNR Measurements
        RTCM1095_LOG_TYPE = 1490,	// MSM5, Galileo Code, Phase, CNR and Doppler Measurements
        RTCM1096_LOG_TYPE = 1491,	// MSM6, Extended Galileo Code, Phase and CNR Measurements
        RTCM1097_LOG_TYPE = 1492,	// MSM7, Extended Galileo Code, Phase, CNR and Doppler Measurements
        RTCM1111_LOG_TYPE = 1648,	// MSM1, QZSS Code Measurements
        RTCM1112_LOG_TYPE = 1649,	// MSM2, QZSS Phase Measurements
        RTCM1113_LOG_TYPE = 1650,	// MSM3, QZSS Code and Phase Measurements
        RTCM1114_LOG_TYPE = 1651,	// MSM4, QZSS Code, Phase and CNR Measurements
        RTCM1115_LOG_TYPE = 1652,	// MSM5, QZSS Code, Phase, CNR and Doppler Measurements
        RTCM1116_LOG_TYPE = 1653,	// MSM6, Extended QZSS Code, Phase and CNR Measurements
        RTCM1117_LOG_TYPE = 1654,	// MSM7, Extended QZSS Code, Phase, CNR and Doppler Measurements
        RTCM1121_LOG_TYPE = 1592,	// MSM1, BeiDou Code Measurements
        RTCM1122_LOG_TYPE = 1593,	// MSM2, BeiDou Phase Measurements
        RTCM1123_LOG_TYPE = 1594,	// MSM3, BeiDou Code and Phase Measurements
        RTCM1124_LOG_TYPE = 1595,	// MSM4, BeiDou Code, Phase and CNR Measurements
        RTCM1125_LOG_TYPE = 1596,	// MSM5, BeiDou Code, Phase, CNR and Doppler Measurements
        RTCM1126_LOG_TYPE = 1597,	// MSM6, Extended BeiDou Code, Phase and CNR Measurements
        RTCM1127_LOG_TYPE = 1598,	// MSM7, Extended BeiDou Code, Phase, CNR and Doppler Measurements

        //NMEA Format Data Logs
        GPALM_LOG_TYPE = 217,	// Almanac Data
        GPGGA_LOG_TYPE = 218,	// GPS Fix Data and Undulation
        GPGLL_LOG_TYPE = 219,	// Geographic Position - latitude/longitude
        GPGRS_LOG_TYPE = 220,	// GPS Range Residuals for Each Satellite
        GPGSA_LOG_TYPE = 221,	// GPS DOP and Active Satellites
        GPGST_LOG_TYPE = 222,	// Pseudorange Measurement Noise Statistics
        GPGSV_LOG_TYPE = 223,	// GPS Satellites in View
        GPRMB_LOG_TYPE = 224,	// Generic Navigation Information
        GPRMC_LOG_TYPE = 225,	// GPS Specific Information
        GPVTG_LOG_TYPE = 226,	// Track Made Good and Ground Speed
        GPZDA_LOG_TYPE = 227,	// UTC Time and Date
        GPGGARTK_LOG_TYPE = 259,	// GPS Fix Data with Extra Precision
        GPGGALONG_LOG_TYPE = 521,	// GPS Fix Data, Extra Precision and Undulation
        GLMLA_LOG_TYPE = 859,	// NMEA GLONASS Almanac Data
        GPHDT_LOG_TYPE = 1045,	// Heading in Degrees True
    };
}

#endif
