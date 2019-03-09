/*
 This file is from https://github.com/GAVLab/ublox.git

 * \author  David Hodo <david.hodo@gmail.com>
 * \author  Chris Collins <cnc0003@tigermail.auburn.edu>
 *
 * The MIT License
 *
 * Copyright (c) 2012 IS4S / Auburn University
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
 */

// uBlox LEA-6T Data Structures
#ifndef UBLOXSTRUCTURES_H
#define UBLOXSTRUCTURES_H

#include "stdint.h"

namespace ublox {

#define MAX_NOUT_SIZE      (5000)   // Maximum size of a NovAtel log buffer (ALMANAC logs are big!)
                    // find MAX_NOUT_SIZE for ublox (ask Scott how he go this one for Novatel)

#define MAXCHAN		50  // Maximum number of signal channels
#define MAX_SAT     33  // maximum number of prns - max prn is 32 plus prn 0 is 33

// define macro to pack structures correctly with both GCC and MSVC compilers
#ifdef _MSC_VER // using MSVC
	#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#else
	#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

//! Header prepended to ubx binary messages
#define HDR_CHKSM_LENGTH 8 //(includes "sync1 sync2 classid msgid length checksum")
#define UBX_SYNC_BYTE_1 0xB5
#define UBX_SYNC_BYTE_2 0x62
    
//! UBX Protocol Class/Message ID's
#define MSG_CLASS_ACK 0X05
    #define MSG_ID_ACK_ACK 0x01
    #define MSG_ID_ACK_NAK 0x00
#define MSG_CLASS_AID 0x0B
    #define MSG_ID_AID_ALM 0x30
    #define MSG_ID_AID_ALPSRV 0X32
    #define MSG_ID_AID_ALP 0x50
    #define MSG_ID_AID_AOP 0x33
    #define MSG_ID_AID_DATA 0x10
    #define MSG_ID_AID_EPH 0x31
    #define MSG_ID_AID_HUI 0x02
    #define MSG_ID_AID_INI 0x01
    #define MSG_ID_AID_REQ 0x00
#define MSG_CLASS_CFG 0x06
    #define MSG_ID_CFG_ANT 0X13
    #define MSG_ID_CFG_CNFGR 0x09
    #define MSG_ID_CFG_DAT 0x06
    #define MSG_ID_CFG_EKF 0x12
    #define MSG_ID_CFG_ESFGWT 0x29
    #define MSG_ID_CFG_FXN 0x0E
    #define MSG_ID_CFG_ITFM 0x39
    #define MSG_ID_CFG_MSG 0x01
    #define MSG_ID_CFG_NAV5 0x24
    #define MSG_ID_CFG_NAVX5 0x23
    #define MSG_ID_CFG_NMEA 0x17
    #define MSG_ID_CFG_NVS 0x22
    #define MSG_ID_CFG_PM2 0x3B
    #define MSG_ID_CFG_PM 0x32
    #define MSG_ID_CFG_PRT 0x00
    #define MSG_ID_CFG_RATE 0x08
    #define MSG_ID_CFG_RINV 0x34
    #define MSG_ID_CFG_RST 0x04
    #define MSG_ID_CFG_RXM 0x11
    #define MSG_ID_CFG_SBAS 0x16
    #define MSG_ID_CFG_TMODE2 0x3D
    #define MSG_ID_CFG_TMODE 0x1D
    #define MSG_ID_CFG_TP5 0x31
    #define MSG_ID_CFG_TP 0x07
    #define MSG_ID_CFG_USB 0x1B
#define MSG_CLASS_ESF 0x10
    #define MSG_ID_ESF_MEAS 0x02
    #define MSG_ID_ESF_STATUS 0x10
#define MSG_CLASS_INF 0x04
    #define MSG_ID_INF_DEBUG 0x04
    #define MSG_ID_INF_ERROR 0x00
    #define MSG_ID_INF_NOTICE 0x02
    #define MSG_ID_INF_TEST 0x03
    #define MSG_ID_INF_WARNING 0x01
#define MSG_CLASS_MON 0x0A
    #define MSG_ID_MON_HW2 0x0B
    #define MSG_ID_MON_HW 0x09
    #define MSG_ID_MON_IO 0x02
    #define MSG_ID_MON_MSGPP 0x06
    #define MSG_ID_MON_RXBUF 0x07
    #define MSG_ID_MON_RXR 0x21
    #define MSG_ID_MON_TXBUF 0X08
    #define MSG_ID_MON_VER 0x04
#define MSG_CLASS_NAV 0x01
    #define MSG_ID_NAV_AOPSTATUS 0x60
    #define MSG_ID_NAV_CLOCK 0x22
    #define MSG_ID_NAV_DGPS 0x31
    #define MSG_ID_NAV_DOP 0x04
    #define MSG_ID_NAV_EKFSTATUS 0x40
    #define MSG_ID_NAV_POSECEF 0x01
    #define MSG_ID_NAV_POSLLH 0x02
    #define MSG_ID_NAV_SBAS 0x32
    #define MSG_ID_NAV_SOL 0x06
    #define MSG_ID_NAV_STATUS 0x03
    #define MSG_ID_NAV_SVINFO 0x30
    #define MSG_ID_NAV_TIMEGPS 0x20
    #define MSG_ID_NAV_TIMEUTC 0x21
    #define MSG_ID_NAV_VELECEF 0x11
    #define MSG_ID_NAV_VELNED 0x12
#define MSG_CLASS_RXM 0x02
    #define MSG_ID_RXM_ALM 0x30
    #define MSG_ID_RXM_EPH 0x31
    #define MSG_ID_RXM_PMREQ 0x41
    #define MSG_ID_RXM_RAW 0x10
    #define MSG_ID_RXM_SFRB 0x11
    #define MSG_ID_RXM_SVSI 0x20
#define MSG_CLASS_TIM 0x0D
    #define MSG_ID_TIM_SVIN 0x04
    #define MSG_ID_TIM_TM2 0x03
    #define MSG_ID_TIM_TP 0x01
    #define MSG_ID_TIM_VRFY 0x06


PACK(
    struct UbloxHeader {
        uint8_t sync1;   //!< start of packet first byte (0xB5)
        uint8_t sync2;   //!< start of packet second byte (0x62)
        uint8_t message_class; //!< Class that defines basic subset of message (NAV, RXM, etc.)
        uint8_t message_id;		//!< Message ID
        uint16_t payload_length; //!< length of the payload data, excluding header and checksum
});


///////////////////////////////////////////////////////////
// Configuration Messages
///////////////////////////////////////////////////////////
/*!
 * CFM-MSG Message Structure
 * This message requests a specifiable message at a given rate.
 * ID: 0x06  0x01 Payload Length=3 bytes
 */
PACK(
    struct CfgMsgRate {
        UbloxHeader header;		//!< Ublox header
        uint8_t message_class;  //!< class of message to request
        uint8_t message_id;     //!< id of message to request
        uint8_t rate;           //!< rate message will be sent
        uint8_t checksum[2];
});

/*!
 * CFM-MSG Message Structure
 * This message requests a message once.
 * ID: 0x06  0x01 Payload Length=2 bytes
 */
 PACK(
    struct CfgMsg {
        UbloxHeader header;		//!< Ublox header
        uint8_t message_class;  //!< class of message to request
        uint8_t message_id;     //!< id of message to request
        uint8_t checksum[2];
});

/*!
 * CFG-CFG Message Structure
 * This message clears, saves, or loads novalitle memory.
 * Set masks to 0x061F to clear, save, or load all values.
 * ID: 0x06  0x09 Payload Length=12 bytes
 */
PACK(
    struct CfgCfg {
        UbloxHeader header;		//!< Ublox header
        uint32_t clear_mask;  //!< clear mask
        uint32_t save_mask;     //!< save mask
        uint32_t load_mask;           //!< load mask
        uint8_t checksum[2];      //!< Checksum
});

/*!
 * CFM-RST Message Structure
 * This message allows a receiver to be reset.
 * ID: 0x06  0x04 Payload Length=4 bytes
 */
 PACK(
    struct CfgRst {
        UbloxHeader header;		//!< Ublox header
        uint16_t nav_bbr_mask;  //!< Nav data to clear: 0x0000 = hot start, 0x0001 = warm start, 0xFFFF=cold start
        uint8_t  reset_mode;     //!< Reset mode
        uint8_t  reserved;       //!< reserved
        uint8_t checksum[2];
});


/*!
 * CFM-PRT Message Structure
 * This message configures a USART or USB port.
 * Use to specify input/output protocols to use
 * ID: 0x06  0x00 Payload Length=20 bytes
 */
PACK(
    struct CfgPrt {
        UbloxHeader header;		//!< Ublox header
        uint8_t port_id; //!< port identifier (0 or 1 for USART or 3 for USB)
        uint8_t reserved; //!< reserved
        uint16_t tx_ready; //!< transmit ready status
        uint32_t reserved2; //!< reserved
        uint32_t reserved3; //!< reserved
        uint16_t input_mask; //!< input protocol mask
        uint16_t output_mask; //!< output protocol mask
        uint16_t reserved4; //!< reserved
        uint16_t reserved5; //!< reserved
        uint8_t checksum[2];
});

/*!
* CFG-NAV5 Message Structure
* This message configures Navigation algorithm
* parameters.
* ID: 0x06  0x24 Payload Length=36 bytes
*/
PACK(
    struct CfgNav5 {
        UbloxHeader header;		//!< Ublox header
        uint16_t mask; //!< parameters bitmask (only masked params applied)
        uint8_t dynamic_model; //!< dynamic platform
        uint8_t fix_mode; //!< positioning fix mode
        int32_t fixed_altitude; //!< (scale .01) (m)
        uint32_t fixed_altitude_variance; //!< (scale .0001) (m^2)
        int8_t min_elevation; //!< (deg)
        uint8_t dead_reckoning_limit; //!< max time to perform DR w/out GPS (sec)
        uint16_t pdop; //!< (scale .1)
        uint16_t tdop; //!< (scale .1)
        uint16_t pos_accuracy_mask; //!< (m)
        uint16_t time_accuracy_mask; //!< (m)
        uint8_t static_hold_threshold; //!<
        uint8_t dgps_timeout; //!<
        uint32_t reserved2; //!< reserved (always set to zero)
        uint32_t reserved3; //!< reserved (always set to zero)
        uint32_t reserved4; //!< reserved (always set to zero)
        uint8_t checksum[2];
});

/*!
* CFG-SBAS Message Structure
* This message configures the SBAS receiver 
* subsystem (i.e. WAAS, EGNOS, MSAS).
* TYPE: POLL(empty payload) and COMMAND
* ID: 0x06  0x16 Payload Length=8 bytes
*/
PACK(
    struct CfgSbas {
        UbloxHeader header;     //!< Ublox header
        uint8_t mode;
        uint8_t usage;
        uint8_t maxSBAS;        //< (0-3)
        uint8_t scanmode2;      //< =0 for autoscan
        uint32_t scanmode1;     //< =0 for autoscan
        uint8_t checksum[2];
});

/////////////////////////////////////////////////////////////
// Navigation Messages
/////////////////////////////////////////////////////////////
/*!
 * NAV-STATUS Message Structure
 * This message contains gps fix type and ttff
 * ID: 0x01 0x03 Payload Length=16 bytes
 */
PACK(
    struct NavStatus {
        UbloxHeader header;
        uint32_t iTOW;      // Time of Week (ms)
        uint8_t fixtype;    // no fix=0x00, deadreckoning only=0x01, 2D=0x02, 3D=0x03, deadreck+GPS=0x04, time fix only=0x05, reserved=0x06..0xff
        uint8_t flags;
        uint8_t fixstat;
        uint8_t flags2;
        uint32_t ttff;      // TTFF (ms)
        uint32_t msss;      // Milliseconds since startup/reset
        uint8_t checksum[2];

});

PACK(
    struct NavStatusBody {
        uint32_t iTOW;      // Time of Week (ms)
        uint8_t fixtype;    // no fix=0x00, deadreckoning only=0x01, 2D=0x02, 3D=0x03, deadreck+GPS=0x04, time fix only=0x05, reserved=0x06..0xff
        uint8_t flags;
        uint8_t fixstat;
        uint8_t flags2;
        uint32_t ttff;      // TTFF (ms)
        uint32_t msss;      // Milliseconds since startup/reset
    }
);
/*!
* NAV-SOL Message Structure
* This message combines Position, velocity and
* time solution in ECEF, including accuracy figures.
* ID: 0x01  0x06  Payload Length=52 bytes
*/

#define NAVSOL_FLAG_GPSFIX_VALID 0b0001
#define NAVSOL_FLAG_DGPS_USED_FOR_FIX 0b0010
#define NAVSOL_FLAG_WEEK_NUM_VALID 0b0100
#define NAVSOL_FLAG_TOW_VALID 0b1000

PACK(
    struct NavSol{
        UbloxHeader header;
        uint32_t iTOW;
        int32_t fTOW;
        int16_t week;
        uint8_t gpsFix;
        int8_t flags;
        int32_t ecefX;
        int32_t ecefY;
        int32_t ecefZ;
        uint32_t pAcc;
        int32_t ecefVX;
        int32_t ecefVY;
        int32_t ecefVZ;
        uint32_t sAcc;
        uint16_t pDop;
        uint8_t reserved1;
        uint8_t numSV;
        uint32_t reserved2;
        uint8_t checksum[2];
});


/*!
* NAV-POSLLH Message Structure
* This message outputs the Geodetic position in
* the currently selected Ellipsoid. The default is
* the WGS84 Ellipsoid, but can be changed with the
* message CFG-DAT.
* ID: 0x01  0x02  Payload Length=28 bytes
*/
PACK(
    struct NavPosLLH{
        UbloxHeader header;		//!< Ublox header
        uint32_t iTOW;			//!< GPS millisecond time of week
        int32_t longitude_scaled; //!< longitude in degrees. Scaling 1e-7
        int32_t latitude_scaled; //!< latitude in degrees. Scaling 1e-7
        int32_t height;			 //!< height above ellipsoid [mm]
        int32_t height_mean_sea_level; //!< height above mean sea level [mm]
        uint32_t horizontal_accuracy; //!< horizontal accuracy estimate [mm]
        uint32_t vertical_accuracy;	//!< vertical accuracy estimate [mm]
        uint8_t checksum[2];
});

/*!
* NAV-VELNED Message Structure
* This message outputs the current 3D velocity
* in a north-east-down frame.
* ID: 0x01  0x12  Payload Length=36 bytes
*/
PACK(
    struct NavVelNed{
        UbloxHeader header;		//!< Ublox header
        uint32_t iTOW;
        int32_t velocity_north; //!< north velocity [cm/s]
        int32_t velocity_east; //!< east velocity [cm/s]
        int32_t velocity_down; //!< down velocity [cm/s]
        uint32_t speed; //!< 3D speed [cm/s]
        uint32_t ground_speed; //!< 2D (ground) speed [cm/s]
        int32_t heading_scaled; //!< heading [deg]. Scaling 1e-5
        uint32_t speed_accuracy; //!< speed accuracy estimate [cm/s]
        uint32_t heading_accuracy; //!< course/heading accuracy estimate [deg]. Scaling 1e-5
        uint8_t checksum[2];
});

/*!
* NAV-SVINFO Message Structure
* This message outputs info about SVs each 
* channel is tracking
* ID: 0x01  0x30  Payload Length= (8+12*NumChannels bytes)
*/
PACK(
    struct SVInfoReapBlock{
        uint8_t ch_num;     //!< Channel Number (255 if SV isn't assigned to channel)
        uint8_t svid;       // Satellite ID number
        uint8_t flags;      // bitfield (description of contents follows)
        uint8_t quality;    // signal quality indicator bitfield
        uint8_t cno;        // carrier to noise ratio (dbHz)
        int8_t elev;        // elevation (deg)
        int16_t azim;       // azimuth (deg)
        int32_t prRes;      // Psuedorange residual (centimeters)
});

PACK(
    struct NavSVInfo{
        UbloxHeader header;		//!< Ublox header
        uint32_t iTOW;  // GPS time of week (ms)
        uint8_t numch;  //! number of channels following
        uint8_t global_flags;   // Chip and Hardware Generation
        uint16_t reserved2;
        SVInfoReapBlock svinfo_reap[MAXCHAN]; // NOTE: TODO: True max needs to be confirmed
        uint8_t checksum[2];
});
// Description of flags bitfield
#define NAV_SVINFO_FLAGS_USED4NAV 0B00000001 // SV used in NAV sol
#define NAV_SVINFO_FLAGS_DGPS_AVAIL 0B00000010 // DGPS corr data available for SV
#define NAV_SVINFO_FLAGS_ORBIT_INFO_AVAIL 0B00000100 // Ephemeris of Almanac orbit info available for SV
#define NAV_SVINFO_FLAGS_EPHEMS_AVAIL 0B00001000 // Ephemeris orbit info available for SV
#define NAV_SVINFO_FLAGS_SV_UNHEALTHY 0B00010000 // SV unhealthy and not used
#define NAV_SVINFO_FLAGS_ALMPLUS_AVAIL 0B00100000 // Almanac Plus orbit info used
#define NAV_SVINFO_FLAGS_ASSNOW_AUTO 0B01000000 // AssistNow Autonomous orbit info used
#define NAV_SVINFO_FLAGS_PR_SMOOTHED 0B10000000 // Carrier Smoothed pseudorange used (PPP)


/*!
* NAV-TIMEGPS Message Structure
* This message outputs GPS Time information
* ID: 0x01  0x20  Payload Length= 16 bytes
*/
PACK(
    struct NavGPSTime{
        UbloxHeader header;
        uint32_t iTOW;  // GPS ms time of week
        int32_t ftow;   // fractional nanoseconds remainder
        int16_t week;   // GPS week
        int8_t leapsecs;// GPS UTC leap seconds
        uint8_t valid;  // validity flags
        uint32_t tacc;  // time accuracy measurement (nanosecs)
        uint8_t checksum[2];
});

PACK(
    struct NavGPSTimeBody {
        uint32_t iTOW;  // GPS ms time of week
        int32_t fTOW;   // fractional nanoseconds remainder
        int16_t week;   // GPS week
        int8_t leapsecs;// GPS UTC leap seconds
        uint8_t valid;  // validity flags
        uint32_t tacc;  // time accuracy measurement (nanosecs)
    }
);

/*!
* NAV-TIMEUTC Message Structure
* This message outputs UTC Time information
* ID: 0x01  0x21  Payload Length= 20 bytes
*/
PACK(
    struct NavUTCTime{
        UbloxHeader header;
        uint32_t iTOW;  // GPS time of week (msec)
        uint32_t tacc;  // time accuracy measurement
        int32_t nano;   // Nanoseconds of second
        uint16_t year;  // year
        uint8_t month;  // month
        uint8_t day;    // day
        uint8_t hour;   // hour
        uint8_t min;    // minute
        uint8_t sec;    // second
        uint8_t valid;  // validity flags
        uint8_t checksum[2];
});

/*!
* NAV-DOP Message Structure
* This message outputs various DOPs. All 
* DOP values are scaled by a factor of 100.
* Ex. If gdop contains a value 156, the true
* value is 1.56
* ID: 0x01  0x04  Payload Length= 18 bytes
*/
PACK(
    struct NavDOP{
        UbloxHeader header;
        uint32_t iTOW;  // GPS ms time of week (ms)
        uint16_t gdop;  // Geometric DOP
        uint16_t pdop;  // Position DOP
        uint16_t tdop;  // Time DOP
        uint16_t vdop;  // Vertical DOP
        uint16_t hdop;  // Horizontal DOP
        uint16_t ndop;  // Northing DOP
        uint16_t edop;  // Easting DOP
        uint8_t checksum[2];
});

/*!
* NAV-DGPS Message Structure
* This message outputs DGPS correction data as it
* has been applied to the current NAV Solution
* ID: 0x01  0x31  Payload Length= (16 + 12*numChannels bytes)
*/
PACK(
    struct NavDGPSReap{
        uint8_t svid;
        uint8_t flags;  // bitfield containing channel each sv is on and DGPS status
        uint16_t agecorr;   // age of latest correction data (ms)
        float prcorr;   // psuedorange correction   (m)
        float prrcorr;  // psuedorange rate correction (m/sec)
});

PACK(
    struct NavDGPS{
        UbloxHeader header;
        uint32_t iTOW;  // GPS ms time of week
        int32_t age;    // age of newest correction data (ms)
        int16_t baseID; // DGPS base station ID
        int16_t basehealth; // DGPS base station health
        uint8_t numchan;    // nomber of channels for which correction data is following
        uint8_t status; // DGPS correction type status
        uint16_t reserved;  // reserved
        NavDGPSReap nav_dgps_reap;  // repeated portion of NAV-DGPS message
        uint8_t checksum[2];
});

/*!
* NAV-CLOCK Message Structure
* This message outputs receiver clock information
* ID: 0x01  0x22  Payload Length= 20 bytes
*/
PACK(
    struct NavClock{
        UbloxHeader header;
        uint32_t iTOW;
        int32_t clkbias;    // clock bias in nanoseconds
        int32_t clkdrift;   // clock drift in ns/s
        uint32_t tacc;      // time accuracy estimate (ns)
        uint32_t facc;      // frequency accuracy estimate (ps/s)
        uint8_t checksum[2];
});


//////////////////////////////////////////////////////////////
// AIDING DATA MESSAGES
//////////////////////////////////////////////////////////////
/*!
 * AID-INI Message Structure
 * Reciever Position, Time, Clock Drift, Frequency
 * ID: 0x0B  0x01 Payload Length=48 bytes
 */
#define PAYLOAD_LENGTH_AID_INI 48
#define FULL_LENGTH_AID_INI 48+8
PACK(
    struct AidIni {
        UbloxHeader header;		//!< Ublox header
        int32_t ecefXorLat;  //!< ECEF x position or latitude [cm or deg*1e-7]
        int32_t ecefYorLon;  //!< ECEF y position or longitude [cm or deg*1e-7]
        int32_t ecefZorAlt;  //!< ECEF z position or altitude [cm]
        uint32_t position_accuracy; //!< position accuracy - std dev [cm]
        uint16_t time_configuration; //!< time configuration bit misk
        uint16_t week_number; //!< actual week number
        uint32_t time_of_week; //!< actual time of week [ms]
        int32_t time_of_week_ns; //!< fractional part of time of week [ns]
        uint32_t time_accuracy_ms; //!< time accuracy [ms]
        uint32_t time_accuracy_ns; //!< time accuracy [ns]
        int32_t clock_drift_or_freq; //!< clock drift or frequency [ns/s or Hz*1e-2]
        uint32_t clock_drift_or_freq_accuracy; //!< clock drift or frequency accuracy [ns/s or ppb]
        uint32_t flags; //!< bit field that determines contents of other fields
        uint8_t checksum[2];
});

PACK(
    struct AidIniBody {
        int32_t ecefXorLat;  //!< ECEF x position or latitude [cm or deg*1e-7]
        int32_t ecefYorLon;  //!< ECEF y position or longitude [cm or deg*1e-7]
        int32_t ecefZorAlt;  //!< ECEF z position or altitude [cm]
        uint32_t position_accuracy; //!< position accuracy - std dev [cm]
        uint16_t time_configuration; //!< time configuration bit misk
        uint16_t week_number; //!< actual week number
        uint32_t time_of_week; //!< actual time of week [ms]
        int32_t time_of_week_ns; //!< fractional part of time of week [ns]
        uint32_t time_accuracy_ms; //!< time accuracy [ms]
        uint32_t time_accuracy_ns; //!< time accuracy [ns]
        int32_t clock_drift_or_freq; //!< clock drift or frequency [ns/s or Hz*1e-2]
        uint32_t clock_drift_or_freq_accuracy; //!< clock drift or frequency accuracy [ns/s or ppb]
        uint32_t flags; //!< bit field that determines contents of other fields
    }
);

// defines for AidIni flags
#define AIDINI_FLAG_POSITION_VALID 0x01
#define AIDINI_FLAG_TIME_VALID 0x02
#define AIDINI_FLAG_CLOCK_DRIFT_VALID 0x04
#define AIDINI_FLAG_USE_TIME_PULSE 0X08
#define AIDINI_FLAG_CLOCK_FREQ_VALID 0x10
#define AIDINI_FLAG_USE_LLA 0x20
#define AIDINI_FLAG_ALTITUDE_INVALID 0X40
#define AIDINI_USE_PREV_TIME_PULSE 0X80

/*!
 * AID-HUI Message Structure
 * GPS Health, Ionospheric, and UTC Parameters
 * ID: 0x0B  0x02 Payload Length: 72
 */
#define PAYLOAD_LENGTH_AID_HUI 72
#define FULL_LENGTH_AID_HUI 72+8
PACK(
    struct AidHui{
        UbloxHeader header;
        uint32_t health;
        double a0;
        double a1;
        uint32_t tow;
        int16_t week;
        int16_t beforeleapsecs;
        int16_t nextleapsecweek;
        int16_t nextleapsec;
        int16_t afterleapsecs;
        int16_t spare;
        float kloba0;
        float kloba1;
        float kloba2;
        float kloba3;
        float klobb0;
        float klobb1;
        float klobb2;
        float klobb3;
        uint32_t flags;
        uint8_t checksum[2];
});
// defines for AID-HUI flags
#define AIDHUI_FLAG_HEALTH_VALID 0b001
#define AIDHUI_FLAG_UTC_VALID 0b010
#define AIDHUI_FLAG_KLOB_VALID 0b100

/*!
 * AID-EPH Message Structure
 * This message contains ephemeris for a satellite.
 * ID: 0x0B 0x31 Payload Length = (16) or (112) bytes
 */
#define PAYLOAD_LENGTH_AID_EPH_WITH_DATA 104
#define PAYLOAD_LENGTH_AID_EPH_NO_DATA 8
#define FULL_LENGTH_AID_EPH_WITH_DATA 104+8
#define FULL_LENGTH_AID_EPH_NO_DATA 8+8
PACK(
    struct EphemW{
        uint8_t byte[4];				// Each Word contains 4 bytes (4th is ignored)
});

PACK(
    struct EphemSF{
        //uint32_t W[8];				// Words 3-10 of Subframes
        EphemW W[8];
});	

PACK(
    struct EphemSV{					// Ephemeris for a Satellite
        UbloxHeader header;			// Header
        uint32_t svprn;				// Satellite Number
        uint32_t HOW;				// Hand Over Word
        EphemSF SF[3];				// Subframes
        uint8_t checksum[2];		// Checksum
});

PACK(
    struct Ephemerides{             // Holds EphemSV message for all SVs
        EphemSV ephemsv[MAX_SAT];
});

// Parsed Ephemeris Parameters for a SV - NOT FINISHED
PACK(
    struct ParsedEphemData {
        uint32_t prn;				//PRN number
        uint8_t tow;				//time stamp of subframe 0 (s)
        //uint8_t tow;				//time stamp of subframe 0 (s)
        unsigned long health;		//health status, defined in ICD-GPS-200
        unsigned long iode1;		//issue of ephemeris data 1
        unsigned long iode2;		//issue of ephemeris data 2
        unsigned long week;			//GPS week number
        unsigned long zweek;		//z count week number
        double toe;					//reference time for ephemeris (s)
        double majaxis;				//semi major axis (m)
        double dN;					//Mean motion difference (rad/s)
        double anrtime;				//mean anomoly reference time (rad)
        double ecc;					//eccentricity
        double omega;				//arguement of perigee (rad)
        double cuc;					//arugument of latitude - cos (rad)
        double cus;					//argument of latitude - sine (rad)
        double crc;					//orbit radius - cos (rad)
        double crs;					//orbit radius - sine (rad)
        double cic;					//inclination - cos (rad)
        double cis;					//inclination - sine (rad)
        double ia;					//inclination angle (rad)
        double dia;					//rate of inclination angle (rad/s)
        double wo;					//right ascension (rad)
        double dwo;					//rate of right ascension (rad/s)
        unsigned long iodc;			//issue of data clock
        double toc;					//SV clock correction term (s)
        double tgd;					//estimated group delay difference
        double af0;					//clock aiging parameter 0
        double af1;					//clock aiging parameter 1
        double af2;					//clock aiging parameter 2
//      yes_no spoof;			//anti spoofing on
        double cmot;				//corrected mean motion
        unsigned int ura;			//user range accuracy variance (value 0-15)
});

// Contains Ephemeris Parameters for all SVs
PACK(
    struct ParsedEphemeridesData{
        ParsedEphemData sv_eph_data[MAX_SAT];
});

/*!
 * AID-ALM Message Structure
 * This message contains GPS almanac data for a satellite
 * ID: 0x0B 0x30 Payload Length = (8) or (48) bytes
 */
#define PAYLOAD_LENGTH_AID_ALM_WITH_DATA 40
#define PAYLOAD_LENGTH_AID_ALM_NO_DATA 8
#define FULL_LENGTH_AID_ALM_WITH_DATA 40+8
#define FULL_LENGTH_AID_ALM_NO_DATA 8+8
PACK(
    struct AlmSV{
        UbloxHeader header;			// Header
        uint32_t svprn;				// Satellite Number
        uint32_t issue_week;                    // Issue date of Almanac
        uint32_t words[8];			// Words 3-10 of Almanac data for an SV
        uint8_t checksum[2];                    // Checksum
});

PACK(
    struct AlmanacNoPayload {
        uint32_t svprn;                         // Satellite Number
        uint32_t issue_week;                    // Issue date of Almanac
    }
);

PACK(
    struct AlmanacWithPayload {
        uint32_t svprn;                         // Satellite Number
        uint32_t issue_week;                    // Issue date of Almanac
        uint32_t words[8];                      // Words 3-10 of Almanac data for an SV
    }
);

// Holds Almanac data for all SVs
PACK(
    struct Almanac{
        AlmSV almsv[MAX_SAT];
});

/*!
 * RXM-RAW Message Structure
 * This message contains raw DGPS measurements data
 * ID: 0x02 0x10 Payload Length = (8 + 24*#SVs) bytes
 */
//#define RXMRAW_QUALITY_PR_DOP_GOOD 4 // Min value for pseudorange and doppler to be good    
//#define RXMRAW_QUALITY_PR_DOP_CP_GOOD 4 // Min value for pseudorange, doppler, and carrier phase to be good
PACK(
    struct RawMeasReap{
        double carrier_phase;           // cycles - Carrier Phase measurement
        double psuedorange;             // m - Psuedorange measurement
        float doppler;                  // Hz - Doppler Measurement
        uint8_t svid;                   // SV Number
        int8_t quality;                 // Nav Measurement Quality Indicator  -- (>=4 PR+DO OK) (>=5 PR+DO+CP OK) (<6 likel loss carrier lock)
        int8_t cno;                     // dbHz - Carrier to Noise Ratio
        uint8_t loss_of_lock_indicator; // Loss of Lock Indicator (RINEX Definition)
});

PACK(
    struct RawMeas{
        UbloxHeader header;
        int32_t iTow;   // ms - Time of Week
        int16_t week;   // weeks
        uint8_t numSV;  // # of SVs following
        uint8_t reserved;
        RawMeasReap rawmeasreap[MAX_SAT];
        uint8_t checksum[2];
});


/*!
 * RXM-SFRB Message Structure
 * This message contains the contents of a single subframe
 * w/ parity bits removed
 * ID: 0x02 0x11 Payload Length = (42) bytes
 */

PACK(
    struct SubframeData{
        UbloxHeader header;
        uint8_t chan;       // Channel Number
        uint8_t svid;       // Satellite ID Number
        int32_t words[10];  // Words of data
            /*
            Each word contains 24 bits of data (Bits 23 to 0).  The remaining 8
            bits are undefined.  The direction within the Word is that the higher
            order bits are received from the SV first.
            Example:
                The Preamble can be found in dwrd[0], at bit position 23 down to 16.
            */
        uint8_t checksum[2];
});


/*!
 * RXM-SVSI Message Structure
 * This message contains SV orbit knowledge for SVs
 * ID: 0x02 0x20 Payload Length = (8 + 6*#SVs) bytes
 */
PACK(
    struct SVStatusReap{
        uint8_t svid;       // Satellite ID
        uint8_t svflag;     // Information Flag
        int16_t azim;       // Azimuth
        int8_t elev;        // Elevation
        uint8_t age;        // Age of almanac and ephemeris

});

PACK(
    struct SVStatus{
        UbloxHeader header;
        int32_t iTow;       // ms - Time of Week
        int16_t week;       // weeks - GPS Week
        uint8_t numvis;     // Number of visible SVs
        uint8_t numSV;      // # of SVs following
        SVStatusReap svstatusreap[100]; // NOTE: TODO: Find the max repititions possible for this!! max thus far: (71)
        uint8_t checksum[2];
});

enum Message_ID
{
    CFG_PRT = 1536,                 // (ID 0x06 0x00) I/O Protocol Settings
    CFG_NAV5 = 1572,                // (ID 0x06 0x24) Navigation Algorithm Parameter Settings
    NAV_STATUS = 259,               // (ID 0x01 0x03) TTFF, GPS Fix type, time since startup/reset
    NAV_SOL = 262,                  // (ID 0x01 0x06) ECEF Pos,Vel, TOW, Accuracy,
    NAV_VELNED = 274,               // (ID 0x01 0x12) Vel (North, East, Down), Speed, Ground Speed
    NAV_POSLLH = 258,               // (ID 0x01 0x02) Pos (Lat,Long,Height)
    NAV_SVINFO = 304,               // (ID 0x01 0x30) Info on Channels and the SVs they're tracking
    NAV_GPSTIME = 288,              // (ID 0x01 0x20) GPS Time
    NAV_DGPS = 305,                 // (ID 0x01 0x31) Outputs correction data used for the nav solution
    NAV_DOP = 260,                  // (ID 0x01 0x04) Various Dilution of Precisions
    NAV_UTCTIME = 289,              // (ID 0x01 0x21) UTC Time
    NAV_CLK = 290,                  // (ID 0x01 0x22) Clock information
    AID_REQ = 2816,                 // (ID 0x0B 0x00) Receiver Requests Aiding data if not present at startup
    AID_EPH = 2865,					// (ID 0x0B 0x31) Ephemerides
    AID_ALM = 2864,					// (ID 0x0B 0x30) Almanac
    AID_HUI = 2818,                 // (ID 0x0B 0x02) GPS Health, Ionospheric, UTC
    AID_INI = 2817,                 // (ID 0x0B 0x01) Position, Time, Frequency, Clock Drift
    MON_VER = 2564,                 // (ID 0x0A 0x04) Reciever/Software/ROM Version
    RXM_RAW = 528,                  // (ID 0x02 0x10) Raw DGPS Data
    RXM_SFRB = 529,                 // (ID 0x02 0x11) GPS Subframe Data
    RXM_SVSI = 544,                 // (ID 0x02 0x20) SV Status Info
};

//typedef enum BINARY_LOG_TYPE BINARY_LOG_TYPE;
} // end namespace
#endif
