/** @file */

/* Note: Pragma packing varies across processors.  void* pointers will vary with processor! */

#ifndef DATA_SETS_H
#define DATA_SETS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*! Defines the 4 parts to the communications version. Major changes involve changes to the com manager. Minor changes involve additions to data structures */
// #define COMM_VERSION_CHAR0 1        // Major (in com_manager.h)
// #define COMM_VERSION_CHAR1 0
#define COMM_VERSION_CHAR2 2
#define COMM_VERSION_CHAR3 3			// Minor (in data_sets.h)

/*! Maximum number of messages that may be broadcast simultaneously */
#define MAX_NUM_BCAST_MSGS 10

/*! Maximum number of satellite channels */
#define MAX_NUM_SAT_CHANNELS 50

/*! Maximum length of device info manufacturer string (must be a multiple of 4) */
#define DEVINFO_MANUFACTURER_STRLEN 24

/*! Start of extended external ids - NEVER change this value */
#define DID_EXTERNAL_EXTENDED_START 8192

#if RTK_EMBEDDED

#include "rtklib_embedded.h"

#endif

// *****************************************
// ****** NEVER REORDER THESE VALUES! ******
// *****************************************

/*! Data identifiers */
enum eDataIDs
{
	/*! 0 : NULL (INVALID) */
	DID_NULL = 0,

	/*! 1 : (dev_info_t) Device information */
	DID_DEV_INFO,
	
	/*! 2: (imu_t) Inertial measurement unit data: calibrated gyro, accelerometer, magnetometer, and barometric pressure. */
	DID_IMU,

	/*! 3: (delta_theta_vel_t) Conning and sculling integral in body/IMU frame. Updated at IMU rate. */
	DID_DELTA_THETA_VEL,
	
	/*! 4 : (ins_1_t) Inertial navigation data with euler attitude and NED from reference LLA */
	DID_INS_1,

	/*! 5 : (ins_2_t) Inertial navigation data with quaternion attitude */
	DID_INS_2,

	/*! 6 : (gps_t) GPS data */
	DID_GPS,

	/*! 7 : (config_t) Configuration data */
	DID_CONFIG,
	
	/*! 8 : (ascii_msgs_t) Broadcast period for ASCII messages */
	DID_ASCII_BCAST_PERIOD,
	
	/*! 9 : (ins_misc_t) Other INS data */
	DID_INS_MISC,
	
	/*! 10 : (sys_params_t) System parameters */
	DID_SYS_PARAMS,
	
	/*! 11 : (sys_sensors_t) System sensor information (10) */
	DID_SYS_SENSORS,
	
	/*! 12 : (nvm_flash_cfg_t) Flash memory configuration */
	DID_FLASH_CONFIG,
	
	/*! 13 : (gps_rssi_t) GPS received signal strength indicator */
	DID_GPS_RSSI,
	
	/*! 14 : (gps_nav_poslla_t) GPS velocity data */
	DID_GPS_POS,
	
	/*! 15 : (gps_nav_velned_t) GPS velocity data */
	DID_GPS_VEL,
	
	/*! 16 : (io_t) I/O: Servos */
	DID_IO,

	/*! 17 : (io_servos_t) I/O: Servos Pulse Width Modulation (PWM) */
	DID_IO_SERVOS_PWM,

	/*! 18 : (io_servos_t) I/O: Servos Pulse Position Modulation (PPM), single wire servo pulse train */
	DID_IO_SERVOS_PPM,

	/*! 19 : (mag_cal_t) Magnetometer calibration */
	DID_MAGNETOMETER_CAL,
	
	/*! 20 : (ins_res_t) Resources */
	DID_INS_RESOURCES,
	
	/*! 21 : Differential GPS Correction */
	DID_DGPS_CORRECTION,

	/*! 22 : (Rtk data - packet parsing is done in RTKLib specific code) */
	DID_RTK,

    /*! 23 : End of non-extended external data identifiers */
    DID_EXTERNAL_NON_EXTENDED_END,

    /*! Reserved for the next external data identifier */
    DID_EXTERNAL_RESERVED = DID_EXTERNAL_EXTENDED_START,

    // adding a new data id? make sure to do the following:
    // modify data_sets.c for string and double flipping...
    // increment DID_COUNT down below in this file

    /*! This identifier marks the end of extended external data identifiers */
    // this MUST stay as the very last enum value
    DID_EXTERNAL_EXTENDED_END
};

/*! Number of external data identifiers available, if you ever add an external data id, be sure to increment */
#define DID_EXTERNAL_COUNT (DID_EXTERNAL_NON_EXTENDED_END + (DID_EXTERNAL_EXTENDED_END - DID_EXTERNAL_EXTENDED_START - 1))
#define DID_IS_EXTERNAL(v) ((v >= 0 && v < DID_EXTERNAL_NON_EXTENDED_END) || (v >= DID_EXTERNAL_EXTENDED_START && v < DID_EXTERNAL_EXTENDED_END))

// assumes that v is a valid external did
#define DID_FLATTEN_EXTERNAL(v) (v < DID_EXTERNAL_NON_EXTENDED_END ? v : (DID_EXTERNAL_NON_EXTENDED_END + (v - DID_EXTERNAL_EXTENDED_START)))

#ifdef USE_IS_INTERNAL
#	include "../../libs/IS_internal.h"
#endif

/*! Number of data identifiers available, if you ever add an internal or external data id, be sure to increment */
#ifndef DID_COUNT       
#	define DID_COUNT			52
#endif

/*! INS status flags */
enum eInsStatus
{
	/*! INS attitude is coarse aligned */
	INS_STATUS_ATT_ALIGNED_COARSE			= (int)0x00000001,
	/*! INS velocity is coarse aligned */
	INS_STATUS_VEL_ALIGNED_COARSE			= (int)0x00000002,
	/*! INS position is coarse aligned */
	INS_STATUS_POS_ALIGNED_COARSE			= (int)0x00000004,
	/*! INS coarse aligned mask */
	INS_STATUS_ALIGNED_COARSE_MASK			= (int)0x00000007,

	/*! INS attitude is aligned */
	INS_STATUS_ATT_ALIGNED					= (int)0x00000010,
	/*! INS velocity is aligned */
	INS_STATUS_VEL_ALIGNED					= (int)0x00000020,
	/*! INS position is aligned */
	INS_STATUS_POS_ALIGNED					= (int)0x00000040,
	/*! INS aligned mask */
	INS_STATUS_ALIGNED_MASK					= (int)0x00000070,
	/*! INS attitude is fine aligned */
	INS_STATUS_ATT_ALIGNED_FINE				= (int)0x00000080,
	/*! INS all fine aligned mask */
	INS_STATUS_FINE_ALIGNED_MASK			= (int)0x000000FF,

	/*! INS attitude is aligning from GPS */
	INS_STATUS_ATT_ALIGNING_GPS				= (int)0x00000100,
	/*! INS velocity is aligning */
	INS_STATUS_VEL_ALIGNING					= (int)0x00000200,
	/*! INS position is aligning */
	INS_STATUS_POS_ALIGNING					= (int)0x00000400,
	/*! INS attitude is aligning from Mag */
	INS_STATUS_ATT_ALIGNING_MAG				= (int)0x00000800,
	/*! INS aligning mask */
	INS_STATUS_ALIGNING_MASK				= (int)0x00000F00,

	/*! Reference (GPS) position is valid */
	INS_STATUS_REF_POS_VALID				= (int)0x00001000,
	/*! Reference (GPS) Velocity is valid */
	INS_STATUS_REF_VEL_VALID				= (int)0x00002000,
	/*! Reference (GPS) Acceleration is valid */
	INS_STATUS_REF_ACC_VALID				= (int)0x00004000,

	/*! INS accelerating in horizontal plane. */
	INS_STATUS_INS_ACC_2D					= (int)0x00010000,
	/*! Reference (GPS) accelerating in horizontal plane. */
	INS_STATUS_REF_ACC_2D					= (int)0x00020000,
	/*! Reference (GPS) moving in horizontal plane. */
	INS_STATUS_REF_VEL_2D					= (int)0x00040000,
	
	/*! Startup static aligning */
	INS_STATUS_STARTUP_STATIC_ALIGNING		= (int)0x00100000,
	/*! Startup dynamic alignment complete */
	INS_STATUS_STARTUP_DYNAMIC_ALIGNED		= (int)0x00200000,
	/*! Magnetometer calibration running */
	INS_STATUS_CALIBRATING_MAG				= (int)0x00400000,

	/*! INS PQR bias estimation running */
	INS_STATUS_BIAS_EST_PQR					= (int)0x01000000,
	/*! INS acceleration bias estimation running */
	INS_STATUS_BIAS_EST_ACC					= (int)0x02000000,
	/*! INS barometric altimeter bias estimation running */
	INS_STATUS_BIAS_EST_BARO				= (int)0x04000000,
	/*! INS bias estimation mask */
	INS_STATUS_BIAS_EST_MASK				= (int)0x07000000,
	/*! INS PQR bias estimation stable */
	INS_STATUS_BIAS_EST_PQR_STABLE			= (int)0x08000000,

	/*! INS has rotated while not translating/moving */
	INS_STATUS_STATIONARY_ROTATION			= (int)0x10000000,
	/*! General fault */
	INS_STATUS_GENERAL_FAULT				= (int)0x80000000,
};

/*! Hardware status flags */
enum eHardwareStatus
{
	/*! Gyro motion detected sigma */
	HDW_STATUS_MOTION_GYR_SIG				= (int)0x00000001,
	HDW_STATUS_MOTION_GYR_SIG_INV			= (int)0xFFFFFFFE,
	/*! Accelerometer motion detected sigma */
	HDW_STATUS_MOTION_ACC_SIG				= (int)0x00000002,
	HDW_STATUS_MOTION_ACC_SIG_INV			= (int)0xFFFFFFFD,
	/*! Gyro motion detected deviation */
	HDW_STATUS_MOTION_GYR_DEV				= (int)0x00000004,
	HDW_STATUS_MOTION_GYR_DEV_INV			= (int)0xFFFFFFFB,
	/*! Accelerometer motion detected deviation */
	HDW_STATUS_MOTION_ACC_DEV				= (int)0x00000008,
	HDW_STATUS_MOTION_ACC_DEV_INV			= (int)0xFFFFFFF7,
	/*! Unit is moving and NOT stationary */
	HDW_STATUS_MOTION_SIG_MASK				= (int)0x00000003,
	HDW_STATUS_MOTION_MASK					= (int)0x0000000F,

	/*! Sensor saturation on gyro 1 */
	HDW_STATUS_SATURATION_GYR1				= (int)0x00000100,
	/*! Sensor saturation on accelerometer 1 */
	HDW_STATUS_SATURATION_ACC1				= (int)0x00000200,
	/*! Sensor saturation on magnetometer 1 */
	HDW_STATUS_SATURATION_MAG1				= (int)0x00000400,
	/*! Sensor saturation on barometric pressure */
	HDW_STATUS_SATURATION_BARO				= (int)0x00000800,

	/*! Sensor saturation on gyro 2 */
	HDW_STATUS_SATURATION_GYR2				= (int)0x00001000,
	/*! Sensor saturation on accelerometer 2 */
	HDW_STATUS_SATURATION_ACC2				= (int)0x00002000,
	/*! Sensor saturation on magnetometer 2 */
	HDW_STATUS_SATURATION_MAG2				= (int)0x00004000,
	/*! Sensor saturation happened in past for MPU1 and MPU2 */
	HDW_STATUS_SATURATION_HISTORY			= (int)0x00008000,
	/*! Sensor saturation mask */
	HDW_STATUS_SATURATION_MASK				= (int)0x0000FF00,
	/*! Bitwise inverse of sensor saturation mask */
	HDW_STATUS_SATURATION_MASK_INV			= (int)0xFFFF00FF,
	/*! Sensor saturation offset */
	HDW_STATUS_SATURATION_OFFSET			= 8,

	/*! Communications Tx buffer limited */
	HDW_STATUS_ERR_COM_TX_LIMITED			= (int)0x00010000,
	/*! Communications Rx buffer overrun */
	HDW_STATUS_ERR_COM_RX_OVERRUN			= (int)0x00020000,
	/*! GPS Tx buffer limited */
	HDW_STATUS_ERR_GPS_TX_LIMITED			= (int)0x00040000,
	/*! GPS Tx buffer overrun */
	HDW_STATUS_ERR_GPS_RX_OVERRUN			= (int)0x00080000,

	/*! Automatic baudrate detection fault */
	HDW_STATUS_ERR_AUTOBAUD_FAULT			= (int)0x00100000,
	/*! Communications read fault */
	HDW_STATUS_ERR_COM_READ_FAULT			= (int)0x00200000,

	/*! Auto-baud negotiated */
	HDW_STATUS_AUTOBAUD_DETECTED			= (int)0x01000000,
	/*! Magnetometer Update */
	HDW_STATUS_MAGNETOMETER_UPDATE			= (int)0x02000000,
	/*! Barometer Update */
	HDW_STATUS_BAROMETER_UPDATE				= (int)0x04000000,
	/*! Magnetometer and Barometer Update */
	HDW_STATUS_MAG_BARO_UPDATE_MASK			= (int)0x06000000,
	/*! Magnetometer and Barometer Update Inverse */
	HDW_STATUS_MAG_BARO_UPDATE_MASK_INV		= (int)0xF9FFFFFF,

	/*! Watchdog reset fault */
	HDW_STATUS_FAULT_WATCHDOG_RESET			= (int)0x10000000,
	/*! Brownout (low system voltage) detection reset */
	HDW_STATUS_FAULT_BOD_RESET				= (int)0x20000000,
	/*! Power-on reset (from reset pin or software) */
	HDW_STATUS_FAULT_POR_RESET				= (int)0x40000000,
	/*! CPU error reset */
	HDW_STATUS_FAULT_CPU_ERR_RESET			= (int)0x80000000,
};

#if defined(AVR) || defined(ARM)
#pragma pack(1)
#else
#pragma pack(push, 1)
#endif

/*! (DID_DEV_INFO) Device information */
typedef struct
{
	/*! Reserved bits */
	uint32_t        reserved;

	/*! Serial number */
	uint32_t        serialNumber;

	/*! Hardware version */
	uint8_t         hardwareVer[4];

	/*! Firmware (software) version */
	uint8_t         firmwareVer[4];

	/*! Build number */
	uint32_t        buildNumber;

	/*! Communications protocol version */
	uint8_t         commVer[4];

	/*! Repository revision number */
	uint32_t        repoRevision;

	/*! Manufacturer name */
	char            manufacturer[DEVINFO_MANUFACTURER_STRLEN];

    /*! Build date, little endian order: [0] = status ('r'=release, 'd'=debug), [1] = year-2000, [2] = month, [3] = day.  Reversed byte order for big endian systems */
	uint8_t         buildDate[4];

    /*! Build date, little endian order: [0] = hour, [1] = minute, [2] = second, [3] = millisecond.  Reversed byte order for big endian systems */
	uint8_t         buildTime[4];
	
} dev_info_t;


/*! (DID_IMU) IMU data */
typedef struct
{
	/*! Time since boot up in seconds */
	double                  time;

	/*! Hardware status flags (eHardwareStatus). Copy of DID_SYS_PARAMS.hStatus */
	uint32_t				hStatus;

	/*! Gyroscope P, Q, R in radians / second */
	float                   pqr[3];

	/*! Acceleration X, Y, Z in meters / second squared */
	float                   acc[3];

	/*! Magnetometer X, Y, Z in Gauss */
	float                   mag[3];

	/*! Barometric pressure MSL altitude in meters */
	float					mslBar;
} imu_t;


/*! (DID_DELTA_THETA_VEL) Coning and sculling integral in body/IMU frame.  Updated at IMU rate. */
typedef struct
{
	/*! Time since boot up in seconds */
	double                  time;

	/*! Delta theta body frame (gyroscope P, Q, R integral) in radians */
	float                   theta[3];

	/*! Delta velocity body frame (acceleration X, Y, Z integral) in meters / second */
	float                   uvw[3];

	/*! Delta time for delta theta and delta velocity in seconds */
	float					dt;
} delta_theta_vel_t;


/*! (DID_INS_1) INS data with euler attitude and NED from reference LLA */
typedef struct
{
	/*! Weeks since January 1st, 1980 */
	uint32_t				week;
	
	/*! Time of week (since Sunday morning) in seconds, GMT */
	double					timeOfWeek;

	/*! INS status flags (eInsStatus). Copy of DID_SYS_PARAMS.iStatus */
	uint32_t				iStatus;

	/*! Hardware status flags (eHardwareStatus). Copy of DID_SYS_PARAMS.hStatus */
	uint32_t				hStatus;

	/*! Euler angles: roll, pitch, yaw in radians */
	float					theta[3];

	/*! Velocity U, V, W in meters per second */
	float					uvw[3];

	/*! WGS84 Latitude, longitude, height above ellipsoid in meters (not MSL) */
	double					lla[3];

	/*! North, east and down offset from reference latitude, longitude, and altitude to current latitude, longitude, and altitude */
	float					ned[3];
} ins_1_t;


/*! (DID_INS_2) INS data with quaternion attitude */
typedef struct
{
	/*! Weeks since January 1st, 1980 */
	uint32_t				week;
	
	/*! Time of week (since Sunday morning) in seconds, GMT */
	double					timeOfWeek;

	/*! INS status flags (eInsStatus). Copy of DID_SYS_PARAMS.iStatus */
	uint32_t				iStatus;

	/*! Hardware status flags (eHardwareStatus). Copy of DID_SYS_PARAMS.hStatus */
	uint32_t				hStatus;

	/*! Quaternion rotation: W, X, Y, Z */
	float					q[4];

	/*! Velocity U, V, W in meters per second */
	float					uvw[3];

	/*! WGS84 Latitude, longitude, height above ellipsoid in meters (not MSL) */
	double					lla[3];
} ins_2_t;


/*! (DID_GPS_POS) GPS position */
typedef struct
{
	/*! Number of week since January 1st, 1980 */
	uint32_t                week;
	
	/*! Time of week (since Sunday morning) in milliseconds, GMT */
	uint32_t                timeOfWeekMs;

	/*! GPS status: [7:0] number of satellites used in solution, [15:8] status flags, [23:16] fix type */
	uint32_t                status;

	/*! Carrier to noise ratio (receiver signal strength) of strongest satellite (dBHz) */
	uint32_t                cno;

	/*! WGS84 Latitude, longitude, height above ellipsoid in meters (not geoid / MSL) */
	double					lla[3];

	/*! Mean sea level (MSL) height above geoid altitude in meters */
	float					hMSL;

	/*! Horizontal accuracy in meters */
	float					hAcc;

	/*! Vertical accuracy in meters */
	float					vAcc;

	/*! Position dilution of precision in meters */
	float                   pDop;
} gps_nav_poslla_t;

/*! GPS Status */
enum eGpsStatus
{
	GPS_STATUS_NUM_SATS_USED_MASK			= (int)0x000000FF,

	/*! Fix type  */
	GPS_STATUS_FIX_TYPE_NO_FIX				= (int)0x00000000,
	GPS_STATUS_FIX_TYPE_DEAD_RECKONING_ONLY	= (int)0x00000100,
	GPS_STATUS_FIX_TYPE_2D_FIX				= (int)0x00000200,
	GPS_STATUS_FIX_TYPE_3D_FIX				= (int)0x00000300,
	GPS_STATUS_FIX_TYPE_GPS_PLUS_DEAD_RECK	= (int)0x00000400,
	GPS_STATUS_FIX_TYPE_TIME_ONLY_FIX		= (int)0x00000500,
	GPS_STATUS_FIX_TYPE_MASK				= (int)0x0000FF00,
	GPS_STATUS_FIX_TYPE_BIT_OFFSET			= (int)8,

	/*! Fix within limits (e.g. DOP & accuracy)  */
	GPS_STATUS_FIX_STATUS_FIX_OK			= (int)0x00010000,
	/*! Differential GPS (DGPS) used  */
	GPS_STATUS_FIX_STATUS_DGPS_USED			= (int)0x00020000,
	GPS_STATUS_FIX_STATUS_WEEK_VALID		= (int)0x00040000,
	GPS_STATUS_FIX_STATUS_TOW_VALID			= (int)0x00080000,
	GPS_STATUS_FIX_STATUS_MASK				= (int)0x00FF0000,
	GPS_STATUS_FIX_STATUS_BIT_OFFSET		= (int)16,
};

	
/*! (DID_GPS_VEL) GPS velocity */
typedef struct
{
	/*! Time of week (since Sunday morning) in milliseconds, GMT */
	uint32_t				timeOfWeekMs;
		
	/*! North, east and down velocity in meters / second */
	float					ned[3];

	/*! Ground speed magnitude in meters / second (always positive) */
	float					s2D;

	/*! 3D speed magnitude in meters / second */
	float					s3D;

	/*! Speed accuracy in meters / second */
	float					sAcc;

	/*! Velocity ground course (heading) in radians */
	float					course;

	/*! Velocity ground course accuracy in radians */
	float					cAcc;
} gps_nav_velned_t;


/*! (DID_GPS) GPS Data */
typedef struct
{
	/*! GPS position */
	gps_nav_poslla_t		pos;
	
	/*! GPS velocity */
	gps_nav_velned_t		vel;

	/*! Number of GPS messages received per second */
	uint32_t				rxps;

	/*! Time sync offset between local time since boot up to time of week in seconds */
	double                  towOffset;
} gps_t;


/*! GPS satellite information */
typedef struct
{
	/*! Satellite identifier */
	uint32_t				svId;

	/*! Carrier to noise ratio (receiver signal strength, dBHz) */
	uint32_t				cno;
} gps_sat_info_t;


/*! (DID_GPS_RSSI) GPS received signal strength indicator */
typedef struct
{
	/*! Time of week (since Sunday morning) in milliseconds, GMT */
	uint32_t                timeOfWeekMs;

	/*! Number of satellites in the sky */
	uint32_t				numSats;

	/*! Satellite information list */
	gps_sat_info_t			info[MAX_NUM_SAT_CHANNELS];
} gps_rssi_t;


/*! (DID_ASCII_BCAST_PERIOD) ASCII broadcast periods. This data structure (when it is included in the sCommData struct) is zeroed out on stop_all_broadcasts */ 
typedef struct
{
	/*! Broadcast period for ASCII IMU data in milliseconds. 0 for none */
	uint32_t                 imu;

	/*! Broadcast period for ASCII INS 1 data in milliseconds. 0 for none */
	uint32_t                 ins1;

	/*! Broadcast period for ASCII INS 2 data in milliseconds. 0 for none */
	uint32_t                 ins2;

	/*! Broadcast period for ASCII GPS position data in milliseconds. 0 for none */
	uint32_t                 gpsPos;

	/*! Broadcast period for ASCII GPS velocity data in milliseconds. 0 for none */
	uint32_t                 gpsVel;

	/*! Broadcast period for GGA (NMEA) data in milliseconds. 0 for none */
	uint32_t				 gga;

	/*! Broadcast period for GLL (NMEA) data in milliseconds. 0 for none */
	uint32_t				 gll;
} ascii_msgs_t;


/*! (DID_SYS_SENSORS) Output from system sensors */
typedef struct
{
	/*! Time since boot up in seconds */
	double					time;

	/*! Temperature in Celsius */
	float                   temp;

	/*! Gyros in radians / second */
	float                   pqr[3];

	/*! Accelerometers in meters / second squared */
	float                   acc[3];

	/*! Magnetometers in Gauss */
	float                   mag[3];

	/*! Barometric pressure in kilopascals */
	float                   bar;

	/*! Temperature of barometric pressure sensor in Celsius */
	float                   barTemp;

	/*! MSL altitude from barometric pressure sensor in meters */
	float                   mslBar;
	
	/*! EVB system input voltage in volts */
	float                   vin;

	/*! ADC analog input in volts */
	float                   ana1;
	float                   ana3;
	float                   ana4;
} sys_sensors_t;


/*! Sensor state variables */
typedef struct
{
	/*! Latitude, longitude and height above ellipsoid in degrees, degrees and meters */
	double                  lla[3];

	/*! Velocities in body frames of X, Y and Z in meters per second */
	float                   uvw[3];

	/*! Quaternion rotation: W, X, Y, Z */
	float					q[4];
} state_vars_t;

/*! (DID_INS_MISC) INS Misc data */
typedef struct
{
	/*! Time of week (since Sunday morning) in seconds, GMT */
	double                  timeOfWeek;

	/*! Time of week (since Sunday morning) in milliseconds, GMT */
	uint32_t                timeOfWeekMs;
	
	/*! State variables */
	state_vars_t            x;
	
	/*! Euler angles: roll, pitch, yaw (radians) */
	float                   theta[3];

	/*! North, east and down offset between reference and current latitude, longitude and altitude in meters */
	float                   ned[3];

	/*! Inertial to body frame DCM (Direct cosine matrix) */
	float                   dcm[9];

	/*! Body rates (INS bias estimates removed) in radians per second */
	float                   pqr[3];

	/*! Body accelerations (INS bias estimates removed) in meters per second squared */
	float                   acc[3];

	/*! Body magnetic in Gauss */
	float                   mag[3];

	/*! MSL altitude in meters from barometric pressure */
	float					mslBar;
} ins_misc_t;


/*! INS output */
typedef struct
{
	/*! Time of week (since Sunday morning) in milliseconds, GMT */
	uint32_t                timeOfWeekMs;

	/*! State variables */
	state_vars_t            x;

	/*! Euler angles (roll, pitch, yaw) in radians */
	float                   theta[3];

	/*! North, east and down offset between reference and current latitude, longitude and altitude in meters */
	float                   ned[3];

	/*! Inertial to body frame DCM (Direction cosine matrix) */
	float                   dcm[9];
} ins_output_t;


/*! (DID_SYS_PARAMS) System parameters */
typedef struct
{
	/*! Time of week (since Sunday morning) in milliseconds, GMT */
	uint32_t                timeOfWeekMs;

	/*! System status 1 flags (eInsStatus) */
	uint32_t                iStatus;

	/*! System status 2 flags (eHardwareStatus) */
	uint32_t                hStatus;

	/*! Attitude alignment detection */
	float				    alignAttDetect;

	/*! Attitude alignment error (approximation) in radians */
	float				    alignAttError;

	/*! Velocity alignment error in meters per second */
	float				    alignVelError;

	/*! Position alignment error in meters per second */
	float				    alignPosError;

	/*! Solution update period in milliseconds */
	uint32_t				insDtMs;

	/*! Ratio of system tuned clock to actual clock frequencies */
	float					ftf0; 

	/*! Magnetic north inclination (negative pitch offset) in radians */
	float                   magInclination;

	/*! Magnetic north declination (heading offset from true north) in radians */
	float                   magDeclination;

	/*! Earth magnetic field (magnetic north) magnitude (nominally 1) */
	float                   magMagnitude;
	
	/*! General fault code descriptor */
	uint32_t                genFaultCode;
} sys_params_t;


enum{
	SLOG_DISABLED			= 0,
	SLOG_INIT_W_INS1,				// Log configuration and initialization values once, then transition to SLOG_W_INS1
	SLOG_INIT_W_INS2,				// Log   "                                         , then transition to SLOG_W_INS2
	SLOG_W_INS1				= 10,	// Log INS1, INS_PARAMS, SYS_PARAMS, INS_INPUT, BIASES, OBS_PARAMS, GPS_POS
	SLOG_W_INS2,					// Log INS2, "
	SLOG_REDUCED_INS1		= 20,	// Log INS1, INS_PARAMS, SYS_PARAMS
	SLOG_REDUCED_INS2,				// Log INS2, "
};

/*! (DID_CONFIG) Configuration functions */
typedef struct
{
	/*! Set to 1 to reset processor into bootloader mode */
	uint32_t                enBootloader;

	/*! Set to 1 to log solution input */
	uint32_t                sLogCtrl;

	/*! Set to 1 to enable sensor stats */
	uint32_t                enSensorStats;

	/*! Set to 1 to enable RTOS stats */
	uint32_t                enRTOSStats;
} config_t;

#define NUM_SERVOS			8
#define SERVO_PULSE_US_MIN	700
#define SERVO_PULSE_US_MAX	2300

/*! (DID_IO) Input/Output */
typedef struct
{
	/*! Time of week (since Sunday morning) in milliseconds, GMT */
	uint32_t                timeOfWeekMs;

	/*! General purpose I/O status */
	uint32_t				gpioStatus;
} io_t;


/*! (DID_IO_SERVOS_PWM & DID_IO_SERVOS_PPM) I/O: PWM and PPM Servos */
typedef struct
{
	/*! Servo pulse time (us) */
	uint32_t				ch[NUM_SERVOS];
} io_servos_t;


enum eSysConfigBits
{
	/*! Disable automatic baudrate detection on startup */
	SYS_CFG_BITS_DISABLE_AUTOBAUD			= (int)0x00000001,

	/*! Disable automatic mag calibration */
	SYS_CFG_BITS_DISABLE_MAG_AUTO_CAL		= (int)0x00000002,

	/*! Disable LEDs */
	SYS_CFG_BITS_DISABLE_LEDS				= (int)0x00000004,
};

// typedef struct
// {											// Magnetic Distortions:
// 	float				bFrame[3];				// static to body frame
// 	float				iFrame[3];				// static to inertial frame
// 	float				accuracy;				// Goodness of fit cal accuracy indicator (smaller is better)
// } magDistortion_t;

typedef struct
{
	// 		uint32_t			timeMs;				// (ms)		Sample timestamp used to identify age of sample
	float				theta[3];			// (rad)	Euler attitude
	float				mag[3];				// (Gauss)	Measured magnetometer output (body frame)
} magCalPoint_t;

typedef struct
{
	magCalPoint_t		pt[5];
	float				delta;				// (Gauss)	Difference between pt[1].mag and pt[3].mag.
} magCalSet_t;

/*! (DID_MAGNETOMETER_CAL) Magnetometer Calibration */
typedef struct
{
	uint32_t				state;			// Calibration state
	magCalSet_t				data[3];		// Data array.  Each element contains the min and max value found for roll, pitch, and yaw.
// 	magDistortion_t			mDist;			// Temporary holder for newly calculated magnetic distortions
	float					accuracy;		// Goodness of fit cal accuracy indicator (smaller is better)
} mag_cal_t;


/*! (DID_FLASH_CONFIG) Configuration data */
typedef struct
{
	/*! Size of group or union, which is nvm_group_x_t + padding */
	uint32_t				size;

	/*! Checksum, excluding size and checksum */
	uint32_t                checksum;

	/*! Manufacturer method for restoring flash defaults */
	uint32_t                key;

	/*! Startup solution update period in milliseconds */
	uint32_t				startupInsDtMs;

	/*! Serial port 0 baud rate in bits per second */
	uint32_t				ser0BaudRate;
	
	/*! Serial port 1 baud rate in bits per second */
	uint32_t				ser1BaudRate;

	/*! Euler rotation from INS computational frame to INS output frame.  Order applied: heading, pitch, roll. RADIANS. */
	float					insRotation[3];

	/*! Offset to INS output (in INS output frame) in meters */
	float					insOffset[3];

	/*! GPS antenna offset from INS comp frame origin (in INS comp frame) in meters */
	float					gpsAntOffset[3];

	/* INS dynamic platform model.  Determines performance characteristics of system.  */
	uint32_t				insDynModel;
	
	/*! System configuration bits */
	uint32_t				sysCfgBits;

	/*! Reference latitude, longitude and height above ellipsoid for north east down (NED) calculations in degrees, degrees, and meters */
	double                  refLla[3];

	/*! Last latitude, longitude, HAE (height above ellipsoid) used to aid GPS startup in degrees, degrees and meters */
	double					lastLla[3];

	/*! Last LLA time since week start (Sunday morning) in milliseconds */
	uint32_t				lastLlaTimeOfWeekMs;

	/*! Last LLA number of week since January 1st, 1980 */
	uint32_t				lastLlaWeek;
	
	/*! Distance between current and last LLA that triggers an update of lastLla  */
	float					lastLlaUpdateDistance;

	/*! Hardware interface configuration bits */
	uint32_t				ioConfig;

	/*! Carrier board (i.e. eval board) configuration bits */
	uint32_t				cBrdConfig;

	/*! Minimum servo pulse time in microseconds */
// 	uint32_t				servoPulseUsMin[NUM_SERVOS];

	/*! Maximum servo pulse time in microseconds */
// 	uint32_t				servoPulseUsMax[NUM_SERVOS];

	/*! Servo failsafe trigger time in microseconds. Set to zero to disable all failsafes */
	uint32_t				servoFailsafeTriggerUs;

	/*! Servo failsafe pulse time in microseconds. Set to zero to disable failsafe. */
	uint32_t				servoFailsafePulseUs[NUM_SERVOS];

	/*! Earth magnetic field (magnetic north) inclination (negative pitch offset) in radians */
	float                   magInclination;

	/*! Earth magnetic field (magnetic north) declination (heading offset from true north) in radians */
	float                   magDeclination;

	/*! Earth magnetic field (magnetic north) magnitude (nominally 1) */
	float                   magMagnitude;

	/*! Magnetometer bias estimate in body frame (normalized gauss) */
	float					magB[3];

} nvm_flash_cfg_t;

/*! (DID_INS_RESOURCES) */
typedef struct
{	
	uint32_t                timeOfWeekMs;		//			Time of week (since Sunday morning) in milliseconds, GMT
	state_vars_t            x_dot;				//			State variables derivative
// 	float					ned_dot[3];			// (m/s)	Velocity (inertial frame)
// 	float					accCoriolis[3];		// (m/s^2)	Coriolis (centripetal) acceleration (inertial frame)
	float					magYawOffset;		// (rad)	Temporary offset in mag heading used to remove discontinuities when transitioning from moving to stationary (from GPS to mag heading)
} ins_res_t;

#if RTK_EMBEDDED

typedef enum
{
	rtk_data_type_observation = 0,
	rtk_data_type_rover_ephemeris = 1,
	rtk_data_type_rover_glonass_ephemeris = 2,
	rtk_data_type_base_station_antenna = 3
} rtk_data_type_t;

/*! (DID_RTK) - for this data id, the data must be pre-flipped to the endianess of the receiver */
typedef struct
{
	// type of RTK data (0 = observation, 1 = rover gps ephemeris, 2 = rover glonass ephemeris, 3 = base station antenna)
	uint8_t	dataType; // rtk_data_type_t
	union
	{
		obsd_t obsd;
		eph_t eph;
		geph_t geph;
		antenna_t antenna;
	} data;
} rtk_data_t;

// flip 32 bit integers and 64 bit doubles in rtk data
void flipRTK(rtk_data_t* r);

#endif

/*! Union of datasets */
union uDatasets
{
	dev_info_t			devInfo;
	imu_t				imu;
	delta_theta_vel_t	dThetaVel;
	ins_1_t				ins1;
	ins_2_t				ins2;
	gps_t				gps;
	gps_nav_poslla_t	gpsPos;
	gps_nav_velned_t	gpsVel;
	gps_rssi_t			gpsRssi;
	ins_misc_t			misc;
	sys_params_t		sysParams;
	sys_sensors_t		sysSensors;
	nvm_flash_cfg_t		flashCfg;
	io_t				io;
	ins_res_t			insRes;

#if RTK_EMBEDDED

	rtk_data_t			rtkData;

#endif

};


#if defined(AVR) || defined(ARM)
#pragma pack(4)
#else
#pragma pack(pop)
#endif

/*!
Creates a 32 bit checksum from data

@param data the data to create a checksum for
@param count the number of bytes in data

@return the 32 bit checksum for data
*/
uint32_t serialNumChecksum32(void* data, int size);
uint32_t flashChecksum32(void* data, int size);

/*!
Initializes data structures - this must be called ONCE and only ONCE at program start
Upon completion, IS_LITTLE_ENDIAN is populated

@return 0 if CPU architecture could not be determined, otherwise non-zero
*/
int initDataSets(void);

/*!
Flip the endianess of 32 bit values in data

@param data the data to flip 32 bit values in
@param dataLength the number of bytes in data
*/
void flipEndianess32(uint8_t* data, int dataLength);

/*!
Flip the bytes of a double - ptr is assumed to be at least 8 bytes

@param ptr the double to flip
*/
void flipDouble(uint8_t* ptr);

/*!
Flip double (64 bit) floating point values in data

@param data the data to flip doubles in
@param dataLength the number of bytes in data
@param offset offset into data to start flipping at
@param offsets a list of offsets of all doubles in data, starting at position 0
@param offsetsLength the number of items in offsets
*/
void flipDoubles(uint8_t* data, int dataLength, int offset, uint16_t* offsets, uint16_t offsetsLength);

/*!
Flip string values in data - this compensates for the fact that flipEndianess32 is called on all the data

@param data the data to flip string values in
@param dataLength the number of bytes in data
@param offset the offset into data to start flipping strings at
@param offsets a list of offsets and byte lengths into data where strings start at
@param offsetsLength the number of items in offsets, should be 2 times the string count
*/
void flipStrings(uint8_t* data, int dataLength, int offset, uint16_t* offsets, uint16_t offsetsLength);

/*!
Get the offsets of double (64 bit) floating point values given a data id

@param dataId the data id to get double offsets for
@param offsetsLength receives the number of double offsets

@return a list of offets of doubles or 0 if none
*/
uint16_t* getDoubleOffsets(int dataId, uint16_t* offsetsLength);

/*!
Gets the offsets and lengths of strings given a data id

@param dataId the data id to get string offsets and lengths for
@param offsetsLength receives the number of items in the return value

@return a list of offsets and lengths of strings for the data id or 0 if none
*/
uint16_t* getStringOffsetsLengths(int dataId, uint16_t* offsetsLength);

/*!
Contains 0 if big endian, 1 if little endian, 0xFF if unknown CPU architecture.
initDataSets must be called before this value is populated properly.
*/
extern unsigned char IS_LITTLE_ENDIAN; // 0 if big endian, 1 if little endian, 0xFF if unknown architecture

#ifdef __cplusplus
}
#endif

#endif // DATA_SETS_H
