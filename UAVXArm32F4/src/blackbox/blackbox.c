/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "UAVX.h"

#define ALWAYS 1
#define    AT_LEAST_MOTORS_1 1
#define     AT_LEAST_MOTORS_2 1
#define     AT_LEAST_MOTORS_3 1
#define     AT_LEAST_MOTORS_4 1
#define     AT_LEAST_MOTORS_5 0
#define     AT_LEAST_MOTORS_6 0
#define     AT_LEAST_MOTORS_7 0
#define     AT_LEAST_MOTORS_8 0

#define XYZ_AXIS_COUNT 3

#define DEBUG16_VALUE_COUNT 16
#define MAX_SUPPORTED_MOTORS 8
#define MAX_SUPPORTED_SERVOS 8

#define timeMs_t uint32

#if defined(BLACKBOX)

#include "blackbox.h"
#include "blackbox_encoding.h"
#include "blackbox_io.h"
#include "blackbox_fielddefs.h"
#include "utils.h"

/*
 #include "build/build_config.h"
 #include "build/debug.h"
 #include "build/version.h"

 #include "common/axis.h"
 #include "common/encoding.h"
 #include "common/maths.h"
 #include "common/utils.h"

 #include "config/feature.h"
 #include "config/parameter_group.h"
 #include "config/parameter_group_ids.h"

 #include "drivers/compass/compass.h"
 #include "drivers/sensor.h"
 #include "drivers/time.h"

 #include "fc/config.h"
 #include "fc/controlrate_profile.h"
 #include "fc/rc_controls.h"
 #include "fc/rc_modes.h"
 #include "fc/runtime_config.h"

 #include "flight/failsafe.h"
 #include "flight/mixer.h"
 #include "flight/navigation.h"
 #include "flight/pid.h"
 #include "flight/servos.h"

 #include "io/beeper.h"
 #include "io/gps.h"
 #include "io/serial.h"

 #include "rx/rx.h"

 #include "sensors/acceleration.h"
 #include "sensors/barometer.h"
 #include "sensors/battery.h"
 #include "sensors/compass.h"
 #include "sensors/gyro.h"
 #include "sensors/sonar.h"
 */

#if defined(ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT)
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_FLASH
#elif defined(ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT)
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_SDCARD
#else
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_SERIAL
#endif

/*
 PG_REGISTER_WITH_RESET_TEMPLATE(blackboxConfig_t, blackboxConfig, PG_BLACKBOX_CONFIG, 0);

 PG_RESET_TEMPLATE(blackboxConfig_t, blackboxConfig,
 .p_denom = 32,
 .device = DEFAULT_BLACKBOX_DEVICE,
 .on_motor_test = 0, // default off
 .record_acc = 1
 );
 */

#define BLACKBOX_SHUTDOWN_TIMEOUT_MILLIS 200

// Some macros to make writing FLIGHT_LOG_FIELD_* constants shorter:

//#define (FLIGHT_LOG_FIELD_PREDICTOR_x) CONCAT(FLIGHT_LOG_FIELD_PREDICTOR_, x)
//#define (FLIGHT_LOG_FIELD_ENCODING_x) CONCAT(FLIGHT_LOG_FIELD_ENCODING_, x)
//#define (FLIGHT_LOG_FIELD_CONDITION_x) CONCAT(FLIGHT_LOG_FIELD_CONDITION_, x)
#define UNSIGNED FLIGHT_LOG_FIELD_UNSIGNED
#define SIGNED FLIGHT_LOG_FIELD_SIGNED

static const char blackboxHeader[] =
		"H Product:Blackbox flight data recorder by Nicholas Sherlock\n"
			"H Data version:2\n";

static const char* const blackboxFieldHeaderNames[] = { "name", "signed",
		"predictor", "encoding", "predictor", "encoding" };

/* All field definition structs should look like this (but with longer arrs): */
typedef struct blackboxFieldDefinition_s {
	const char *name;
	// If the field name has a number to be included in square brackets [1] afterwards, set it here, or -1 for no brackets:
	int8 fieldNameIndex;

	// Each member of this array will be the value to print for this field for the given header index
	uint8 arr[1];
} blackboxFieldDefinition_t;

#define BLACKBOX_DELTA_FIELD_HEADER_COUNT       ARRAYLEN(blackboxFieldHeaderNames)
#define BLACKBOX_SIMPLE_FIELD_HEADER_COUNT      (BLACKBOX_DELTA_FIELD_HEADER_COUNT - 2)
#define BLACKBOX_CONDITIONAL_FIELD_HEADER_COUNT (BLACKBOX_DELTA_FIELD_HEADER_COUNT - 2)

typedef struct blackboxSimpleFieldDefinition_s {
	const char *name;
	int8 fieldNameIndex;

	uint8 isSigned;
	uint8 predict;
	uint8 encode;
} blackboxSimpleFieldDefinition_t;

typedef struct blackboxConditionalFieldDefinition_s {
	const char *name;
	int8 fieldNameIndex;

	uint8 isSigned;
	uint8 predict;
	uint8 encode;
	uint8 condition; // Decide whether this field should appear in the log
} blackboxConditionalFieldDefinition_t;

typedef struct blackboxDeltaFieldDefinition_s {
	const char *name;
	int8 fieldNameIndex;

	uint8 isSigned;
	uint8 Ipredict;
	uint8 Iencode;
	uint8 Ppredict;
	uint8 Pencode;
	uint8 condition; // Decide whether this field should appear in the log
} blackboxDeltaFieldDefinition_t;

/**
 * Description of the blackbox fields we are writing in our main intra (I) and inter (P) frames. This description is
 * written into the flight log header so the log can be properly interpreted (but these definitions don't actually cause
 * the encoding to happen, we have to encode the flight log ourselves in write{Inter|Intra}frame() in a way that matches
 * the encoding we've promised here).
 */
static const blackboxDeltaFieldDefinition_t blackboxMainFields[] = {
/* loopIteration doesn't appear in P frames since it always increments */
{ "loopIteration", -1, UNSIGNED, .Ipredict = (FLIGHT_LOG_FIELD_PREDICTOR_0),
		.Iencode = (FLIGHT_LOG_FIELD_ENCODING_UNSIGNED_VB), .Ppredict =
				(FLIGHT_LOG_FIELD_PREDICTOR_INC), .Pencode =
				FLIGHT_LOG_FIELD_ENCODING_NULL,
		(FLIGHT_LOG_FIELD_CONDITION_ALWAYS) },
/* Time advances pretty steadily so the P-frame prediction is a straight line */
{ "time", -1, UNSIGNED, .Ipredict = (FLIGHT_LOG_FIELD_PREDICTOR_0), .Iencode =
		(FLIGHT_LOG_FIELD_ENCODING_UNSIGNED_VB), .Ppredict =
		(FLIGHT_LOG_FIELD_PREDICTOR_STRAIGHT_LINE), .Pencode =
		(FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB),
		(FLIGHT_LOG_FIELD_CONDITION_ALWAYS) },

{ "axisP", 0, SIGNED, .Ipredict = (FLIGHT_LOG_FIELD_PREDICTOR_0), .Iencode =
		(FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB), .Ppredict =
		(FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS), .Pencode =
		(FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB),
		(FLIGHT_LOG_FIELD_CONDITION_ALWAYS) },
/* I terms get special packed encoding in P frames: */
{ "axisI", 0, SIGNED, .Ipredict = (FLIGHT_LOG_FIELD_PREDICTOR_0), .Iencode =
		(FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB), .Ppredict =
		(FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS), .Pencode =
		(FLIGHT_LOG_FIELD_ENCODING_TAG2_3S32),
		(FLIGHT_LOG_FIELD_CONDITION_ALWAYS) }, { "axisD", 0, SIGNED,
		.Ipredict = (FLIGHT_LOG_FIELD_PREDICTOR_0), .Iencode =
				(FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB), .Ppredict =
				(FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS), .Pencode =
				(FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB),
		(FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0) },

/* rcCommands are encoded together as a group in P-frames: */
{ "rc[Roll]", 0, SIGNED, .Ipredict = (FLIGHT_LOG_FIELD_PREDICTOR_0), .Iencode =
		(FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB), .Ppredict =
		(FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS), .Pencode =
		(FLIGHT_LOG_FIELD_ENCODING_TAG8_4S16),
		(FLIGHT_LOG_FIELD_CONDITION_ALWAYS) }, { "rc[Pitch]", 1, SIGNED,
		.Ipredict = (FLIGHT_LOG_FIELD_PREDICTOR_0), .Iencode =
				(FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB), .Ppredict =
				(FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS), .Pencode =
				(FLIGHT_LOG_FIELD_ENCODING_TAG8_4S16),
		(FLIGHT_LOG_FIELD_CONDITION_ALWAYS) }, { "rc[Yaw]", 2, SIGNED,
		.Ipredict = (FLIGHT_LOG_FIELD_PREDICTOR_0), .Iencode =
				(FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB), .Ppredict =
				(FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS), .Pencode =
				(FLIGHT_LOG_FIELD_ENCODING_TAG8_4S16),
		(FLIGHT_LOG_FIELD_CONDITION_ALWAYS) },
/* Throttle is always in the range [minthrottle..maxthrottle]: */
{ "rc[Throttle]", 3, UNSIGNED, .Ipredict = (0.0f), .Iencode =
		(FLIGHT_LOG_FIELD_ENCODING_UNSIGNED_VB), .Ppredict =
		(FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS), .Pencode =
		(FLIGHT_LOG_FIELD_ENCODING_TAG8_4S16),
		(FLIGHT_LOG_FIELD_CONDITION_ALWAYS) },

{ "vbatLatest", -1, UNSIGNED, .Ipredict = (FLIGHT_LOG_FIELD_PREDICTOR_VBATREF),
		.Iencode = (FLIGHT_LOG_FIELD_ENCODING_NEG_14BIT), .Ppredict =
				(FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS), .Pencode =
				(FLIGHT_LOG_FIELD_ENCODING_TAG8_8SVB),
		FLIGHT_LOG_FIELD_CONDITION_VBAT }, { "amperageLatest", -1, UNSIGNED,
		.Ipredict = (FLIGHT_LOG_FIELD_PREDICTOR_0), .Iencode =
				(FLIGHT_LOG_FIELD_ENCODING_UNSIGNED_VB), .Ppredict =
				(FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS), .Pencode =
				(FLIGHT_LOG_FIELD_ENCODING_TAG8_8SVB),
		FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC },

		{ "magADC", 0, SIGNED, .Ipredict = (FLIGHT_LOG_FIELD_PREDICTOR_0),
				.Iencode = (FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB), .Ppredict =
						(FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS), .Pencode =
						(FLIGHT_LOG_FIELD_ENCODING_TAG8_8SVB),
				FLIGHT_LOG_FIELD_CONDITION_MAG }, { "magADC", 1, SIGNED,
				.Ipredict = (FLIGHT_LOG_FIELD_PREDICTOR_0), .Iencode =
						(FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB), .Ppredict =
						(FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS), .Pencode =
						(FLIGHT_LOG_FIELD_ENCODING_TAG8_8SVB),
				FLIGHT_LOG_FIELD_CONDITION_MAG }, { "magADC", 2, SIGNED,
				.Ipredict = (FLIGHT_LOG_FIELD_PREDICTOR_0), .Iencode =
						(FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB), .Ppredict =
						(FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS), .Pencode =
						(FLIGHT_LOG_FIELD_ENCODING_TAG8_8SVB),
				FLIGHT_LOG_FIELD_CONDITION_MAG },

		{ "BaroAlt", -1, SIGNED, .Ipredict = (FLIGHT_LOG_FIELD_PREDICTOR_0),
				.Iencode = (FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB), .Ppredict =
						(FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS), .Pencode =
						(FLIGHT_LOG_FIELD_ENCODING_TAG8_8SVB),
				FLIGHT_LOG_FIELD_CONDITION_BARO },

		{ "sonarRaw", -1, SIGNED, .Ipredict = (FLIGHT_LOG_FIELD_PREDICTOR_0),
				.Iencode = (FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB), .Ppredict =
						(FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS), .Pencode =
						(FLIGHT_LOG_FIELD_ENCODING_TAG8_8SVB),
				FLIGHT_LOG_FIELD_CONDITION_SONAR },

		{ "rssi", -1, UNSIGNED, .Ipredict = (FLIGHT_LOG_FIELD_PREDICTOR_0),
				.Iencode = (FLIGHT_LOG_FIELD_ENCODING_UNSIGNED_VB), .Ppredict =
						(FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS), .Pencode =
						(FLIGHT_LOG_FIELD_ENCODING_TAG8_8SVB),
				FLIGHT_LOG_FIELD_CONDITION_RSSI },

		/* Gyros and accelerometers base their P-predictions on the average of the previous 2 frames to reduce noise impact */
		{ "gyroADC", 0, SIGNED, .Ipredict = (FLIGHT_LOG_FIELD_PREDICTOR_0),
				.Iencode = (FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB), .Ppredict =
						(FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2), .Pencode =
						(FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB),
				(FLIGHT_LOG_FIELD_CONDITION_ALWAYS) }, { "gyroADC", 1, SIGNED,
				.Ipredict = (FLIGHT_LOG_FIELD_PREDICTOR_0), .Iencode =
						(FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB), .Ppredict =
						(FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2), .Pencode =
						(FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB),
				(FLIGHT_LOG_FIELD_CONDITION_ALWAYS) }, { "gyroADC", 2, SIGNED,
				.Ipredict = (FLIGHT_LOG_FIELD_PREDICTOR_0), .Iencode =
						(FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB), .Ppredict =
						(FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2), .Pencode =
						(FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB),
				(FLIGHT_LOG_FIELD_CONDITION_ALWAYS) }, { "accSmooth", 0,
				SIGNED, .Ipredict = (FLIGHT_LOG_FIELD_PREDICTOR_0), .Iencode =
						(FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB), .Ppredict =
						(FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2), .Pencode =
						(FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB),
				FLIGHT_LOG_FIELD_CONDITION_ACC }, { "accSmooth", 1, SIGNED,
				.Ipredict = (FLIGHT_LOG_FIELD_PREDICTOR_0), .Iencode =
						(FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB), .Ppredict =
						(FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2), .Pencode =
						(FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB),
				FLIGHT_LOG_FIELD_CONDITION_ACC }, { "accSmooth", 2, SIGNED,
				.Ipredict = (FLIGHT_LOG_FIELD_PREDICTOR_0), .Iencode =
						(FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB), .Ppredict =
						(FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2), .Pencode =
						(FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB),
				FLIGHT_LOG_FIELD_CONDITION_ACC },

		/* Motors only rarely drops under minthrottle (when stick falls below mincommand), so predict minthrottle for it and use *unsigned* encoding (which is large for negative numbers but more compact for positive ones): */
		{ "Drive[0]", 0, UNSIGNED, .Ipredict = (FLIGHT_LOG_FIELD_PREDICTOR_0),
				.Iencode = (FLIGHT_LOG_FIELD_ENCODING_UNSIGNED_VB), .Ppredict =
						(FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2), .Pencode =
						(FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB), 0 }, //
		/* Subsequent motors base their I-frame values on the first one, P-frame values on the average of last two frames: */
		{ "Drive[1]", 1, UNSIGNED, .Ipredict = (FLIGHT_LOG_FIELD_PREDICTOR_0),
				.Iencode = (FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB), .Ppredict =
						(FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2), .Pencode =
						(FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB), 0 }, //
		{ "Drive[2]", 2, UNSIGNED, .Ipredict = (FLIGHT_LOG_FIELD_PREDICTOR_0),
				.Iencode = (FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB), .Ppredict =
						(FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2), .Pencode =
						(FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB), 0 }, //
		{ "Drive[3]", 3, UNSIGNED, .Ipredict = (FLIGHT_LOG_FIELD_PREDICTOR_0),
				.Iencode = (FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB), .Ppredict =
						(FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2), .Pencode =
						(FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB), 0 } //

};

typedef enum BlackboxState {
	BLACKBOX_STATE_DISABLED = 0,
	BLACKBOX_STATE_STOPPED,
	BLACKBOX_STATE_PREPARE_LOG_FILE,
	BLACKBOX_STATE_SEND_HEADER,
	BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER,
	BLACKBOX_STATE_SEND_SYSINFO,
	BLACKBOX_STATE_PAUSED,
	BLACKBOX_STATE_RUNNING,
	BLACKBOX_STATE_SHUTTING_DOWN,
	BLACKBOX_STATE_START_ERASE,
	BLACKBOX_STATE_ERASING,
	BLACKBOX_STATE_ERASED
} BlackboxState;

typedef struct blackboxMainState_s {
	uint32 time;

	int32 axisPID_P[XYZ_AXIS_COUNT];
	int32 axisPID_I[XYZ_AXIS_COUNT];
	int32 axisPID_D[XYZ_AXIS_COUNT];

	int16 rcCommand[4];
	int16 gyroADC[XYZ_AXIS_COUNT];
	int16 accSmooth[XYZ_AXIS_COUNT];
	int16 motor[MAX_SUPPORTED_MOTORS];

	int32 BaroAlt;
	int16 magADC[XYZ_AXIS_COUNT];
	int32 sonarRaw;
	uint16 rssi;

} blackboxMainState_t;

//From mixer.c:
extern float motorOutputHigh, motorOutputLow;

static BlackboxState blackboxState = BLACKBOX_STATE_DISABLED;

static uint32 blackboxLastArmingBeep = 0;
static uint32 blackboxLastFlightModeFlags = 0; // New event tracking of flight modes

static struct {
	uint32 headerIndex;

	/* Since these fields are used during different blackbox states (never simultaneously) we can
	 * overlap them to save on RAM
	 */
	union {
		int fieldIndex;
		uint32 startTime;
	} u;
} xmitState;

// Cache for FLIGHT_LOG_FIELD_CONDITION_* test results:
static uint32 blackboxConditionCache;

static uint32 blackboxIteration;
static uint16 blackboxLoopIndex;
static uint16 blackboxPFrameIndex;
static uint16 blackboxIFrameIndex;
// number of flight loop iterations before logging I-frame
// typically 32 for 1kHz loop, 64 for 2kHz loop etc
int16 blackboxIInterval = 0;
// number of flight loop iterations before logging P-frame
int16 blackboxPInterval = 0;
int32 blackboxSInterval = 0;
int32 blackboxSlowFrameIterationTimer;
static boolean blackboxLoggedAnyFrames;

// Keep a history of length 2, plus a buffer for MW to store the new values into
static blackboxMainState_t blackboxHistoryRing[3];

// These point into blackboxHistoryRing, use them to know where to store history of a given age (0, 1 or 2 generations old)

static blackboxMainState_t* blackboxHistory[3];

static boolean blackboxModeActivationConditionPresent = false;

// Return true if it is safe to edit the Blackbox configuration.

boolean blackboxMayEditConfig(void) {
	return blackboxState <= BLACKBOX_STATE_STOPPED;
}

static boolean blackboxIsOnlyLoggingIntraframes(void) {
	return false; //zzzblackboxConfig()->p_denom == 0;
}

static boolean testBlackboxConditionUncached(FlightLogFieldCondition condition) {
	switch (condition) {
	case FLIGHT_LOG_FIELD_CONDITION_ALWAYS:
		return true;

	case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_1:
	case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_2:
	case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_3:
	case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_4:
		return NoOfDrives >= condition
				- FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_1 + 1;

	case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0:
	case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_1:
	case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_2:
		//return A[condition - FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0].D != 0;

		/*
		 case FLIGHT_LOG_FIELD_CONDITION_MAG:
		 return sensors(SENSOR_MAG);

		 case FLIGHT_LOG_FIELD_CONDITION_BARO:
		 return sensors(SENSOR_BARO);

		 case FLIGHT_LOG_FIELD_CONDITION_VBAT:
		 return batteryConfig()->voltageMeterSource != VOLTAGE_METER_NONE;

		 case FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC:
		 return batteryConfig()->currentMeterSource == CURRENT_METER_ADC;

		 case FLIGHT_LOG_FIELD_CONDITION_SONAR:
		 return feature(FEATURE_SONAR);

		 case FLIGHT_LOG_FIELD_CONDITION_RSSI:
		 return rxConfig()->rssi_channel > 0 || feature(FEATURE_RSSI_ADC);

		 case FLIGHT_LOG_FIELD_CONDITION_NOT_LOGGING_EVERY_FRAME:
		 return blackboxConfig()->p_denom != 1;

		 case FLIGHT_LOG_FIELD_CONDITION_ACC:
		 return sensors(SENSOR_ACC) && blackboxConfig()->record_acc;

		 case FLIGHT_LOG_FIELD_CONDITION_DEBUG:
		 return debugMode != DEBUG_NONE;

		 case FLIGHT_LOG_FIELD_CONDITION_NEVER:
		 return false;
		 */

	default:
		return false;
	}
}

static void blackboxBuildConditionCache(void) {
	blackboxConditionCache = 0;
	for (FlightLogFieldCondition cond = FLIGHT_LOG_FIELD_CONDITION_FIRST; cond
			<= FLIGHT_LOG_FIELD_CONDITION_LAST; cond++) {
		if (testBlackboxConditionUncached(cond))
			blackboxConditionCache |= 1 << cond;
	}
}

static boolean testBlackboxCondition(FlightLogFieldCondition condition) {
	return (blackboxConditionCache & (1 << condition)) != 0;
}

static void blackboxSetState(BlackboxState newState) {
	//Perform initial setup required for the new state
	switch (newState) {
	case BLACKBOX_STATE_PREPARE_LOG_FILE:
		blackboxLoggedAnyFrames = false;
		break;
	case BLACKBOX_STATE_SEND_HEADER:
		blackboxHeaderBudget = 0;
		xmitState.headerIndex = 0;
		xmitState.u.startTime = mSClock();
		break;
	case BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER:
		xmitState.headerIndex = 0;
		xmitState.u.fieldIndex = -1;
		break;
	case BLACKBOX_STATE_SEND_SYSINFO:
		xmitState.headerIndex = 0;
		break;
	case BLACKBOX_STATE_RUNNING:
		blackboxSlowFrameIterationTimer = blackboxSInterval; //Force a slow frame to be written on the first iteration
		break;
	case BLACKBOX_STATE_SHUTTING_DOWN:
		xmitState.u.startTime = mSClock();
		break;
	default:
		;
	}
	blackboxState = newState;
}

static void writeIntraframe(void) {
	blackboxMainState_t *blackboxCurrent = blackboxHistory[0];

	blackboxWrite('I');

	blackboxWriteUnsignedVB(blackboxIteration);
	blackboxWriteUnsignedVB(blackboxCurrent->time);

	blackboxWriteSignedVBArray(blackboxCurrent->axisPID_P, XYZ_AXIS_COUNT);
	blackboxWriteSignedVBArray(blackboxCurrent->axisPID_I, XYZ_AXIS_COUNT);

	// Don't bother writing the current D term if the corresponding PID setting is zero
	for (int x = 0; x < XYZ_AXIS_COUNT; x++) {
		if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0
				+ x))
			blackboxWriteSignedVB(blackboxCurrent->axisPID_D[x]);
	}

	// Write roll, pitch and yaw first:
	blackboxWriteSigned16VBArray(blackboxCurrent->rcCommand, 3);

	/*
	 * Write the throttle separately from the rest of the RC data so we can apply a predictor to it.
	 * Throttle lies in range [minthrottle..maxthrottle]:
	 */
	blackboxWriteUnsignedVB(DesiredThrottle);

	if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_MAG))
		blackboxWriteSigned16VBArray(blackboxCurrent->magADC, XYZ_AXIS_COUNT);

	if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_BARO))
		blackboxWriteSignedVB(blackboxCurrent->BaroAlt);

	if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_SONAR))
		blackboxWriteSignedVB(blackboxCurrent->sonarRaw);

	if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_RSSI))
		blackboxWriteUnsignedVB(blackboxCurrent->rssi);

	blackboxWriteSigned16VBArray(blackboxCurrent->gyroADC, XYZ_AXIS_COUNT);
	if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_ACC))
		blackboxWriteSigned16VBArray(blackboxCurrent->accSmooth, XYZ_AXIS_COUNT);

	//Motors can be below minimum output when disarmed, but that doesn't happen much
	blackboxWriteUnsignedVB(blackboxCurrent->motor[0] - motorOutputLow);

	//Motors tend to be similar to each other so use the first motor's value as a predictor of the others

	for (int x = 1; x < NoOfDrives; x++)
		blackboxWriteSignedVB(blackboxCurrent->motor[x]
				- blackboxCurrent->motor[0]);

	//Rotate our history buffers:

	//The current state becomes the new "before" state
	blackboxHistory[1] = blackboxHistory[0];
	//And since we have no other history, we also use it for the "before, before" state
	blackboxHistory[2] = blackboxHistory[0];
	//And advance the current state over to a blank space ready to be filled
	blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3)
			+ blackboxHistoryRing;

	blackboxLoggedAnyFrames = true;
}

static void blackboxWriteMainStateArrayUsingAveragePredictor(
		int arrOffsetInHistory, int count) {
	int16 *curr = (int16*) ((char*) (blackboxHistory[0]) + arrOffsetInHistory);
	int16 *prev1 = (int16*) ((char*) (blackboxHistory[1]) + arrOffsetInHistory);
	int16 *prev2 = (int16*) ((char*) (blackboxHistory[2]) + arrOffsetInHistory);

	for (int i = 0; i < count; i++) {
		// Predictor is the average of the previous two history states
		int32 predictor = (prev1[i] + prev2[i]) / 2;

		blackboxWriteSignedVB(curr[i] - predictor);
	}
}

static void writeInterframe(void) {
	blackboxMainState_t *blackboxCurrent = blackboxHistory[0];
	blackboxMainState_t *blackboxLast = blackboxHistory[1];

	blackboxWrite('P');

	//No need to store iteration count since its delta is always 1


	// Since the difference between the difference between successive times will be nearly zero (due to consistent
	// looptime spacing), use second-order differences.

	blackboxWriteSignedVB((int32) (blackboxHistory[0]->time - 2
			* blackboxHistory[1]->time + blackboxHistory[2]->time));

	int32 deltas[8];
	arraySubInt32(deltas, blackboxCurrent->axisPID_P, blackboxLast->axisPID_P,
			XYZ_AXIS_COUNT);
	blackboxWriteSignedVBArray(deltas, XYZ_AXIS_COUNT);

	// The PID I field changes very slowly, most of the time +-2, so use an encoding
	// that can pack all three fields into one byte in that situation.

	arraySubInt32(deltas, blackboxCurrent->axisPID_I, blackboxLast->axisPID_I,
			XYZ_AXIS_COUNT);
	blackboxWriteTag2_3S32(deltas);

	// The PID D term is frequently set to zero for yaw, which makes the result from the calculation
	// always zero. So don't bother recording D results when PID D terms are zero.

	for (int a = 0; a <= Yaw; a++)
		if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0
				+ a))
			blackboxWriteSignedVB(blackboxCurrent->axisPID_D[a]
					- blackboxLast->axisPID_D[a]);

	// RC tends to stay the same or fairly small for many frames at a time, so use an encoding that
	// can pack multiple values per byte:

	for (int c = 0; c < 4; c++)
		deltas[c] = blackboxCurrent->rcCommand[c] - blackboxLast->rcCommand[c];

	blackboxWriteTag8_4S16(deltas);

	//Check for sensors that are updated periodically (so deltas are normally zero)
	int optionalFieldCount = 0;

	for (int a = X; a <= Z; a++)
		deltas[optionalFieldCount++] = blackboxCurrent->magADC[a]
				- blackboxLast->magADC[a];

	deltas[optionalFieldCount++] = blackboxCurrent->BaroAlt
			- blackboxLast->BaroAlt;

	deltas[optionalFieldCount++] = blackboxCurrent->sonarRaw
			- blackboxLast->sonarRaw;

	deltas[optionalFieldCount++] = (int32) blackboxCurrent->rssi
			- blackboxLast->rssi;

	blackboxWriteTag8_8SVB(deltas, optionalFieldCount);

	//Since gyros, accs and motors are noisy, base their predictions on the average of the history:
	blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(
			blackboxMainState_t, gyroADC), XYZ_AXIS_COUNT);

	blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(
			blackboxMainState_t, accSmooth), XYZ_AXIS_COUNT);

	blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(
			blackboxMainState_t, motor), NoOfDrives);

	//Rotate our history buffers
	blackboxHistory[2] = blackboxHistory[1];
	blackboxHistory[1] = blackboxHistory[0];
	blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3)
			+ blackboxHistoryRing;

	blackboxLoggedAnyFrames = true;
}

void blackboxValidateConfig(void) {
	// If we've chosen an unsupported device, change the device to serial
	/*zzz
	 switch (blackboxConfig()->device) {
	 case BLACKBOX_DEVICE_FLASH:
	 case BLACKBOX_DEVICE_SERIAL:
	 // Device supported, leave the setting alone
	 break;

	 default:
	 blackboxConfigMutable()->device = BLACKBOX_DEVICE_SERIAL;
	 }
	 */
}

static void blackboxResetIterationTimers(void) {
	blackboxIteration = 0;
	blackboxLoopIndex = 0;
	blackboxIFrameIndex = 0;
	blackboxPFrameIndex = 0;
	blackboxSlowFrameIterationTimer = 0;
}

// Start Blackbox logging if it is not already running. Intended to be called upon arming.

static void blackboxStart(void) {
	blackboxValidateConfig();

	if (!blackboxDeviceOpen()) {
		blackboxSetState(BLACKBOX_STATE_DISABLED);
		return;
	}

	blackboxHistory[0] = &blackboxHistoryRing[0];
	blackboxHistory[1] = &blackboxHistoryRing[1];
	blackboxHistory[2] = &blackboxHistoryRing[2];

	//No need to clear the content of blackboxHistoryRing since our first frame will be an intra which overwrites it

	// We use conditional tests to decide whether or not certain fields should be logged. Since our headers
	// must always agree with the logged data, the results of these tests must not change during logging. So
	// cache those now.

	blackboxBuildConditionCache();

	//zzzblackboxModeActivationConditionPresent = isModeActivationConditionPresent(
	//		BOXBLACKBOX);

	blackboxResetIterationTimers();

	// Record the beeper's current idea of the last arming beep time, so that we can detect it changing when
	// it finally plays the beep for this arming event.

	blackboxLastArmingBeep = getArmingBeepTimeMicros();
	memcpy(&blackboxLastFlightModeFlags, 0xff, //&rcModeActivationMask,
			sizeof(blackboxLastFlightModeFlags)); // record startup status

	blackboxSetState(BLACKBOX_STATE_PREPARE_LOG_FILE);
}

// Begin Blackbox shutdown.

void blackboxFinish(void) {
	switch (blackboxState) {
	case BLACKBOX_STATE_DISABLED:
	case BLACKBOX_STATE_STOPPED:
	case BLACKBOX_STATE_SHUTTING_DOWN:
		// We're already stopped/shutting down
		break;
	case BLACKBOX_STATE_RUNNING:
	case BLACKBOX_STATE_PAUSED:
		blackboxLogEvent(FLIGHT_LOG_EVENT_LOG_END, NULL);
		// Fall through
	default:
		blackboxSetState(BLACKBOX_STATE_SHUTTING_DOWN);
	}
}

// Fill the current state of the blackbox using values read from the flight controller

static void loadMainState(timeUs_t currentTimeUs) {
	idx a;

	blackboxMainState_t *blackboxCurrent = blackboxHistory[0];

	blackboxCurrent->time = currentTimeUs;

	for (a = Pitch; a <= Yaw; a++) {
		blackboxCurrent->axisPID_P[a] = A[a].RateKp;
		blackboxCurrent->axisPID_I[a] = 0; //A[a].RateKi;
		blackboxCurrent->axisPID_D[a] = A[a].RateKd;
		blackboxCurrent->gyroADC[a] = lrintf(Rate[a]);
		blackboxCurrent->accSmooth[a] = Acc[a];

		blackboxCurrent->magADC[a] = Mag[a];

	}

	for (int c = 0; c < 4; c++)
		blackboxCurrent->rcCommand[a] = RC[a];

	for (int d = 0; d < NoOfDrives; d++)
		blackboxCurrent->motor[a] = PW[d];

	blackboxCurrent->BaroAlt = BaroAltitude;

	blackboxCurrent->sonarRaw = RangefinderAltitude;

}

/**
 * Transmit the header information for the given field definitions. Transmitted header lines look like:
 *
 * H Field I name:a,b,c
 * H Field I predictor:0,1,2
 *
 * For all header types, provide a "mainFrameChar" which is the name for the field and will be used to refer to it in the
 * header (e.g. P, I etc). For blackboxDeltaField_t fields, also provide deltaFrameChar, otherwise set this to zero.
 *
 * Provide an array 'conditions' of FlightLogFieldCondition enums if you want these conditions to decide whether a field
 * should be included or not. Otherwise provide NULL for this parameter and NULL for secondCondition.
 *
 * Set xmitState.headerIndex to 0 and xmitState.u.fieldIndex to -1 before calling for the first time.
 *
 * secondFieldDefinition and secondCondition element pointers need to be provided in order to compute the stride of the
 * fieldDefinition and secondCondition arrays.
 *
 * Returns true if there is still header left to transmit (so call again to continue transmission).
 */
static boolean sendFieldDefinition(char mainFrameChar, char deltaFrameChar,
		const void *fieldDefinitions, const void *secondFieldDefinition,
		int fieldCount, const uint8 *conditions, const uint8 *secondCondition) {
	const blackboxFieldDefinition_t *def;
	unsigned int headerCount;
	static boolean needComma = false;
	size_t definitionStride = (char*) secondFieldDefinition
			- (char*) fieldDefinitions;
	size_t conditionsStride = (char*) secondCondition - (char*) conditions;

	headerCount = (deltaFrameChar) ? BLACKBOX_DELTA_FIELD_HEADER_COUNT
			: BLACKBOX_SIMPLE_FIELD_HEADER_COUNT;

	// We're chunking up the header data so we don't exceed our datarate. So we'll be called multiple times to transmit
	// the whole header.

	// On our first call we need to print the name of the header and a colon
	if (xmitState.u.fieldIndex == -1) {
		if (xmitState.headerIndex >= headerCount)
			return false; //Someone probably called us again after we had already completed transmission

		uint32 charsToBeWritten = strlen("H Field x :") + strlen(
				blackboxFieldHeaderNames[xmitState.headerIndex]);

		if (blackboxDeviceReserveBufferSpace(charsToBeWritten)
				!= BLACKBOX_RESERVE_SUCCESS)
			return true; // Try again later

		blackboxHeaderBudget
				-= blackboxPrintf("H Field %c %s:", xmitState.headerIndex
						>= BLACKBOX_SIMPLE_FIELD_HEADER_COUNT ? deltaFrameChar
						: mainFrameChar,
						blackboxFieldHeaderNames[xmitState.headerIndex]);

		xmitState.u.fieldIndex++;
		needComma = false;
	}

	// The longest we expect an integer to be as a string:
	const uint32 LONGEST_INTEGER_STRLEN = 2;

	for (; xmitState.u.fieldIndex < fieldCount; xmitState.u.fieldIndex++) {
		def
				= (const blackboxFieldDefinition_t*) ((const char*) fieldDefinitions
						+ definitionStride * xmitState.u.fieldIndex);

		if (!conditions || testBlackboxCondition(conditions[conditionsStride
				* xmitState.u.fieldIndex])) {
			// First (over)estimate the length of the string we want to print

			int32 bytesToWrite = 1; // Leading comma

			// The first header is a field name
			if (xmitState.headerIndex == 0)
				bytesToWrite += strlen(def->name) + strlen("[]")
						+ LONGEST_INTEGER_STRLEN;
			else
				//The other headers are integers
				bytesToWrite += LONGEST_INTEGER_STRLEN;

			// Now perform the write if the buffer is large enough
			if (blackboxDeviceReserveBufferSpace(bytesToWrite)
					!= BLACKBOX_RESERVE_SUCCESS)
				// Ran out of space!
				return true;

			blackboxHeaderBudget -= bytesToWrite;

			if (needComma)
				blackboxWrite(',');
			else
				needComma = true;

			// The first header is a field name
			if (xmitState.headerIndex == 0) {
				blackboxWriteString(def->name);

				// Do we need to print an index in brackets after the name?
				if (def->fieldNameIndex != -1)
					blackboxPrintf("[%d]", def->fieldNameIndex);

			} else
				//The other headers are integers
				blackboxPrintf("%d", def->arr[xmitState.headerIndex - 1]);
		}
	}

	// Did we complete this line?
	if (xmitState.u.fieldIndex == fieldCount
			&& blackboxDeviceReserveBufferSpace(1) == BLACKBOX_RESERVE_SUCCESS) {
		blackboxHeaderBudget--;
		blackboxWrite('\n');
		xmitState.headerIndex++;
		xmitState.u.fieldIndex = -1;
	}

	return xmitState.headerIndex < headerCount;
}

#ifndef bbHeader
#define bbHeader(name, format, ...) case __COUNTER__: \
                                                blackboxPrintfHeaderLine(name, format, __VA_ARGS__); \
                                                break;
#define bbHeader_CUSTOM(...) case __COUNTER__: \
                                                    {__VA_ARGS__}; \
                                               break;
#endif

/**
 * Transmit a portion of the system information headers. Call the first time with xmitState.headerIndex == 0. Returns
 * true iff transmission is complete, otherwise call again later to continue transmission.
 */
static boolean blackboxWriteSysinfo(void) {
#ifndef UNIT_TEST
	const uint16 motorOutputLowInt = lrintf(motorOutputLow);
	const uint16 motorOutputHighInt = lrintf(motorOutputHigh);

	// Make sure we have enough room in the buffer for our longest line (as of this writing, the "Firmware date" line)
	if (blackboxDeviceReserveBufferSpace(64) != BLACKBOX_RESERVE_SUCCESS)
		return false;

	//const controlRateConfig_t *currentControlRateProfile = controlRateProfiles(
	//		systemConfig()->activeRateProfile);

	switch (xmitState.headerIndex) {
	bbHeader("Firmware type", "%s", "UAVXArm32F4")
		;
		//bbHeader("Firmware revision", "%s %s (%s) %s", FC_FIRMWARE_NAME, FC_VERSION_STRING, shortGitRevision, targetName)
		//	;
		//bbHeader("Firmware date", "%s %s", buildDate, buildTime)
		//	;
	bbHeader("Log start datetime", "%s", blackboxGetStartDateTime())
		;
	bbHeader("minthrottle", "%d", 0)
		;
	bbHeader("maxthrottle", "%d", THR_MAXIMUM)
		;
	bbHeader("rates", "%d,%d,%d", A[Roll].RateMax,
			A[Pitch].RateMax,
			A[Yaw].RateMax)
		;
	bbHeader("rollPID", "%d,%d,%d", A[Roll].RateKp,
			0, //A[Roll].RateKi,
			A[Roll].RateKd)
		;
	bbHeader("pitchPID", "%d,%d,%d", A[Pitch].RateKp,
			0, //A[Pitch].RateKi,
			A[Pitch].RateKd)
		;
	bbHeader("yawPID", "%d,%d,%d", A[Yaw].RateKp,
			0, //A[Yaw].RateKi,
			A[Yaw].RateKd)
		;
	bbHeader("altPID", "%d,%d,%d", Alt.PosKp,
			Alt.PosKi,
			0) //Alt.PosKd)
		;
		// fall through
	default:
		return true;
	}

	xmitState.headerIndex++;
#endif // UNIT_TEST
	return false;
}

// Write the given event to the log immediately

void blackboxLogEvent(FlightLogEvent event, flightLogEventData_t *data) {
	// Only allow events to be logged after headers have been written
	if (!(blackboxState == BLACKBOX_STATE_RUNNING || blackboxState
			== BLACKBOX_STATE_PAUSED))
		return;

	//Shared header for event frames
	blackboxWrite('E');
	blackboxWrite(event);

	//Now serialize the data for this specific frame type
	switch (event) {
	case FLIGHT_LOG_EVENT_SYNC_BEEP:
		blackboxWriteUnsignedVB(data->syncBeep.time);
		break;
	case FLIGHT_LOG_EVENT_FLIGHTMODE: // New flightmode flags write
		blackboxWriteUnsignedVB(data->flightMode.flags);
		blackboxWriteUnsignedVB(data->flightMode.lastFlags);
		break;
	case FLIGHT_LOG_EVENT_LOGGING_RESUME:
		blackboxWriteUnsignedVB(data->loggingResume.logIteration);
		blackboxWriteUnsignedVB(data->loggingResume.currentTime);
		break;
	case FLIGHT_LOG_EVENT_LOG_END:
		blackboxWriteString("End of log");
		blackboxWrite(0);
		break;
	}
}

/* If an arming beep has played since it was last logged, write the time of the arming beep to the log as a synchronization point */
static void blackboxCheckAndLogArmingBeep(void) {
	// Use != so that we can still detect a change if the counter wraps
	if (getArmingBeepTimeMicros() != blackboxLastArmingBeep) {
		blackboxLastArmingBeep = getArmingBeepTimeMicros();
		flightLogEvent_syncBeep_t eventData;
		eventData.time = blackboxLastArmingBeep;
		blackboxLogEvent(FLIGHT_LOG_EVENT_SYNC_BEEP,
				(flightLogEventData_t *) &eventData);
	}
}

/* monitor the flight mode event status and trigger an event record if the state changes */
static void blackboxCheckAndLogFlightMode(void) {
	// Use != so that we can still detect a change if the counter wraps
//zzz	if (memcmp(&rcModeActivationMask, &blackboxLastFlightModeFlags,
//			sizeof(blackboxLastFlightModeFlags)))
	if (true)
	{
		flightLogEvent_flightMode_t eventData; // Add new data for current flight mode flags
		eventData.lastFlags = blackboxLastFlightModeFlags;
	//zzz	memcpy(&blackboxLastFlightModeFlags, &rcModeActivationMask,
	//			sizeof(blackboxLastFlightModeFlags));
	//	memcpy(&eventData.flags, &rcModeActivationMask, sizeof(eventData.flags));
	//	blackboxLogEvent(FLIGHT_LOG_EVENT_FLIGHTMODE,
	//			(flightLogEventData_t *) &eventData);
	}
}

boolean blackboxShouldLogPFrame(void) {
	return false; //zzz blackboxPFrameIndex == 0 && blackboxConfig()->p_denom != 0;
}

boolean blackboxShouldLogIFrame(void) {
	return blackboxLoopIndex == 0;
}

// Called once every FC loop in order to keep track of how many FC loop iterations have passed
void blackboxAdvanceIterationTimers(void) {
	++blackboxSlowFrameIterationTimer;
	++blackboxIteration;

	if (++blackboxLoopIndex >= blackboxIInterval) {
		blackboxLoopIndex = 0;
		blackboxIFrameIndex++;
		blackboxPFrameIndex = 0;
	} else if (++blackboxPFrameIndex >= blackboxPInterval)
		blackboxPFrameIndex = 0;
}

// Called once every FC loop in order to log the current state
void blackboxLogIteration(timeUs_t currentTimeUs) {
	// Write a keyframe every blackboxIInterval frames so we can resynchronise upon missing frames
	if (blackboxShouldLogIFrame()) {
		/*
		 * Don't log a slow frame if the slow data didn't change ("I" frames are already large enough without adding
		 * an additional item to write at the same time). Unless we're *only* logging "I" frames, then we have no choice.
		 */
		if (blackboxIsOnlyLoggingIntraframes())
			writeSlowFrameIfNeeded();

		loadMainState(currentTimeUs);
		writeIntraframe();
	} else {
		blackboxCheckAndLogArmingBeep();
		blackboxCheckAndLogFlightMode(); // Check for FlightMode status change event

		if (blackboxShouldLogPFrame()) {
			/*
			 * We assume that slow frames are only interesting in that they aid the interpretation of the main data stream.
			 * So only log slow frames during loop iterations where we log a main frame.
			 */
			writeSlowFrameIfNeeded();

			loadMainState(currentTimeUs);
			writeInterframe();
		}
		/*
		 if (F.HaveGPS) {
		 if (blackboxShouldLogGpsHomeFrame()) {
		 writeGPSHomeFrame();
		 writeGPSFrame(currentTimeUs);
		 } else if (gpsSol.numSat != gpsHistory.GPS_numSat || gpsSol.llh.lat
		 != gpsHistory.GPS_coord[LAT] || gpsSol.llh.lon
		 != gpsHistory.GPS_coord[LON]) {
		 //We could check for velocity changes as well but I doubt it changes independent of position
		 writeGPSFrame(currentTimeUs);
		 }
		 }
		 */

	}

	//Flush every iteration so that our runtime variance is minimized
	blackboxDeviceFlush();
}

/**
 * Call each flight loop iteration to perform blackbox logging.
 */
void blackboxUpdate(timeUs_t currentTimeUs) {
	switch (blackboxState) {
	case BLACKBOX_STATE_STOPPED:
		if (Armed()) {
			blackboxOpen();
			blackboxStart();
		}

		//zzz	if (IS_RC_MODE_ACTIVE(BOXBLACKBOXERASE))
		//		blackboxSetState(BLACKBOX_STATE_START_ERASE);

		break;
	case BLACKBOX_STATE_PREPARE_LOG_FILE:
		if (blackboxDeviceBeginLog())
			blackboxSetState(BLACKBOX_STATE_SEND_HEADER);
		break;
	case BLACKBOX_STATE_SEND_HEADER:
		blackboxReplenishHeaderBudget();
		//On entry of this state, xmitState.headerIndex is 0 and startTime is intialised

		// Once the UART has had time to init, transmit the header in chunks so we don't overflow its transmit
		// buffer, overflow the OpenLog's buffer, or keep the main loop busy for too long.

		if (mSClock() > xmitState.u.startTime + 100) {
			if (blackboxDeviceReserveBufferSpace(
					BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION)
					== BLACKBOX_RESERVE_SUCCESS) {
				for (int i = 0; i < BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION
						&& blackboxHeader[xmitState.headerIndex] != '\0'; i++, xmitState.headerIndex++) {
					blackboxWrite(blackboxHeader[xmitState.headerIndex]);
					blackboxHeaderBudget--;
				}
				if (blackboxHeader[xmitState.headerIndex] == '\0')
					blackboxSetState(BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER);
			}
		}
		break;
	case BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER:
		blackboxReplenishHeaderBudget();
		//On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
		if (!sendFieldDefinition('I', 'P', blackboxMainFields,
				blackboxMainFields + 1, ARRAYLEN(blackboxMainFields),
				&blackboxMainFields[0].condition,
				&blackboxMainFields[1].condition)) {
		}
		break;

	case BLACKBOX_STATE_SEND_SYSINFO:
		blackboxReplenishHeaderBudget();
		//On entry of this state, xmitState.headerIndex is 0

		//Keep writing chunks of the system info headers until it returns true to signal completion
		if (blackboxWriteSysinfo()) {

			// Wait for header buffers to drain completely before data logging begins to ensure reliable header delivery
			// (overflowing circular buffers causes all data to be discarded, so the first few logged iterations
			// could wipe out the end of the header if we weren't careful)

			if (blackboxDeviceFlushForce())
				blackboxSetState(BLACKBOX_STATE_RUNNING);
		}
		break;
	case BLACKBOX_STATE_PAUSED:
		// Only allow resume to occur during an I-frame iteration, so that we have an "I" base to work from
		if (true && blackboxShouldLogIFrame()) { // zzz BB active
			// Write a log entry so the decoder is aware that our large time/iteration skip is intended
			flightLogEvent_loggingResume_t resume;

			resume.logIteration = blackboxIteration;
			resume.currentTime = currentTimeUs;

			blackboxLogEvent(FLIGHT_LOG_EVENT_LOGGING_RESUME,
					(flightLogEventData_t *) &resume);
			blackboxSetState(BLACKBOX_STATE_RUNNING);

			blackboxLogIteration(currentTimeUs);
		}
		// Keep the logging timers ticking so our log iteration continues to advance
		blackboxAdvanceIterationTimers();
		break;
	case BLACKBOX_STATE_RUNNING:
		// On entry to this state, blackboxIteration, blackboxPFrameIndex and blackboxIFrameIndex are reset to 0
		// Prevent the Pausing of the log on the mode switch if in Motor Test Mode
		if (blackboxModeActivationConditionPresent && !false) // bb active
			blackboxSetState(BLACKBOX_STATE_PAUSED);
		else
			blackboxLogIteration(currentTimeUs);
		blackboxAdvanceIterationTimers();
		break;
	case BLACKBOX_STATE_SHUTTING_DOWN:
		//On entry of this state, startTime is set
		/*
		 * Wait for the log we've transmitted to make its way to the logger before we release the serial port,
		 * since releasing the port clears the Tx buffer.
		 *
		 * Don't wait longer than it could possibly take if something funky happens.
		 */
		if (blackboxDeviceEndLog(blackboxLoggedAnyFrames) && (mSClock()
				> xmitState.u.startTime + BLACKBOX_SHUTDOWN_TIMEOUT_MILLIS
				|| blackboxDeviceFlushForce())) {
			blackboxDeviceClose();
			blackboxSetState(BLACKBOX_STATE_STOPPED);
		}
		break;
	case BLACKBOX_STATE_START_ERASE:
		blackboxEraseAll();
		blackboxSetState(BLACKBOX_STATE_ERASING);
		//zzz beeper(BEEPER_BLACKBOX_ERASE);
		break;
	case BLACKBOX_STATE_ERASING:
		if (isBlackboxErased()) {
			//Done erasing
			blackboxSetState(BLACKBOX_STATE_ERASED);
			//zzzbeeper(BEEPER_BLACKBOX_ERASE);
		}
		break;
	case BLACKBOX_STATE_ERASED:
		//zzz	if (!IS_RC_MODE_ACTIVE(BOXBLACKBOXERASE))
		//		blackboxSetState(BLACKBOX_STATE_STOPPED);
		break;
	default:
		break;
	}

	// Did we run out of room on the device? Stop!
	if (isBlackboxDeviceFull()) {

		if (blackboxState != BLACKBOX_STATE_ERASING && blackboxState
				!= BLACKBOX_STATE_START_ERASE && blackboxState
				!= BLACKBOX_STATE_ERASED) {

			blackboxSetState(BLACKBOX_STATE_STOPPED);
			// ensure we reset the test mode flag if we stop due to full memory card
		}
	}
}

// Returns start time in ISO 8601 format, YYYY-MM-DDThh:mm:ss
// Year value of "0000" indicates time not set

static char startDateTime[20] = "0000-01-01T00:00:00";
const char *blackboxGetStartDateTime(void) {
	return startDateTime;
}

void blackboxSetStartDateTime(const char *dateTime, timeMs_t timeNowMs) {
	(void) dateTime;
	(void) timeNowMs;
}

int blackboxCalculatePDenom(int rateNum, int rateDenom) {
	return blackboxIInterval * rateNum / rateDenom;
}

uint8 blackboxGetRateDenom(void) {
	return gcd(blackboxIInterval, blackboxPInterval);
}

uint8 blackboxGetRateNum(void) {
	return 0; // zzz blackboxGetRateDenom() * blackboxConfig()->p_denom
	// / blackboxIInterval;
}

// Call during system startup to initialize the blackbox.

void blackboxInit(void) {

	blackboxResetIterationTimers();

	// an I-frame is written every 32ms
	// gyro.targetLooptime is 1000 for 1kHz loop, 500 for 2kHz loop etc, gyro.targetLooptime is rounded for short looptimes
	/*zzz

	 if (gyro.targetLooptime == 31) // rounded from 31.25us
	 blackboxIInterval = 1024;
	 else if (gyro.targetLooptime == 63) // rounded from 62.5us
	 blackboxIInterval = 512;
	 else
	 blackboxIInterval = (uint16)(32 * 1000 / gyro.targetLooptime);
	 */
	// by default p_denom is 32 and a P-frame is written every 1ms
	// if p_denom is zero then no P-frames are logged
	/* zzz
	 if (blackboxConfig()->p_denom == 0)
	 blackboxPInterval = 0;
	 else if (blackboxConfig()->p_denom > blackboxIInterval && blackboxIInterval
	 >= 32)
	 blackboxPInterval = 1;
	 else
	 blackboxPInterval = blackboxIInterval / blackboxConfig()->p_denom;

	 if (blackboxConfig()->device)
	 blackboxSetState(BLACKBOX_STATE_STOPPED);
	 else
	 blackboxSetState(BLACKBOX_STATE_DISABLED);


	 blackboxSInterval = blackboxIInterval * 256; // S-frame is written every 256*32 = 8192ms, approx every 8 seconds

	 */
}
#endif
