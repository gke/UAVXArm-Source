// Based on ArduSoar by Peter Braswell and Samuel Tabor


#include "UAVX.h"

#include "ekf.h"

// EKF zzz move elf.h?
#define INIT_STRENGTH_COVARIANCE 0.0049f
#define INIT_RADIUS_COVARIANCE 2500.0f
#define INIT_POS_COVARIANCE 300.0f
#define THERMAL_Q1 0.001f
#define THERMAL_Q2 0.03f
#define THERMAL_R 0.45f

// Filters
#define K_AS 0.05f // hard filtering
#define K_TE  0.03f // hard filtering


WPStruct TH = { { 0, 0, 0 }, AS_MIN_MPS, 10000, DEF_FW_LOITER_RADIUS_M, ALT_MAX_M,
		AS_MIN_MPS, navGlide };

#define F(o, n, k) (k * n + (1.0f - k) * o)

real32 AltCutOff = ALT_CUTOFF_M;
real32 AltMax = ALT_MAX_M;
real32 AltMin = ALT_MIN_M;

timemS LastKFUpdatemS = 0;

boolean ThrottleSuppressed = false;
boolean Thermalling = false;

SoarStruct Soar;

real32 Vario;
real32 VarioFilt;

real32 ThermalStrength;
boolean Thermalling;
real32 ThermalRadius;

real32 MacCready(real32 Alt) {
	const real32 XP[] = { 5, 30 }; // 500 3000
	const real32 YP[] = { 0, 4 };
	const int16 n = 2;
	int16 i;

	// Linear interpolation (without extrap)
	if (Alt <= XP[0])
		return YP[0];
	else if (Alt >= XP[n - 1])
		return YP[n - 1];
	else
		for (i = 0; i < n; i++) {
			if (Alt >= XP[i])
				return (((Alt - XP[i]) * (YP[i + 1] - YP[i]) / (XP[i + 1]
						- XP[i])) + YP[i]);
		}

	return -1.0; // never happens

} // MacCready

boolean ThermalOK(void) {
	ThermalStrength = (ekf.X[0] * exp(-Sqr(ThermalRadius / ekf.X[1])))
			+ EXP_THERMAL_SINK_MPS;

	return ThermalStrength > THERMAL_MIN_MPS; //MacCready(Altitude);
} // ThermalOK


boolean InAltitudeBand(void) {

	return (Altitude >= AltMinM) && (Altitude <= AltMaxM);
} // InAltitudeBand


boolean SuppressThrottle(void) {

	if (ThrottleSuppressed && (Altitude < AltMinM))
		ThrottleSuppressed = false;
	else if (!ThrottleSuppressed && (Altitude > AltCutoffM)) {
		ThrottleSuppressed = true;
		VarioFilt = 0; // zooming on motor climb
		//zzz spdHgt->reset_pitch_I(); // zero pitch integral
		mSTimer(CruiseTimeoutmS, CRUISE_MIN_MS);
	};

	return ThrottleSuppressed;

} // SuppressThrottle

boolean CommenceThermalling(void) {
	return mSTimeout(CruiseTimeoutmS) && (VarioFilt > THERMAL_MIN_MPS)
			&& InAltitudeBand();
	//zzz&& (StickThrottle <= IdleThrottle); // zzz
} // CommenceThermalling


boolean ResumeGlide(void) {

	return mSTimeout(ThermalTimeoutmS) && (!InAltitudeBand()
			|| !ThermalOK());

} // ResumeGlide


void InitThermalling(void) {
	real32 R[1][1] = { { Sqr(THERMAL_R) }, };
	const real32 cov_q1 = Sqr(THERMAL_Q1);
	const real32 cov_q2 = Sqr(THERMAL_Q2);
	real32 Q[4][4] = { { cov_q1, 0, 0, 0 }, { 0, cov_q2, 0, 0 }, { 0, 0,
			cov_q2, 0 }, { 0, 0, 0, cov_q2 } };
	real32 P[4][4] = { { INIT_STRENGTH_COVARIANCE, 0, 0, 0 }, { 0,
			INIT_RADIUS_COVARIANCE, 0, 0 }, { 0, 0, INIT_POS_COVARIANCE, 0 }, {
			0, 0, 0, INIT_POS_COVARIANCE } };

	// New state vector filter will be reset. Thermal location is placed in front of a/c
	real32 XR[] = { INIT_THERMAL_STRENGTH, INIT_THERMAL_RADIUS_M,
			THERMAL_DIST_AHEAD_M * cosf(Heading), THERMAL_DIST_AHEAD_M * sinf(
					Heading) };
	// Also reset covariance matrix p so filter is not affected by previous data
	EKFreset(XR, P, Q, R);

	AltCutOff = ALT_CUTOFF_M;
	AltMax = ALT_MAX_M;
	AltMin = ALT_MIN_M;

	Soar.Th[NorthC].Pos = Nav.C[NorthC].Pos;
	Soar.Th[EastC].Pos = Nav.C[EastC].Pos;

	LastKFUpdatemS = mSClock();
	mSTimer(ThermalTimeoutmS, THERMAL_MIN_MS);

} // InitThermalling


void InitCruising(void) {
	mSTimer(CruiseTimeoutmS, CRUISE_MIN_MS);
	ThrottleSuppressed = true; // glide initially
} // InitCruising

void UpdateThermalEstimate(void) {
	real32 dx, dy, dx_w, dy_w, KFdT;

	//invoked when F.NewNavUpdate

	timemS NowmS = mSClock();

	dx = Nav.C[NorthC].Pos - Soar.Th[NorthC].Pos;
	dy = Nav.C[EastC].Pos - Soar.Th[EastC].Pos;

	// Wind correction
	//if (F.WindEstValid) {
	//	KFdT = (NowmS - LastKFUpdatemS) * 0.001f;
	//	dx_w = Wind.Est[X] * KFdT;
	//	dy_w = Wind.Est[Y] * KFdT;
	//	dx -= dx_w;
	//	dy -= dy_w;
	//} else {
		dx_w = 0.0f;
		dy_w = 0.0f;
	//}

	// write log - save the data.
	SoaringTune.mS = NowmS;
	SoaringTune.vario = Vario;
	SoaringTune.dx = dx;
	SoaringTune.dy = dy;
	SoaringTune.x0 = ekf.X[0]; // strength
	SoaringTune.x1 = ekf.X[1]; // radius
	SoaringTune.x2 = ekf.X[2]; // North
	SoaringTune.x3 = ekf.X[3]; // East
	SoaringTune.lat = MToGPS(Nav.C[NorthC].Pos + ekf.X[2])
			+ GPS.C[NorthC].OriginRaw;
	SoaringTune.lon = MToGPS((Nav.C[EastC].Pos + ekf.X[3])
			/ GPS.longitudeCorrection) + GPS.C[EastC].OriginRaw;
	SoaringTune.alt = Altitude;
	SoaringTune.dx_w = dx_w;
	SoaringTune.dy_w = dy_w;

	EKFupdate(Vario, dx, dy); // update the filter

	Soar.Th[NorthC].Pos = Nav.C[NorthC].Pos;
	Soar.Th[EastC].Pos = Nav.C[EastC].Pos;

	LastKFUpdatemS = NowmS;

} // UpdateThermalEstimate


real32 CorrectNettoRate(real32 Vario, real32 Roll, real32 Airspeed) {
	/* Netto variometer

	 A second type of compensated variometer is the Netto or airmass variometer.
	 In addition to TE compensation, the Netto variometer adjusts for the intrinsic
	 sink rate of the glider at a given speed (the polar curve) adjusted for the
	 wing loading due to water ballast. The Netto variometer will always read zero
	 in still air. This provides the pilot with the accurate measurement of air
	 mass vertical movement critical for final glides (the last glide to the ultimate
	 destination location).

	 The Relative Netto Variometer indicates the vertical speed the glider would
	 achieve IF it flies at thermalling speed - independent of current air speed and
	 attitude. This reading is calculated as the Netto reading minus the glider's
	 minimum sink.

	 When the glider circles to thermal, the pilot needs to know the glider's
	 vertical speed instead of that of the air mass. The Relative Netto Variometer
	 (or sometimes the super Netto) includes a g-sensor to detect thermalling.

	 When thermalling, the sensor will detect acceleration (gravity plus centrifugal)
	 above 1 g and tell the relative netto variometer to stop subtracting the
	 sailplane's wing load-adjusted polar sink rate for the duration. Some earlier
	 nettos used a manual switch instead of the g sensor.
	 */
	real32 CL0; // CL0 = 2*W/(rho*S*V^2)
	real32 C1; // C1 = CD0/CL0
	real32 C2; // C2 = CDi0/CL0 = B*CL0
	real32 Netto;
	real32 cRoll;

	CL0 = POLAR_K / Sqr(Airspeed);
	C1 = POLAR_CD0 / CL0; // angle to overcome zero-lift drag
	C2 = POLAR_B * CL0; // angle to overcome lift induced drag at zero bank

	cRoll = 1.0f - 0.5f * Sqr(Roll); // first two terms of mclaurin series for cos(phi)
	Netto = Vario + Airspeed * (C1 + C2 / Sqr(cRoll)); // effect of aircraft drag removed

	return Netto;

} // CorrectNettoRate


void UpdateVario(void) {

	//real32 Headroom, AltitudeLost;

	/*
	 if (F.UsingWPNavigation) {
	 Headroom = WP.Pos[DownC] + DESCENT_ALT_DIFF_M;
	 AltitudeLost = (WPDistance(&WP) / GPS.gspeed) * (-EXP_THERMAL_SINK_MPS);
	 AltCutOff = AltitudeLost * 1.5f + Headroom;
	 AltMax = AltitudeLost * 2.0f + Headroom;
	 AltMin = Max(DESCENT_SAFETY_ALT_M, AltitudeLost + Headroom); //(real32) P(NavRTHAlt] + DESCENT_ALT_DIFF_M;
	 } else {
	 */
	AltCutOff = ALT_CUTOFF_M;
	AltMax = ALT_MAX_M;
	AltMin = ALT_MIN_M;
	//	}

	//#define USE_NETTO
#if defined(USE_NETTO)
	static timemS LastThermalUpdatemS = 0;
	static real32 TEP = 0.0f;
	static real32 AirspeedFilt = 0.0f;

	AirspeedFilt = F(AirspeedFilt, Airspeed, K_AS);
	real32 TE = Altitude + 0.5f * Sqr(AirspeedFilt) / GRAVITY_MPS_S;
	real32 SinkRate = CorrectNettoRate(0.0f, A[Roll].Angle, AirspeedFilt);

	Vario = SinkRate + (TE - TEP) / ((mSClock() - LastThermalUpdatemS) * 0.001f);
	VarioFilt = F(VarioFilt, Vario, K_TE);

	TEP = TE;

	LastThermalUpdatemS = mSClock();
#else
	Vario = VarioFilt = ROC;
#endif

} // UpdateVario


void DoGlider(void) {

	if (ResumeGlide()) {
		F.Soaring = false;
		mSTimer(CruiseTimeoutmS, CRUISE_MIN_MS);
		PrevWPNo = -1; // force CaptureWPHeading
		NavState = HoldingStation;
	} else {
		UpdateThermalEstimate();
		Soar.Th[NorthC].Pos = Nav.C[NorthC].Pos + ekf.X[2];
		Soar.Th[EastC].Pos = Nav.C[EastC].Pos + ekf.X[3];
		Soar.Th[EastC].Vel = 0.0f;
		Soar.Th[NorthC].Vel = 0.0f;
		//F.OrbitingWP = true;
		Navigate(&TH);
	}

} // DoGlider


