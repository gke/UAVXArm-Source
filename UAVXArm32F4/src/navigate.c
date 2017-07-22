// ===============================================================================================
// =                                UAVX Quadrocopter Controller                                 =
// =                           Copyright (c) 2008 by Prof. Greg Egan                             =
// =                 Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer                   =
// =                     http://code.google.com/p/uavp-mods/ http://uavp.ch                      =
// ===============================================================================================

//    This is part of UAVX.

//    UAVX is free software: you can redistribute it and/or modify it under the terms of the GNU 
//    General Public License as published by the Free Software Foundation, either version 3 of the 
//    License, or (at your option) any later version.

//    UAVX is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY; without
//    even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
//    See the GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License along with this program.  
//    If not, see http://www.gnu.org/licenses/

// Autonomous flight routines

#include "UAVX.h"

real32 NavdT, NavdTR;
uint32 LastNavUpdateuS = 0;
NavStruct Nav;
real32 DesiredVel;
real32 POIHeading = 0.0f;
real32 NorthP, EastP;
uint8 PrevWPNo;
boolean ResetNavHold = true;
real32 VelScale[2];

void RotateWPPath(real32 * nx, real32 * ny, real32 x, real32 y) {
	static real32 wpS = 0.0f;
	static real32 wpC = 1.0f;
	real32 HR, NorthDiff, EastDiff;

	if (CurrWPNo != PrevWPNo) {
		NorthDiff = WP.Pos[NorthC] - NorthP;
		EastDiff = WP.Pos[EastC] - EastP;

		NorthP = WP.Pos[NorthC];
		EastP = WP.Pos[EastC];

		HR = sqrtf(Sqr(EastDiff) + Sqr(NorthDiff));
		if (HR > 0.1f) {
			HR = 1.0f / HR;
			wpS = EastDiff * HR;
			wpC = NorthDiff * HR;
		}
		PrevWPNo = CurrWPNo;
		F.CrossTrackActive = true;
	}

	*nx = x * wpC + y * wpS;
	*ny = -x * wpS + y * wpC;

} // RotateWPPath

#if defined(INC_AEROSONDE)

real32 Aerosonde(void) {
	// "Lateral Track Control Law for Aerosonde UAV", M. Niculescu,
	// Paper 16, AIAA, 8-11 January 2001, Reno, NV
	real32 PosXTrack, VelXTrack, PosYTrack, VelYTrack;
	real32 DesYawRate;

	RotateWPPath(&PosXTrack, &PosYTrack, Nav.C[NorthC].PosE, Nav.C[EastC].PosE);
	RotateWPPath(&VelXTrack, &VelYTrack, Nav.C[NorthC].Vel, Nav.C[EastC].VelE);

	DesYawRate = (0.2f * PosXTrack * VelYTrack - PosYTrack * VelXTrack)
	* 0.0025;

	return (Limit1(DesYawRate, NAV_YAW_MAX_SLEW_RAD_S));

} // Aerosonde

#endif

#if defined(INC_L1)

//_______________________________________________________________________________

//Bank angle command based on angle between aircraft velocity vector and reference vector to path.
//S. Park, J. Deyst, and J. P. How, "A New Nonlinear Guidance Logic for Trajectory Tracking,"
//Proceedings of the AIAA Guidance, Navigation and Control
//Conference, Aug 2004. AIAA-2004-4900.
//Modified to use PD control for circle tracking to enable loiter radius less than L1 length
//Modified to enable period and damping of guidance loop to be set explicitly
//Modified to provide explicit control over capture angle

// Period in seconds of L1 tracking loop. This parameter is the primary control for agressiveness of turns in auto mode. This needs to be larger for less responsive airframes. The default of 20 is quite conservative, but for most RC aircraft will lead to reasonable flight. For smaller more agile aircraft a value closer to 15 is appropriate, or even as low as 10 for some very agile aircraft. When tuning, change this value in small increments, as a value that is much too small (say 5 or 10 below the right value) can lead to very radical turns, and a risk of stalling.
real32 L1Period = 20.0f;
// Damping ratio for L1 control. Increase this in increments of 0.05 if you are getting overshoot in path tracking. You should not need a value below 0.7 or above 0.85.
real32 L1Damping = 0.75f;
//Crosstrack error integrator gain. This gain is applied to the crosstrack error to ensure it converges to zero. Set to zero to disable. Smaller values converge slower, higher values will cause crosstrack error oscillation.
// @Range: 0 0.1 @Increment: 0.01
real32 _L1_ForwardTrack_i_gain = 0.02f;

real32 _L1_ForwardTrack_i, _L1_ForwardTrack_i_gain_prev;
real32 L1Dist;
boolean GPSDataStale = false;

real32 EAS2TAS;

real32 alongTrackDist;
real32 omega;
real32 _bearing_error;

real32 DesiredLateralAcc;
real32 OrbitDirection;
boolean _reverse;
real32 omegaA;
real32 AB_length;
real32 A_air_unity, A_air_unitx;

real32 _groundspeed_roll, _groundspeed_pitch;

real32 ABx, ABy;
real32 B_air_unitx, B_air_unity;

real32 A_airlength;
real32 NuP;

// Wrap AHRS yaw if in reverse - radians


real32 L1_get_yaw(void) {
	if (_reverse)
	return MakePi(PI + Heading);

	return Heading;
} // L1_get_yaw

// return the bank angle needed to achieve tracking from the last
// update_*() operation

real32 L1_nav_roll(void) {
	real32 ret;
	ret = cosf(A[Pitch].Angle) * atanf(DesiredLateralAcc * GRAVITY_MPS_S_R);
	return Limit1(ret, HALF_PI);

} // L1_nav_roll

// this is the turn distance assuming a 90 degree turn
real32 _L1_turn_distance(real32 WPRadius) {

	WPRadius *= Sqr(EAS2TAS);
	return Min(WPRadius, L1Dist);
}

/*
 this approximates the turn distance for a given turn angle. If the
 turn_angle is > 90 then a 90 degree turn distance is used, otherwise
 the turn distance is reduced linearly.
 This function allows straight ahead mission legs to avoid thinking
 they have reached the waypoint early, which makes things like camera
 trigger and ball drop at exact positions under mission control much easier
 */
real32 L1_turn_distance(real32 WPRadius, real32 turn_angle) {
	real32 distance_90 = 123; //XXX_turn_distance(WPRadius);

	turn_angle = fabsf(turn_angle);
	if (turn_angle >= HALF_PI)
	return distance_90;

	return distance_90 * turn_angle / HALF_PI;
}

boolean L1_reached_loiter_target(void) {
	return F.WayPointCentred;
}

// prevent indecision in our turning by using our previous turn
// decision if we are in a narrow angle band pointing away from the
// target and the turn angle has changed sign

real32 L1PreventIndecision(real32 Nu) {
	// we are moving away from the target waypoint and pointing
	// away from the waypoint (not flying backwards). The sign
	// of Nu has also changed, which means we are
	// oscillating in our decision about which way to go
	const real32 Nu_limit = 0.9f * PI;

	if ((fabsf(Nu) > Nu_limit) && (fabsf(NuP) > Nu_limit)
			&& (Abs(MakePi(Nav.WPBearing - Heading)) > RadiansToDegrees(120))
			&& ((Nu * NuP) < 0.0f))
	Nu = NuP;

	return Nu;
} // L1PreventIndecision


void L1CrossTrack(void) {
	real32 Nu;
	real32 ForwardTrackVel, LateralTrackVel;

	// Calculate L1 gain required for specified damping
	real32 K_L1 = 4.0f * Sqr(L1Damping);

	// Get current position and velocity
	//if (!_ahrs.get_position(_current_loc)) {
	// if no GPS loc available, maintain last nav/target_bearing
	//	GPSDataStale = true;
	//	return;
	//}

	// Calculate time varying control parameters
	// Calculate the L1 length required for specified period
	L1Dist = (1.0f / PI) * L1Damping * L1Period * GPS.gspeed;

	// Calculate the NE position of WP B relative to WP A

	// Check for AB zero length and track directly to the destination
	// if too small
	//	if (AB.length() < 1.0e-6f) {
	//		AB = location_diff(_current_loc, next_WP);
	//		if (AB.length() < 1.0e-6f)
	//			AB = Vector2f(cosf(get_yaw()), sinf(get_yaw()));
	//	}
	//	AB.normalize();

	// Calculate the NE position of the aircraft relative to WP A

	// calculate distance to target track, for reporting

	//Determine if the aircraft is behind a +-135 degree degree arc centred on WP A
	//and further than L1 distance from WP A. Then use WP A as the L1 reference point
	//Otherwise do normal L1 guidance
	//real32 WP_A_dist = WPDistance(CurrWP - 1);
	real32 alongTrackDist = 123; //A_air * AB;
	if ((Nav.WPDistance > L1Dist) && (alongTrackDist
					/Max(Nav.WPDistance, 1.0f) < -0.7071f)) {
		//Calc Nu to fly To WP A
		//	Vector2f A_air_unit = (A_air).normalized; // Unit vector from WP A to aircraft
		ForwardTrackVel = _groundspeed_roll; // Velocity across line
		LateralTrackVel = _groundspeed_pitch; // Velocity along line
		Nu = atan2f(ForwardTrackVel, LateralTrackVel);
		Nav.DesiredHeading = atan2f(-A_air_unity, -A_air_unitx); // bearing (radians) from AC to L1 point
	} else if (alongTrackDist > AB_length + GPS.gspeed * 3.0f) {
		// we have passed point B by 3 seconds. Head towards B
		// Calc Nu to fly To WP B
		//	Vector2f B_air = location_diff(next_WP, _current_loc);
		//	Vector2f B_air_unit = (B_air).normalized; // Unit vector from WP B to aircraft
		ForwardTrackVel = _groundspeed_roll; // Velocity across line
		LateralTrackVel = _groundspeed_pitch; // Velocity along line
		Nu = atan2f(ForwardTrackVel, LateralTrackVel);
		Nav.DesiredHeading = atan2f(-B_air_unity, -B_air_unitx); // bearing (radians) from AC to L1 point

	} else { //Calc Nu to fly along AB line

		//Calculate Nu2 angle (angle of velocity vector relative to line connecting waypoints)
		ForwardTrackVel = 123; //_groundspeed_vector % AB; // Velocity cross track
		LateralTrackVel = 123; //_groundspeed_vector * AB; // Velocity along track
		real32 Nu2 = atan2f(ForwardTrackVel, LateralTrackVel);
		//Calculate Nu1 angle (Angle to L1 reference point)
		real32 sine_Nu1 = 123; //_crosstrack_error/MAX(L1Dist, 0.1f);
		//Limit sine of Nu1 to provide a controlled track capture angle of 45 deg
		sine_Nu1 = Limit1(sine_Nu1, sinf(DegreesToRadians(45)));
		real32 Nu1 = asinf(sine_Nu1);

		// compute integral error component to converge to a crosstrack of zero when traveling
		// straight but reset it when disabled or if it changes. That allows for much easier
		// tuning by having it re-converge each time it changes.
		if ((_L1_ForwardTrack_i_gain <= 0) || (_L1_ForwardTrack_i_gain
						!= _L1_ForwardTrack_i_gain_prev)) {
			_L1_ForwardTrack_i = 0;
			_L1_ForwardTrack_i_gain_prev = _L1_ForwardTrack_i_gain;
		} else if (fabsf(Nu1) < DegreesToRadians(5)) {
			_L1_ForwardTrack_i += Nu1 * _L1_ForwardTrack_i_gain * NavdT;
			// an AHRS_TRIM_X=0.1 will drift to about 0.08 so 0.1 is a good worst-case to clip at
			_L1_ForwardTrack_i = Limit1(_L1_ForwardTrack_i, 0.1f);
		}

		// to converge to zero we must push Nu1 harder
		Nu1 += _L1_ForwardTrack_i;

		Nu = Nu1 + Nu2;
		Nav.DesiredHeading = atan2f(ABy, ABx) + Nu1; // bearing (radians) from AC to L1 point
	}

	NuP = Nu = L1PreventIndecision(Nu);

	Nu = Limit1(Nu, HALF_PI);
	DesiredLateralAcc = K_L1 * Sqr(GPS.gspeed) / L1Dist * sinf(Nu);

	// Waypoint capture status is always false during waypoint following
	F.WayPointCentred = false;

	_bearing_error = Nu; // bearing error angle (radians), +ve to left of track

	GPSDataStale = false; // status are correctly updated with current waypoint data

} // L1CrossTrack


void L1_update_loiter(real32 radius, uint8 OrbitDirection) { //void) //Location *center_WP, real32 radius, int8 OrbitDirection)

	// scale loiter radius with square of EAS2TAS to allow us to stay
	// stable at high altitude
	radius *= Sqr(EAS2TAS);

	// Calculate guidance gains used by PD loop (used during circle tracking)
	real32 omega = (TWO_PI / L1Period);
	real32 Kx = Sqr(omega);
	real32 Kv = 2.0f * L1Damping * omega;

	// Calculate L1 gain required for specified damping (used during waypoint capture)
	real32 K_L1 = 4.0f * Sqr(L1Damping);

	//Get current position and velocity
	//	if (_ahrs.get_position(_current_loc) == false) {
	// if no GPS loc available, maintain last nav/target_bearing
	GPSDataStale = true;
	//		return;
	//	}

	//zzz	Vector2f _groundspeed_vector = _ahrs.groundspeed_vector;

	//Calculate groundspeed
	//zzz	real32 groundSpeed = Max(_groundspeed_vector.length , 1.0f);


	// Calculate time varying control parameters
	// Calculate the L1 length required for specified period
	L1Dist = (1.0f / PI) * L1Damping * L1Period * GPS.gspeed;

	//Calculate the NE position of the aircraft relative to WP A
	//	Vector2f A_air = location_diff(center_WP, _current_loc);

	// Calculate the unit vector from WP A to aircraft
	// protect against being on the waypoint and having zero velocity
	// if too close to the waypoint, use the velocity vector
	// if the velocity vector is too small, use the heading vector
	//	Vector2f A_air_unit;
	//	if (A_air.length > 0.1f)
	//		A_air_unit = A_air.normalized;
	//	else
	//		if (_groundspeed_vector.length < 0.1f)
	//			A_air_unit = Vector2f(cosf(_ahrs.yaw), sinf(L1_get_yaw()));
	//		else
	//			A_air_unit = _groundspeed_vector.normalized;


	//Calculate Nu to capture center_WP
	real32 ForwardTrackVelCap = 123; // A_air_unit % _groundspeed_vector; // Velocity across line - perpendicular to radial inbound to WP
	real32 LateralTrackVelCap = 123; // - (_groundspeed_vector * A_air_unit); // Velocity along line - radial inbound to WP
	real32 Nu = atan2f(ForwardTrackVelCap, LateralTrackVelCap);

	NuP = Nu = L1PreventIndecision(Nu);
	Nu = Limit1(Nu, HALF_PI);

	//Calculate lat accln demand to capture center_WP (use L1 guidance law)
	real32 LateralAccDemCap = K_L1 * Sqr(GPS.gspeed) / L1Dist * sinf(Nu);

	//Calculate radial position and velocity errors
	real32 ForwardTrackVelCirc = -LateralTrackVelCap; // Radial outbound velocity - reuse previous radial inbound velocity
	real32 ForwardTrackErrCirc = A_airlength - radius; // Radial distance from the loiter circle

	// keep crosstrack error for reporting
	Nav.CrossTrackE = ForwardTrackErrCirc;

	//Calculate PD control correction to circle waypoint_ahrs.roll
	real32 LateralAccDemCircPD = (ForwardTrackErrCirc * Kx
			+ ForwardTrackVelCirc * Kv);

	//Calculate tangential velocity
	real32 velTangent = ForwardTrackVelCap * OrbitDirection;

	//Prevent PD demand from turning the wrong way by limiting the command when flying the wrong way
	if ((LateralTrackVelCap < 0.0f) && (velTangent < 0.0f))
	LateralAccDemCircPD = Max(LateralAccDemCircPD, 0.0f);

	// Calculate centripetal acceleration demand
	real32 LateralAccDemCircCtr = velTangent * velTangent
	/ Max((0.5f * radius), (radius + ForwardTrackErrCirc));

	//Sum PD control and centripetal acceleration to calculate lateral manoeuvre demand
	real32 LateralAccDemCirc = OrbitDirection * (LateralAccDemCircPD
			+ LateralAccDemCircCtr);

	// Perform switchover between 'capture' and 'circle' modes at the
	// point where the commands cross over to achieve a seamless transfer
	// Only fly 'capture' mode if outside the circle
	if ((ForwardTrackErrCirc > 0.0f) && (OrbitDirection * LateralAccDemCap
					< OrbitDirection * LateralAccDemCirc)) {
		DesiredLateralAcc = LateralAccDemCap;
		F.WayPointCentred = false;
		_bearing_error = Nu; // angle between demanded and achieved velocity vector, +ve to left of track
		Nav.DesiredHeading = atan2f(-A_air_unity, -A_air_unitx); // bearing (radians) from AC to L1 point
	} else {
		DesiredLateralAcc = LateralAccDemCirc;
		F.WayPointCentred = true;
		_bearing_error = 0.0f; // bearing error (radians), +ve to left of track
		Nav.DesiredHeading = atan2f(-A_air_unity, -A_air_unitx); // bearing (radians)from AC to L1 point
	}

	GPSDataStale = false; // status are correctly updated with current waypoint data
} // L1_update_loiter


void L1_update_heading_hold(void) {
	// Calculate normalised frequency for tracking loop
	const real32 omegaA = 4.4428f / L1Period; // sqrt(2)*pi/period
	// Calculate additional damping gain

	real32 Nu;

	// copy to Nav.WPBearing and _nav_bearing
	Nav.DesiredHeading = MakePi(Nav.DesiredHeading);
	//	_nav_bearing = DegreesToRadians(navigation_heading_cd * 0.01f);

	Nu = MakePi(Nav.DesiredHeading - Heading);

	//Vector2f _groundspeed_vector = _ahrs.groundspeed_vector;

	//Calculate groundspeed
	//	real32 groundSpeed = _groundspeed_vector.length;

	// Calculate time varying control parameters
	L1Dist = GPS.gspeed / omegaA; // L1 distance is adjusted to maintain a constant tracking loop frequency
	real32 VomegaA = GPS.gspeed * omegaA;

	// Waypoint capture status is always false during heading hold
	F.WayPointCentred = false;

	Nav.CrossTrackE = 0.0f;

	_bearing_error = Nu; // bearing error angle (radians), +ve to left of track

	// Limit Nu to +-pi
	Nu = Limit1(Nu, PI);
	DesiredLateralAcc = 2.0f * sinf(Nu) * VomegaA;

	GPSDataStale = false; // status are correctly updated with current waypoint data

} // L1_update_heading_hold

#endif

//_______________________________________________________________________________

void CaptureWPHeading(void) {

	if (CurrWPNo != PrevWPNo) {
		Nav.OriginalWPBearing = Nav.WPBearing;
		PrevWPNo = CurrWPNo;
		F.CrossTrackActive = true;
	}

} // CaptureWPHeading

boolean UseCrossTrack(real32 DiffHeading) {

	boolean r = (((NavState == Transiting) || (NavState == AcquiringAltitude)
			|| (NavState == ReturningHome)) && (Nav.WPDistance
			> NV.Mission.ProximityRadius * 2.0f) && (Abs(MakePi(DiffHeading))
			< DegreesToRadians(45)));
	if (!r)
		Nav.CrossTrackE = 0.0f;
	return r;
} // UseCrossTrack


void CompensateCrossTrackError1D(void) {
	real32 DiffHeading;

	CaptureWPHeading();

	DiffHeading = Nav.WPBearing - Nav.OriginalWPBearing;
	if (UseCrossTrack(DiffHeading)) {
		Nav.CrossTrackE = sinf(DiffHeading) * Nav.WPDistance;
		Nav.WPBearing
				+= Limit1(Nav.CrossTrackE * Nav.CrossTrackKp, DegreesToRadians(30));
		Nav.WPBearing = Make2Pi(Nav.WPBearing);
	} else {
		Nav.OriginalWPBearing = Nav.WPBearing; // safety
		F.CrossTrackActive = false;
		Nav.CrossTrackE = 0.0f;
	}

} // CompensateCrossTrackError1D

void CheckProximity(void) {

	if (CurrGPSType == UBXBinGPS)
		F.WayPointCentred = IsFixedWing ? Nav.WPDistance
				< NV.Mission.ProximityRadius : Nav.WPDistance < (GPS.hAcc
				* 5.0f); // TODO: is 5 too large?
	else
		F.WayPointCentred = Nav.WPDistance < NV.Mission.ProximityRadius;

	F.WayPointAchieved = F.WayPointCentred && (Abs(DesiredAltitude - Altitude)
			< NV.Mission.ProximityAltitude);
} // CheckProximity


void DecayPosCorr(void) {
	static uint32 LastUpdateuS = 0;
	real32 dT, Decay;
	int32 a;

	dT = dTUpdate(uSClock(), &LastUpdateuS);
	Decay = NAV_CORR_DECAY;

	for (a = NorthC; a <= EastC; a++)
		Nav.C[a].Corr = Nav.C[a].PosIntE = 0.0f;

	for (a = Pitch; a <= Yaw; a++)
		A[a].NavCorr = A[a].NavCorrP = DecayX(A[a].NavCorr, Decay, dT);

	F.WayPointAchieved = F.WayPointCentred = false;
} // DecayPosCorr

real32 WPDistance(WPStruct * W) {
	real32 NorthE, EastE;

	NorthE = W->Pos[NorthC] - Nav.C[NorthC].Pos;
	EastE = W->Pos[EastC] - Nav.C[EastC].Pos;

	return sqrtf(Sqr(EastE) + Sqr(NorthE));

} // WPDistance


void CheckResetNavHold(void) {
	uint8 a;

	if (ResetNavHold) {
		ResetNavHold = F.RapidDescentHazard = F.WayPointAchieved
				= F.WayPointCentred = false;
		for (a = NorthC; a <= EastC; a++) {
			Nav.C[a].PosIntE = 0.0f;
		}
	}
} // CheckResetNavHold


void DoOrbit(real32 Radius, real32 OrbitVelocity) {
	real32 TangentialVelocity;

	TangentialVelocity = (Nav.WPDistance - Radius) * Nav.O.Kp;

	Rotate(&Nav.C[NorthC].DesVel, &Nav.C[EastC].DesVel, TangentialVelocity,
			OrbitVelocity, -Heading);

	Nav.DesiredHeading = Nav.WPBearing;

} // DoOrbit


void NavYaw(WPStruct * W) {
	real32 POIEastDiff, POINorthDiff, POIDistance;

	if (F.RapidDescentHazard)
		DoOrbit(DESCENT_RADIUS_M, DESCENT_VELOCITY_MPS);
	else if (F.OrbitingWP)
		DoOrbit(W->OrbitRadius, W->OrbitVelocity);
	else {
		if (F.UsingPOI) {
			POIEastDiff = POI.Pos[EastC] - Nav.C[EastC].Pos;
			POINorthDiff = POI.Pos[NorthC] - Nav.C[NorthC].Pos;

			POIDistance = sqrtf(Sqr(POIEastDiff) + Sqr(POINorthDiff));
			Nav.DesiredHeading = (POIDistance > (NV.Mission.ProximityRadius
					* 2.0f)) ? atan2f(POIEastDiff, POINorthDiff) : Heading;
		} else {
			// manual or perhaps turn to heading?
		}
	}

	A[Yaw].NavCorr = 0.0f;

} // NavYaw


void NavPI_P(void) {
	uint8 a;
	real32 Ppos, Pvel, Ipos, Windup, S, NavMaxVel;

	for (a = NorthC; a <= EastC; a++) {

		NavMaxVel = VelScale[a] * Nav.MaxVelocity;

		//Position
		if (F.OrbitingWP || F.RapidDescentHazard) {

			Ppos = Nav.C[a].DesVel; // velocity computed in DoOrbit
			Nav.C[a].PosIntE = Ipos = 0.0f;

		} else {

			Ppos = Limit1(Nav.C[a].PosE * Nav.O.Kp, NavMaxVel);

			Nav.C[a].PosIntE += Nav.C[a].PosE * Nav.O.Ki * NavdT;
			Ipos = Nav.C[a].PosIntE;

			Nav.C[a].DesVel = Ppos + Ipos;

			Windup = Abs(Nav.C[a].DesVel) - NavMaxVel;
			if (Windup > 0.0f) {
				S = Sign(Nav.C[a].DesVel);
				Nav.C[a].PosIntE = S * (Abs(Nav.C[a].PosIntE) - Windup);
				Nav.C[a].DesVel = S * NavMaxVel;
			}
		}

		// Velocity - really this is bank angle and controls acceleration not velocity
		Nav.C[a].VelE = Nav.C[a].DesVel - Nav.C[a].Vel;

		Pvel = Nav.C[a].VelE * Nav.I.Kp * Nav.Sensitivity;
		Nav.C[a].Corr = Pvel;

	}

} // NavPI_P


void Navigate(WPStruct * W) {
	uint8 a;

	NavdT = dTUpdate(uSClock(), &LastNavUpdateuS);
	NavdTR = 1.0f / NavdT;

	Nav.C[NorthC].DesPos = W->Pos[NorthC];
	Nav.C[EastC].DesPos = W->Pos[EastC];

	Nav.C[NorthC].PosE = Nav.C[NorthC].DesPos - Nav.C[NorthC].Pos;
	Nav.C[EastC].PosE = Nav.C[EastC].DesPos - Nav.C[EastC].Pos;

	Nav.WPDistance = sqrtf(Sqr(Nav.C[EastC].PosE) + Sqr(Nav.C[NorthC].PosE));
	Nav.WPBearing = Make2Pi(atan2f(Nav.C[EastC].PosE, Nav.C[NorthC].PosE));

	CheckResetNavHold();

	CompensateCrossTrackError1D();

	CheckProximity();

	if ((State == InFlight) && (Nav.Sensitivity > NAV_SENS_THRESHOLD_STICK))
		if (IsFixedWing) {

			A[Pitch].NavCorr = A[Yaw].NavCorr = 0.0f;
			Nav.DesiredHeading = MakePi(Nav.WPBearing);

			A[Roll].NavCorr = atanf(Nav.DesiredHeading * Airspeed * GRAVITY_MPS_S_R
					* Nav.Sensitivity);

			A[Roll].NavCorr = Limit1(A[Roll].NavCorr, Nav.MaxAngle);
			A[Roll].NavCorr = SlewLimit(A[Roll].NavCorrP, A[Roll].NavCorr,
					Nav.AttitudeSlewRate, NavdT);
			A[Roll].NavCorrP = A[Roll].NavCorr;

		} else {

			NavYaw(W);

			VelScale[NorthC] = Abs(cosf(Nav.WPBearing));
			VelScale[EastC] = Abs(sinf(Nav.WPBearing));

			NavPI_P();

			Rotate(&A[Pitch].NavCorr, &A[Roll].NavCorr, -Nav.C[NorthC].Corr,
					Nav.C[EastC].Corr, -Heading);

			for (a = Pitch; a <= Roll; a++) {
				A[a].NavCorr = Limit1(A[a].NavCorr, Nav.MaxAngle);

				A[a].NavCorr = SlewLimit(A[a].NavCorrP, A[a].NavCorr,
						Nav.AttitudeSlewRate, NavdT);
				A[a].NavCorrP = A[a].NavCorr;
			}

		}
	else
		DecayPosCorr();

} // Navigate

void ZeroNavCorrections(void) {
	uint8 a;

	for (a = Pitch; a <= Yaw; a++)
		A[a].NavCorr = A[a].NavCorrP = 0.0f;

	for (a = NorthC; a <= EastC; a++)
		Nav.C[a].Corr = 0.0f;
} // ZeroNavCorrections

void InitNavigation(void) {

	//DEFINITELY not memset(&Nav, 0, sizeof(NavStruct));

	ZeroNavCorrections();

	Nav.Sensitivity = 1.0f;

	Nav.Elevation = Nav.Bearing = Nav.Distance = Nav.TakeoffBearing
			= Nav.WPDistance = Nav.WPBearing = Nav.CrossTrackE = 0.0f;

	if (!F.OriginValid || F.Emulation) {
		GPS.C[NorthC].OriginRaw = DEFAULT_HOME_LAT;
		GPS.C[EastC].OriginRaw = DEFAULT_HOME_LON;
		GPS.longitudeCorrection = DEFAULT_LON_CORR;
		//if (F.Emulation)
		//	GenerateNavTestMission();
	} else {
		NV.Mission.NoOfWayPoints = 0;
		NV.Mission.OriginAltitude = OriginAltitude;
		NV.Mission.RTHAltHold = (int16) (P(NavRTHAlt)); // ??? not used
	}

	NavState = PIC;

	POI.Pos[EastC] = POI.Pos[NorthC] = 0.0f;

	AttitudeHoldResetCount = 0;
	F.OriginValid = F.NavigationEnabled = F.NavigationActive
			= F.CrossTrackActive = F.WayPointAchieved = F.WayPointCentred
					= F.OrbitingWP = F.RapidDescentHazard = F.UsingPOI = false;

	CurrWPNo = 0;
	PrevWPNo = 255;
	NorthP = EastP = 0.0f; // origin
	RefreshNavWayPoint();
	DesiredAltitude = 0.0f;

} // InitNavigation

#if defined(INC_INAV)
/*-----------------------------------------------------------
 * XY-position controller for multicopter aircraft
 *-----------------------------------------------------------*/
static pt1Filter_t mcPosControllerAccFilterStateX, mcPosControllerAccFilterStateY;
static real32 lastAccelTargetX = 0.0f, lastAccelTargetY = 0.0f;

void resetMulticopterPositionController(void)
{
	for (int axis = 0; axis < 2; axis++) {
		navPidReset(&Nav.pids.vel[axis]);
		Nav.rcAdjustment[axis] = 0;
		pt1FilterReset(&mcPosControllerAccFilterStateX, 0.0f);
		pt1FilterReset(&mcPosControllerAccFilterStateY, 0.0f);
		lastAccelTargetX = 0.0f;
		lastAccelTargetY = 0.0f;
	}
}

boolean adjustMulticopterPositionFromRCInput(void)
{
	const int16_t rcPitchAdjustment = applyDeadband(rcCommand[PITCH], rcControlsConfig()->pos_hold_deadband);
	const int16_t rcRollAdjustment = applyDeadband(rcCommand[ROLL], rcControlsConfig()->pos_hold_deadband);

	if (rcPitchAdjustment || rcRollAdjustment) {
		// If mode is GPS_CRUISE, move target position, otherwise POS controller will passthru the RC input to ANGLE PID
		if (navConfig()->general.flags.user_control_mode == NAV_GPS_CRUISE) {
			const real32 rcVelX = rcPitchAdjustment * navConfig()->general.max_manual_speed / 500;
			const real32 rcVelY = rcRollAdjustment * navConfig()->general.max_manual_speed / 500;

			// Rotate these velocities from body frame to to earth frame
			const real32 neuVelX = rcVelX * Nav.actualState.cosYaw - rcVelY * Nav.actualState.sinYaw;
			const real32 neuVelY = rcVelX * Nav.actualState.sinYaw + rcVelY * Nav.actualState.cosYaw;

			// Calculate new position target, so Pos-to-Vel P-controller would yield desired velocity
			Nav.C[NorthC].DesPos = Nav.C[NorthC].Pos + (neuVelX / Nav[NorthC].I.Kp);
			Nav.C[EastC].DesPos = Nav.C[EastC].Pos + (neuVelY / Nav[NorthC].I.Kp);
		}

		return true;
	}
	else {
		// Adjusting finished - reset desired position to stay exactly where pilot released the stick
		if (Nav.flags.isAdjustingPosition) {
			t_fp_vector stopPosition;
			calculateMulticopterInitialHoldPosition(&stopPosition);
			setDesiredPosition(&stopPosition, 0, NAV_POS_UPDATE_XY);
		}

		return false;
	}
}

static real32 getVelocityHeadingAttenuationFactor(void)
{
	// In WP mode scale velocity if heading is different from bearing
	if (navGetCurrentStateFlags() & NAV_AUTO_WP) {
		const int32_t headingError = constrain(wrap_18000(Nav.desiredState.yaw - Nav.actualState.yaw), -9000, 9000);
		const real32 velScaling = cosf(CENTIDEGREES_TO_RADIANS(headingError));

		return Limit(velScaling * velScaling, 0.05f, 1.0f);
	} else {
		return 1.0f;
	}
}

static real32 getVelocityExpoAttenuationFactor(real32 velTotal, real32 velMax)
{
	// Calculate factor of how velocity with applied expo is different from unchanged velocity
	const real32 velScale = Limit(velTotal / velMax, 0.01f, 1.0f);

	// navConfig()->max_speed * ((velScale * velScale * velScale) * Nav.posResponseExpo + velScale * (1 - Nav.posResponseExpo)) / velTotal;
	// ((velScale * velScale * velScale) * Nav.posResponseExpo + velScale * (1 - Nav.posResponseExpo)) / velScale
	// ((velScale * velScale) * Nav.posResponseExpo + (1 - Nav.posResponseExpo));
	return 1.0f - Nav.posResponseExpo * (1.0f - Sqr(velScale)); // x^3 expo factor
}

static void updatePositionVelocityController_MC(void)
{
	Nav.C[NorthC].PosE = Nav.C[NorthC].DesPos - Nav.C[NorthC].Pos;
	Nav.C[EastC].PosE = Nav.C[EastC].DesPos - Nav.C[EastC].Pos;

	// Calculate target velocity
	Nav.C[NorthC].DesVel = Nav.C[NorthC].PosE * Nav[NorthC].I.Kp;
	Nav.C[EastC].DesVel = Nav.C[EastC].PosE * Nav[NorthC].I.Kp;

	// Get max speed from generic NAV (waypoint specific), don't allow to move slower than 0.5 m/s
	const real32 maxSpeed = getActiveWaypointSpeed();

	// Scale velocity to respect max_speed
	real32 newVelTotal = sqrtf(sq(Nav.C[NorthC].DesVel) + sq(Nav.C[EastC].DesVel));
	if (newVelTotal > maxSpeed) {
		Nav.C[NorthC].DesVel = maxSpeed * (Nav.C[NorthC].DesVel / newVelTotal);
		Nav.C[EastC].DesVel = maxSpeed * (Nav.C[EastC].DesVel / newVelTotal);
		newVelTotal = maxSpeed;
	}

	// Apply expo & attenuation if heading in wrong direction - turn first, accelerate later (effective only in WP mode)
	const real32 velHeadFactor = getVelocityHeadingAttenuationFactor();
	const real32 velExpoFactor = getVelocityExpoAttenuationFactor(newVelTotal, maxSpeed);
	Nav.[NorthC].DesVel= Nav.C[NorthC].DesVel * velHeadFactor * velExpoFactor;
	Nav.[EastC].DesVel = Nav.C[EastC].DesVel * velHeadFactor * velExpoFactor;

}

static void updatePositionAccelController_MC(timeDelta_t deltaMicros, real32 maxAccelLimit)
{

	// Calculate velocity error
	Nav.C[NorthC].VelE = Nav.[NorthC].DesVel- Nav.C[NorthC].Vel;
	Nav.C[EastC].VelE = Nav.[EastC].DesVel - Nav.C[EastC].Vel;

	// Calculate XY-acceleration limit according to velocity error limit
	real32 accelLimitX, accelLimitY;
	const real32 velErrorMagnitude = sqrtf(sq(Nav.C[NorthC].VelE) + sq(Nav.C[EastC].VelE));
	if (velErrorMagnitude > 0.1f) {
		accelLimitX = maxAccelLimit / velErrorMagnitude * fabsf(Nav.C[NorthC].VelE);
		accelLimitY = maxAccelLimit / velErrorMagnitude * fabsf(Nav.C[EastC].VelE);
	}
	else {
		accelLimitX = maxAccelLimit / 1.414213f;
		accelLimitY = accelLimitX;
	}

	// Apply additional jerk limiting of 1700 cm/s^3 (~100 deg/s), almost any copter should be able to achieve this rate
	// This will assure that we wont't saturate out LEVEL and RATE PID controller
	const real32 maxAccelChange = NavdT * 1700.0f;
	const real32 accelLimitXMin = Limit1(lastAccelTargetX - maxAccelChange, accelLimitX);
	const real32 accelLimitXMax = Limit1(lastAccelTargetX + maxAccelChange, accelLimitX);
	const real32 accelLimitYMin = Limit1(lastAccelTargetY - maxAccelChange, accelLimitY);
	const real32 accelLimitYMax = Limit1(lastAccelTargetY + maxAccelChange, accelLimitY);

	// TODO: Verify if we need jerk limiting after all

	// Apply PID with output limiting and I-term anti-windup
	// Pre-calculated accelLimit and the logic of navPidApply2 function guarantee that our newAccel won't exceed maxAccelLimit
	// Thus we don't need to do anything else with calculated acceleration
	const real32 newAccelX = navPidApply2(&Nav.C[NorthC].I, Nav.C[NorthC].DesVel, Nav.C[NorthC].Vel, NavdT, accelLimitXMin, accelLimitXMax, 0);
	const real32 newAccelY = navPidApply2(&Nav.C[EastC].I, Nav.C[EastC].DesVel, Nav.C[EastC].Vel, NavdT, accelLimitYMin, accelLimitYMax, 0);

	// Save last acceleration target
	lastAccelTargetX = newAccelX;
	lastAccelTargetY = newAccelY;

	// Apply LPF to jerk limited acceleration target
	const real32 accelN = pt1FilterApply4(&mcPosControllerAccFilterStateX, newAccelX, NAV_ACCEL_CUTOFF_FREQUENCY_HZ, NavdT);
	const real32 accelE = pt1FilterApply4(&mcPosControllerAccFilterStateY, newAccelY, NAV_ACCEL_CUTOFF_FREQUENCY_HZ, NavdT);

	// Rotate acceleration target into forward-right frame (aircraft)
	const real32 accelForward = accelN * Nav.actualState.cosYaw + accelE * Nav.actualState.sinYaw;
	const real32 accelRight = -accelN * Nav.actualState.sinYaw + accelE * Nav.actualState.cosYaw;

	// Calculate banking angles
	const real32 desiredPitch = atan2f(accelForward, GRAVITY_MPS_S);
	const real32 desiredRoll = atan2f(accelRight * cosf(desiredPitch), GRAVITY_MPS_S);

	Nav.rcAdjustment[ROLL] = Limit1(desiredRoll, Max.Angle);
	Nav.rcAdjustment[PITCH] = Limit1(desiredPitch, Max.Angle);
}

static void applyMulticopterPositionController(timeUs_t currentTimeUs)
{
	static timeUs_t previousTimePositionUpdate; // Occurs @ GPS update rate
	static timeUs_t previousTimeUpdate; // Occurs @ looptime rate

	const timeDelta_t deltaMicros = currentTimeUs - previousTimeUpdate;
	previousTimeUpdate = currentTimeUs;
	boolean bypassPositionController;

	// We should passthrough rcCommand is adjusting position in GPS_ATTI mode
	bypassPositionController = (navConfig()->general.flags.user_control_mode == NAV_GPS_ATTI) && Nav.flags.isAdjustingPosition;

	// If last call to controller was too long in the past - ignore it (likely restarting position controller)
	if (deltaMicros > HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
		previousTimeUpdate = currentTimeUs;
		previousTimePositionUpdate = currentTimeUs;
		resetMulticopterPositionController();
		return;
	}

	// Apply controller only if position source is valid. In absence of valid pos sensor (GPS loss), we'd stick in forced ANGLE mode
	// and pilots input would be passed thru to PID controller
	if (Nav.flags.hasValidPositionSensor) {
		// If we have new position - update velocity and acceleration controllers
		if (Nav.flags.horizontalPositionDataNew) {
			const timeDelta_t deltaMicrosPositionUpdate = currentTimeUs - previousTimePositionUpdate;
			previousTimePositionUpdate = currentTimeUs;

			if (!bypassPositionController) {
				// Update position controller
				if (deltaMicrosPositionUpdate < HZ2US(MIN_POSITION_UPDATE_RATE_HZ)) {
					updatePositionVelocityController_MC();
					updatePositionAccelController_MC(deltaMicrosPositionUpdate, NAV_ACCELERATION_XY_MAX);
				}
				else {
					resetMulticopterPositionController();
				}
			}

			// Indicate that information is no longer usable
			Nav.flags.horizontalPositionDataConsumed = 1;
		}
	}
	else {
		/* No position data, disable automatic adjustment, rcCommand passthrough */
		Nav.rcAdjustment[PITCH] = 0;
		Nav.rcAdjustment[ROLL] = 0;
		bypassPositionController = true;
	}

	if (!bypassPositionController) {
		rcCommand[PITCH] = pidAngleToRcCommand(Nav.rcAdjustment[PITCH], pidProfile()->max_angle_inclination[FD_PITCH]);
		rcCommand[ROLL] = pidAngleToRcCommand(Nav.rcAdjustment[ROLL], pidProfile()->max_angle_inclination[FD_ROLL]);
	}
}
#endif

