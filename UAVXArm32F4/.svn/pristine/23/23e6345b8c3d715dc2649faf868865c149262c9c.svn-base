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
timeuS LastNavUpdateuS = 0;
NavStruct Nav;
real32 DesiredVel;
real32 POIHeading = 0.0f;
boolean SavedPIOState = false;
real32 NorthP, EastP;
uint8 PrevWPNo;
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


//_______________________________________________________________________________


void UpdateWhere(void) {

	Nav.Distance = sqrtf(Sqr(Nav.C[EastC].Pos) + Sqr(Nav.C[NorthC].Pos));
	Nav.Bearing = Make2Pi(atan2f(Nav.C[EastC].Pos, Nav.C[NorthC].Pos));
	Nav.Elevation = MakePi(atan2f(Altitude, Nav.Distance));
	Nav.Hint = MakePi((Nav.Bearing - PI) - Heading);

} // UpdateWhere

void CaptureWPHeading(void) {

	if (CurrWPNo != PrevWPNo) {
		Nav.OriginalWPBearing = Nav.WPBearing;
		PrevWPNo = CurrWPNo;
		F.CrossTrackActive = true;
	}

} // CaptureWPHeading

boolean UseCrossTrack(real32 DiffHeading) {

	boolean r = (((NavState == Transiting) || (NavState == AcquiringAltitude))
			&& (Nav.WPDistance > (Nav.ProximityRadius * 4.0f))
			&& (Abs(MakePi(DiffHeading)) < DegreesToRadians(45)));
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

timemS WPDistanceTimeout(void) {
	return (timemS) (FromPercent(120) * Nav.WPDistance / Nav.MaxVelocity)
			* 1000;
} // WPDistanceTimeout

timemS WPAltitudeTimeout(void) {
	return (timemS) (FromPercent(120) * Abs(Alt.P.Desired - Altitude)
			/ MaxROCMPS) * 1000;
} // WPDistanceTimeout


void CheckProximity(real32 V, real32 H) {

	F.WayPointCentred = Nav.WPDistance < Max(H, 2.0f * GPS.hAcc); // 95%

	if (F.AltControlEnabled) {
		if (F.WayPointCentred)
			F.WayPointAchieved = Abs(Alt.P.Error) < Max(V, 2.0f * GPS.vAcc); // 95%
		else
			F.WayPointAchieved = false;
	} else
		F.WayPointAchieved = F.WayPointCentred;

} // CheckProximity


void ZeroNavCorrections(void) {
	idx a;

	F.Glide =
			//F.Navigate = F.ReturnHome =
			F.CrossTrackActive
			= F.WayPointAchieved = F.WayPointCentred = F.OrbitingWP
					= F.RapidDescentHazard = false;

	SavedPIOState = F.UsingPOI;
	F.UsingPOI = false;

	for (a = NorthC; a <= EastC; a++)
		Nav.C[a].Corr = 0.0f;

	for (a = Pitch; a <= Yaw; a++)
		A[a].NavCorr = A[a].NavCorrP = Nav.C[a].PosIntE = 0.0f;

} // ZeroNavCorrections


void DecayNavCorrections(void) {
	static timeuS LastUpdateuS = 0;
	real32 dT, Decay;
	int32 a;

	dT = dTUpdate(&LastUpdateuS);
	Decay = NAV_CORR_DECAY;

	for (a = NorthC; a <= EastC; a++)
		Nav.C[a].Corr = Nav.C[a].PosIntE = 0.0f;

	for (a = Pitch; a <= Yaw; a++)
		A[a].NavCorrP = A[a].NavCorr = DecayX(A[a].NavCorr, Decay, dT);

	F.WayPointAchieved = F.WayPointCentred = F.CrossTrackActive = F.OrbitingWP
			= F.RapidDescentHazard = false;

} // DecayNavCorrections


real32 WPDistance(WPStruct * W) {
	real32 NorthE, EastE;

	NorthE = W->Pos[NorthC] - Nav.C[NorthC].Pos;
	EastE = W->Pos[EastC] - Nav.C[EastC].Pos;

	return sqrtf(Sqr(EastE) + Sqr(NorthE));

} // WPDistance


void DoOrbit(real32 Radius, real32 OrbitVelocity) {
	real32 TangentialVelocity;

	TangentialVelocity = (Nav.WPDistance - Radius) * Nav.VelKp;

	Rotate(&Nav.C[NorthC].DesVel, &Nav.C[EastC].DesVel, TangentialVelocity,
			OrbitVelocity, -Heading);

	Nav.DesiredHeading = Nav.WPBearing;

} // DoOrbit


real32 MinimumTurn(real32 Desired) {
	real32 HE, absHE;
	static real32 TurnSign;
	static boolean TurnCommit = false;

	HE = MakePi(Desired - Heading);

	if (F.IsFixedWing) {
		if (NavState == UsingThermal) {
			TurnCommit = true;
			TurnSign = 1.0f;
			HE = Make2Pi(Desired - Heading); // turn right
		} else {
			HE = MakePi(Desired - Heading);
			absHE = fabsf(HE);
			if (absHE > DegreesToRadians(160)) {
				TurnCommit = true;
				TurnSign = Sign(HE);
			} else if (absHE < DegreesToRadians(135))
				TurnCommit = false;

			if (TurnCommit)
				HE = TurnSign * absHE;
		}
	}
	return (HE);

} // MinimumTurn


void NavYaw(WPStruct * W) {
	real32 POIEastDiff, POINorthDiff, POIDistance;

	if (F.RapidDescentHazard)
		DoOrbit(DESCENT_RADIUS_M, DESCENT_VELOCITY_MPS); // only non FW
	else if (F.OrbitingWP)
		DoOrbit(W->OrbitRadius, W->OrbitVelocity);
	else {
		if (F.UsingPOI) {
			POIEastDiff = POI.Pos[EastC] - Nav.C[EastC].Pos;
			POINorthDiff = POI.Pos[NorthC] - Nav.C[NorthC].Pos;

			POIDistance = sqrtf(Sqr(POIEastDiff) + Sqr(POINorthDiff));
			Nav.DesiredHeading
					= (POIDistance > (Nav.ProximityRadius * 2.0f)) ? atan2f(
							POIEastDiff, POINorthDiff) : Heading;
		} else {
			if (F.UsingTurnToWP) {
				if (F.WayPointCentred) {
					if (F.WayPointAchieved && (CurrWPNo == 0))
						Nav.DesiredHeading = Nav.TakeoffBearing;
					else {
						// just leave the heading as is to avoid spinning top
					}
				} else
					Nav.DesiredHeading = Nav.WPBearing;
			}
		}
	}

	A[Yaw].NavCorr = 0.0f;

} // NavYaw


void NavPI_P(void) {
	idx a;
	real32 Ppos, Pvel, Ipos, Windup, S, NavMaxVel;

	for (a = NorthC; a <= EastC; a++) {

		NavMaxVel = VelScale[a] * Nav.MaxVelocity;

		//Position
		if (F.OrbitingWP || F.RapidDescentHazard) {

			Ppos = Nav.C[a].DesVel; // velocity computed in DoOrbit
			Nav.C[a].PosIntE = Ipos = 0.0f;

		} else {

			Ppos = Limit1(Nav.C[a].PosE * Nav.PosKp, NavMaxVel);

			Nav.C[a].PosIntE += (Nav.C[a].PosE * Nav.PosKi * NavdT);
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

		Pvel = Nav.C[a].VelE * Nav.VelKp * Nav.Sensitivity;
		Nav.C[a].Corr = Pvel;

	}

} // NavPI_P


void Navigate(WPStruct * W) {
	idx a;

	tickCountOn(NavigateTick);

	NavdT = dTUpdate(&LastNavUpdateuS);
	NavdTR = 1.0f / NavdT;

	Nav.C[NorthC].DesPos = W->Pos[NorthC];
	Nav.C[EastC].DesPos = W->Pos[EastC];

	Nav.C[NorthC].PosE = Nav.C[NorthC].DesPos - Nav.C[NorthC].Pos;
	Nav.C[EastC].PosE = Nav.C[EastC].DesPos - Nav.C[EastC].Pos;

	Nav.WPDistance = sqrtf(Sqr(Nav.C[EastC].PosE) + Sqr(Nav.C[NorthC].PosE));
	Nav.WPBearing = Make2Pi(atan2f(Nav.C[EastC].PosE, Nav.C[NorthC].PosE));

	CompensateCrossTrackError1D();

	if (F.IsFixedWing) {

		CheckProximity(Nav.ProximityAlt, Nav.ProximityRadius);

		A[Pitch].NavCorr = A[Yaw].NavCorr = 0.0f;

		Nav.DesiredHeading = Make2Pi(Nav.WPBearing);

	} else {

		VelScale[NorthC] = Abs(cosf(Nav.WPBearing));
		VelScale[EastC] = Abs(sinf(Nav.WPBearing));

		CheckProximity(Nav.ProximityAlt, Nav.ProximityRadius);

		NavYaw(W);
		NavPI_P();

		Rotate(&A[Pitch].NavCorr, &A[Roll].NavCorr, -Nav.C[NorthC].Corr,
				Nav.C[EastC].Corr, -Heading);

		for (a = Pitch; a <= Roll; a++)
			A[a].NavCorr = Limit1(A[a].NavCorr, Nav.MaxBankAngle);

	}

	tickCountOff(NavigateTick);

} // Navigate

void InitNavigation(void) {

	//DEFINITELY not memset(&Nav, 0, sizeof(NavStruct));

	Nav.Elevation = Nav.Bearing = Nav.Distance = Nav.TakeoffBearing
			= Nav.WPDistance = Nav.WPBearing = Nav.CrossTrackE = 0.0f;

	F.OriginValid = F.OffsetOriginValid = F.NavigationEnabled = F.UsingPOI
			= false;

	//GPS.C[NorthC].OriginRaw = GPS.C[NorthC].Raw = 0;
	//GPS.C[EastC].OriginRaw = GPS.C[EastC].Raw = 0;
	//zzzGPS.longitudeCorrection = 1.0f;

	CurrWPNo = 0;
	PrevWPNo = 255;
	NorthP = EastP = 0.0f; // origin

	GenerateHomeWP();
	UsingSurveyPulse = false;
	memset(&POI, 0, sizeof(WPStruct));
	memset(&NavPulse, 0, sizeof(NavPulseStruct));
	memset(&SavedNavPulse, 0, sizeof(NavPulseStruct));
	RefreshNavWayPoint();
	SetDesiredAltitude(0.0f);

	ZeroNavCorrections();

	NavState = PIC;

} // InitNavigation


