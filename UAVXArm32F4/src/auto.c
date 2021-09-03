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

real32 NavdT;

uint8 NavState, AlarmState;
uint8 LandingState = InitDescent;

uint8 NavSwState = SwLow;
uint8 NavSwStateP = SwUnknown;
boolean WPNavEnabled = false;
real32 SavedDesiredAltitude;

boolean NotDescending(void) {

	//TODO:
	return ((AltHoldThrComp < -(Alt.R.IntLim * 0.75f)) && (ROCTrack > 0.2f));

} // NotDescending

void InitiateDescent(void) {

	LandingState = InitDescent;
	F.AccUBump = false;

} // InitiateDescent

boolean ProbableLanding(void) {

	switch (CurrMotorStopSel) {
	case landContactSw:
		return F.LandingSwitch;
		break;
	case landDescentRate:
		return NotDescending();
		break;
	case landAccUBump:
		return F.AccUBump;
		break;
	case landDescentRateAndAccU:
		return NotDescending() && F.AccUBump;
		break;
	case landNoStop:
	default:
		return false;
		break;
	}

} // ProbableLanding

boolean DoLanding(void) {
	static timeuS LastLandUpdateuS;
	static timemS bucketmS = 0;
	real32 dTmS;
	boolean HasLanded;

	HasLanded = false;

	Alt.P.Desired = -100.0f; // TODO: redundant?

	switch (LandingState) {
	case InitDescent:
		//	if (Abs(Altitude) < NAV_LAND_M) {
		bucketmS = 1000;
		mSTimer(NavStateTimeoutmS, bucketmS); // let descent actually start
		LandingState = CommenceDescent;
		//	}
		break;
	case CommenceDescent:
		if (mSTimeout(NavStateTimeoutmS)) {
			LastLandUpdateuS = uSClock();
			bucketmS = NAV_LAND_TIMEOUT_MS;
			LandingState = Descent;
		}
		break;
	case Descent:
		dTmS = dTUpdate(&LastLandUpdateuS) * 1000;

		if (ProbableLanding()) {
			bucketmS = Max(0, bucketmS - dTmS);
			if (bucketmS <= 0.0f)
				LandingState = DescentStopped;
		} else {
			bucketmS = Min(bucketmS + dTmS * 2, NAV_LAND_TIMEOUT_MS);
			F.AccUBump = false;
		}

		mSTimer(NavStateTimeoutmS, bucketmS);
		break;
	case DescentStopped:
		HasLanded = true;
		break;
	} // switch

	return (HasLanded);
} // DoLanding

void InitiateShutdown(uint8 s) {
	AltHoldThrComp = 0.0f;
	ZeroNavCorrections();
	DesiredThrottle = 0.0f;
	F.DrivesArmed = false;
	LEDsOn();
	AlarmState = s;
	State = Shutdown;
} // InitiateShutdown

void DoAutoLanding(void) {

	if (DoLanding()) {
		InitiateShutdown(NoAlarms);
		NavState = Touchdown;
	}

} // DoAutoLanding

void DoForcedLanding(void) {

	if (IsMulticopter) {
		ZeroNavCorrections();
		CheckRapidDescentHazard();
		//	if (F.RapidDescentHazard)
		//		A[Pitch].NavCorr = DegreesToRadians(1);
	} else {
		ZeroNavCorrections();
		Nav.DesiredHeading = Make2Pi(Heading + Nav.HeadingTurnout);
	}
	AlarmState = ForcedLanding;

	if (DoLanding())
		InitiateShutdown(ForcedLanding);

} // DoForcedLanding

void CheckRapidDescentHazard(void) {

	F.RapidDescentHazard = F.UsingRapidDescent
			&& ((Altitude - Alt.P.Desired) > DESCENT_ALT_DIFF_M)
			&& (Altitude > DESCENT_SAFETY_ALT_M);

} // CheckRapidDescentHazard

void CheckFailsafes(void) {

	if (!F.GPSValid) {
		if (!F.ForcedLanding) {
			SavedDesiredAltitude = Alt.P.Desired;
			F.ForcedLanding = true;
			AlarmState = ForcedLanding;
			InitiateDescent();
		}
		DoForcedLanding();
	} else if (F.ForcedLanding) {
		if (Altitude > 15.0f) {
			F.ForcedLanding = false;
			AlarmState = NoAlarms;
			SetDesiredAltitude(SavedDesiredAltitude);
		}
	}

} // CheckFailsafes

void CapturePosition(void) {

	if (F.OriginValid) {
		HP.Pos[NorthC] = Nav.C[NorthC].Pos;
		HP.Pos[EastC] = Nav.C[EastC].Pos;
	}

} // CapturePosition

void InitiateRTH(void) {

	F.RapidDescentHazard = F.NewNavUpdate = F.WayPointAchieved =
			F.WayPointCentred = false;
	CurrWPNo = 0;
	PrevWPNo = 255;
	SetWPHome();
	NavState = Transiting;
	F.ReturnHome = true;

} // InitiateRTH

void InitiatePerch(void) {
	idx a;

	AltHoldThrComp = 0.0f;
	ZeroIntegrators();

	for (a = NorthC; a <= EastC; a++)
		Nav.C[a].Corr = 0.0f;

	for (a = Pitch; a <= Yaw; a++)
		A[a].NavCorr = A[a].DesiredNavCorr = Nav.C[a].PosIntE = 0.0f;

	mSTimer(NavStateTimeoutmS, (timemS) WP.Loiter * 1000);
	NavState = Perching;

} // InitiatePerch

void InitiatePH(void) {

	// resume mission if PH released? CurrWPNo = 0;
	if (!F.HoldingAlt)
		CaptureDesiredAltitude(Altitude);

	CapturePosition();
	DesiredHeading = Heading;
	NavState = HoldingStation;
	F.Navigate = true;

} // InitiatePH

void DoGliderStuff(void) {

	if (Altitude > AltMaxM) // TODO: pitch a function of Fl
		NavState = AltitudeLimiting;
	else {
		if (Altitude < AltMinM) //set best climb pitch
			NavState = BoostClimb;
		else {
			if (NavState == JustGliding) {
				if (CommenceThermalling()) { // after cruise timeout
					InitThermalling();
					CapturePosition();
					F.Soaring = true;
					mSTimer(ThermalTimeoutmS, THERMAL_MIN_MS);
					NavState = UsingThermal;
				} else
					Navigate(&HP);
			} else {
				if (ResumeGlide()) {
					F.Soaring = false;
					mSTimer(CruiseTimeoutmS, CRUISE_MIN_MS);
					CapturePosition();
					NavState = JustGliding;
				} else {
					UpdateThermalEstimate();
					Soar.Th[NorthC].Pos = Nav.C[NorthC].Pos + ekf.X[2];
					Soar.Th[EastC].Pos = Nav.C[EastC].Pos + ekf.X[3];
					Navigate(&TH);
				}
			}
		}
	}
} // DoGliderStuff

void UpdateRTHSwState(void) { // called in rc.c on every rx packet

	if (F.PassThru || (State != InFlight)) {

		NavSwState = SwLow;
		NavSwStateP = SwUnknown;

		F.ForcedLanding = F.Navigate = F.ReturnHome = F.AltControlEnabled =
				F.HoldingAlt = F.Glide = F.FenceAlarm = false;
		CaptureDesiredAltitude(Altitude);
		AltHoldThrComp = 0.0f;
		ZeroNavCorrections();

		if (F.OriginValid)
			CapturePosition();

		NavState = PIC;

	} else {

		if (NavSwState != NavSwStateP) {
			F.Glide = F.Navigate = F.ReturnHome = false;
			switch (NavSwState) {
			case SwLow:
				F.ForcedLanding = F.FenceAlarm = false;
				ZeroNavCorrections();
				CapturePosition();
				NavState = PIC;
				break;
			case SwMiddle:
				F.ForcedLanding = false;
				if (F.OriginValid) {
					InitiatePH();
					F.UsingPOI = SavedPIOState;
					F.UsingWPNavigation = (Config.Mission.NoOfWayPoints > 0)
							&& WPNavEnabled;
				}
				break;
			case SwHigh:
				if (F.OriginValid)
					InitiateRTH();
				break;
			} // switch

			NavSwStateP = NavSwState;
		}
	}

} // UpdateRTHSwState

void DoNavigation(void) {

	F.NavigationEnabled = (F.Navigate || F.ReturnHome)
			&& !((NavState == PIC) || F.PassThru);

	if (F.NavigationEnabled) {

		CheckFailsafes();

		if (F.NewNavUpdate) {
			F.NewNavUpdate = false;

			CheckFence();

			switch (NavState) {
			case AltitudeLimiting:
				// don't do hold as we may need to escape thermal
				if (Altitude < AltMaxM) { //set best glide pitch
					Sl = 0.0f;
					NavState = JustGliding;
				}
				break;
			case BoostClimb:
				Navigate(&HP);

				if (Altitude > AltCutoffM) //set best glide pitch
					NavState = JustGliding;
				break;
			case JustGliding:
			case UsingThermal:
				DoGliderStuff();
				break;
			case Takeoff:
				RefreshNavWayPoint();
				Navigate(&WP);

				if ((Altitude > NAV_MIN_ALT_M) || (Altitude >= WP.Pos[DownC]))
					NextWP();
				break;
			case Perching:
				if (mSTimeout(NavStateTimeoutmS)) {
					SetDesiredAltitude(WP.Pos[DownC]);
					NavState = Takeoff;
				}
				break;
			case Touchdown:
				if (WP.Action == navPerch)
					InitiatePerch();
				else
					NavPulse.Active = UsingSurveyPulse = false;

				break;
			case Descending:
				RefreshNavWayPoint();
				Navigate(&WP);

				if (WP.Action == navPerch) {
					if (DoLanding())
						InitiatePerch();
				} else
					DoAutoLanding();
				break;
			case AtHome:
				Navigate(&WP);

				if ((F.AltControlEnabled && F.UsingRTHAutoDescend)
						&& mSTimeout(NavStateTimeoutmS)) {
					F.RapidDescentHazard = false;
					if (F.IsFixedWing) {
						// just orbit
					} else {
						InitiateDescent();
						NavState = Descending;
					}
				}
				break;
			case AcquiringAltitude:
				RefreshNavWayPoint();
				Navigate(&WP);

				CheckRapidDescentHazard();

				// need check for already on the ground
				timemS LoitermS = (timemS) WP.Loiter * 1000;

				if (F.WayPointAchieved) {
					if (UsingNavBeep)
						ScheduleBeeper(200);
					switch (WP.Action) {
					case navLand:
						ScheduleBeeper(500);
						NavState = AtHome;
						break;
					case navPerch:
						InitiateDescent();
						NavState = Descending;
						break;
					case navVia:
						if (WP.Loiter > 0)
							if ((NavPulse.WidthmS < LoitermS)
									&& !UsingSurveyPulse)
								ScheduleNavPulse(&NavPulse, WP.PulseWidthmS,
										WP.PulsePeriodmS);
						NavState = OrbitingPOI;
						break;
					default:
						NavState = OrbitingPOI;
						break;
					} // switch

					mSTimer(NavStateTimeoutmS, LoitermS);

					F.RapidDescentHazard = false;
				}
				break;
			case WPAltFail:
				NextWP(); // TODO: skip this wp for now
				break;
			case Loitering:
			case OrbitingPOI:
				RefreshNavWayPoint();
				Navigate(&WP);

				switch (WP.Action) {
				case navOrbit:
					OrbitCamAngle = HALF_PI
							- atan2f(Altitude - WP.OrbitAltitude,
									WP.OrbitRadius);
					OrbitCamAngle = Limit(OrbitCamAngle, 0.0f, HALF_PI);
					break;
				case navVia:
					if (mSTimeout(NavStateTimeoutmS) && !UsingSurveyPulse)
						NavPulse.Active = false;
					break;
				case navPerch: // fixed wing just orbits
				default:
					OrbitCamAngle = 0.0f;
					break;
				} // switch

				if (mSTimeout(NavStateTimeoutmS))
					NextWP();

				break;
			case Transiting:
				RefreshNavWayPoint();
				Navigate(&WP);

				if (F.WayPointCentred) {
					mSTimer(NavStateTimeoutmS, WPAltitudeTimeout());
					NavState = AcquiringAltitude;
				}
				break;
			case WPProximityFail:
				NextWP(); // TODO: skip this wp for now
				break;
			case HoldingStation:
				if (F.IsFixedWing && F.Glide)
					NavState = JustGliding;
				else {
					if (F.AttitudeHold) {
						if (F.UsingWPNavigation) {
							mSTimer(NavStateTimeoutmS, WPDistanceTimeout());
							NextWP(); // start navigating
						} else
							Navigate(&HP); // maintain hold point
					} else { // allow override
						DecayNavCorrections();
						CapturePosition();
					}
				}
				break;
			default:
				// should not happen
				break;
			} // switch NavState

			F.OrbitingWP = (NavState == OrbitingPOI) && (WP.Action == navOrbit);
		}
	} else { // PIC

		F.ForcedLanding = false;
		AlarmState = NoAlarms;
		F.NewNavUpdate = false;
		Nav.WPBearing = DesiredHeading;
	}

	F.NewCommands = false; // Navigate modifies Desired Roll, Pitch and Yaw values.

} // DoNavigation

