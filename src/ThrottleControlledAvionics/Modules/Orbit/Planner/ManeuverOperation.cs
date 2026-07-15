//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License.
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    public class ManeuverPlannerContext
    {
        public VesselWrapper VSL;
        public Vessel Vessel => VSL.vessel;
        public Orbit Orbit;
        public double UT;
        public WayPoint Target;
        public double MinPeR;
        public ManeuverPlanner.Config Config;
    }

    public abstract class ManeuverOperation
    {
        public abstract string Title { get; }
        public abstract string Description { get; }
        public virtual bool NeedsTarget => false;
        public virtual bool LongSearch => false;

        public abstract void DrawParameters(ManeuverPlannerContext context);
        public abstract ManeuverPlan BuildPlan(ManeuverPlannerContext context);

        protected static void DrawRow(string label, Action draw)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(label, GUILayout.ExpandWidth(true));
            draw();
            GUILayout.EndHorizontal();
        }

        protected static double BodyRadius(CelestialBody body, double altitude) => body.Radius + altitude;

        protected static void ValidateApsides(ManeuverPlan plan, Orbit orbit, double minPeR)
        {
            if(orbit.PeR < minPeR)
                plan.Warn(Loc.F("ManeuverPlanner_WarnLowPe", "Resulting periapsis is below the safe limit: <<1>>",
                    Utils.formatBigValue((float)(orbit.PeR - orbit.referenceBody.Radius), "m")));
            if(orbit.ApR > orbit.referenceBody.sphereOfInfluence)
                plan.Warn(Loc.T("ManeuverPlanner_WarnSoI", "Resulting apoapsis is outside the sphere of influence."));
            if(orbit.eccentricity >= 1)
                plan.Warn(Loc.T("ManeuverPlanner_WarnHyperbolic", "Resulting orbit is hyperbolic."));
        }

        protected static void ValidateDeltaV(ManeuverPlan plan, VesselWrapper vsl)
        {
            if(vsl.Engines.MaxDeltaV > 0 && plan.TotalDeltaV > vsl.Engines.MaxDeltaV)
                plan.Warn(Loc.F("ManeuverPlanner_WarnDeltaV",
                    "Plan needs <<1>>, but the vessel has about <<2>>.",
                    Utils.formatBigValue((float)plan.TotalDeltaV, "m/s"),
                    Utils.formatBigValue(vsl.Engines.MaxDeltaV, "m/s")));
        }
    }
}
