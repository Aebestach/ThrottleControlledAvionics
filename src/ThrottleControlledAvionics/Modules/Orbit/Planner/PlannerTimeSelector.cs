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
    public enum PlannerTimeReference
    {
        Apoapsis,
        Periapsis,
        Now,
        FixedOffset,
        Altitude,
        AscendingNode,
        DescendingNode,
        CheapestNode
    }

    public class PlannerTimeSelector : ConfigNodeObject
    {
        [Persistent] public PlannerTimeReference Reference = PlannerTimeReference.Apoapsis;
        [Persistent] public double Offset = 60;
        [Persistent] public double Altitude = 100000;

        public void Draw(bool includeNodes = true, bool includeAltitude = true)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(Loc.T("ManeuverPlanner_Time", "Time:"), GUILayout.ExpandWidth(false));
            var choice = Utils.LeftRightChooser(ReferenceName(Reference), ReferenceDescription(Reference));
            if(choice > 0)
                Next(includeNodes, includeAltitude);
            else if(choice < 0)
                Previous(includeNodes, includeAltitude);
            GUILayout.EndHorizontal();
            if(Reference == PlannerTimeReference.FixedOffset)
                Offset = NumericField(
                    Loc.T("ManeuverPlanner_Offset", "Offset:"),
                    Offset,
                    "s",
                    Loc.T("ManeuverPlanner_Offset_Tip", "Delay after the current time before executing the maneuver."));
            if(Reference == PlannerTimeReference.Altitude)
                Altitude = NumericField(
                    Loc.T("ManeuverPlanner_Altitude", "Altitude:"),
                    Altitude,
                    "m",
                    Loc.T("ManeuverPlanner_Altitude_Tip", "Cross this altitude on the way up or down."));
        }

        void Next(bool includeNodes, bool includeAltitude)
        {
            do Reference = (PlannerTimeReference)(((int)Reference + 1) %
                                                   Enum.GetValues(typeof(PlannerTimeReference)).Length);
            while(!Allowed(Reference, includeNodes, includeAltitude));
        }

        void Previous(bool includeNodes, bool includeAltitude)
        {
            do
            {
                var n = (int)Reference - 1;
                if(n < 0) n = Enum.GetValues(typeof(PlannerTimeReference)).Length - 1;
                Reference = (PlannerTimeReference)n;
            }
            while(!Allowed(Reference, includeNodes, includeAltitude));
        }

        static bool Allowed(PlannerTimeReference reference, bool includeNodes, bool includeAltitude)
        {
            if(!includeAltitude && reference == PlannerTimeReference.Altitude)
                return false;
            if(!includeNodes &&
               (reference == PlannerTimeReference.AscendingNode ||
                reference == PlannerTimeReference.DescendingNode ||
                reference == PlannerTimeReference.CheapestNode))
                return false;
            return true;
        }

        public double ComputeUT(ManeuverPlannerContext context)
        {
            var orbit = context.Orbit;
            switch(Reference)
            {
            case PlannerTimeReference.Apoapsis:
                return NextApsideUT(orbit, context.UT, true);
            case PlannerTimeReference.Periapsis:
                return NextApsideUT(orbit, context.UT, false);
            case PlannerTimeReference.FixedOffset:
                return context.UT + Math.Max(0, Offset);
            case PlannerTimeReference.Altitude:
                return ManeuverPlannerMath.NextRadiusUT(orbit, context.UT, context.VSL.Body.Radius + Altitude);
            case PlannerTimeReference.AscendingNode:
                return ManeuverPlannerMath.NextEquatorialNodeUT(orbit, context.UT, true);
            case PlannerTimeReference.DescendingNode:
                return ManeuverPlannerMath.NextEquatorialNodeUT(orbit, context.UT, false);
            case PlannerTimeReference.CheapestNode:
                return ManeuverPlannerMath.CheapestEquatorialNodeUT(orbit, context.UT);
            default:
                return context.UT;
            }
        }

        public static double NumericField(string label, double value, string suffix, string tooltip = null)
        {
            GUILayout.BeginHorizontal();
            if(string.IsNullOrEmpty(tooltip))
                GUILayout.Label(label, GUILayout.ExpandWidth(true));
            else
                GUILayout.Label(new GUIContent(label, tooltip), GUILayout.ExpandWidth(true));
            var text = GUILayout.TextField(value.ToString("G6"), GUILayout.Width(90));
            if(double.TryParse(text, out var parsed))
                value = parsed;
            GUILayout.Label(suffix, GUILayout.Width(35));
            GUILayout.EndHorizontal();
            return value;
        }

        public static string ReferenceName(PlannerTimeReference reference)
        {
            switch(reference)
            {
            case PlannerTimeReference.Apoapsis:
                return Loc.T("ManeuverPlanner_TimeAp", "Apoapsis");
            case PlannerTimeReference.Periapsis:
                return Loc.T("ManeuverPlanner_TimePe", "Periapsis");
            case PlannerTimeReference.FixedOffset:
                return Loc.T("ManeuverPlanner_TimeOffset", "Offset");
            case PlannerTimeReference.Altitude:
                return Loc.T("ManeuverPlanner_TimeAltitude", "Altitude");
            case PlannerTimeReference.AscendingNode:
                return Loc.T("ManeuverPlanner_TimeAN", "Ascending Node");
            case PlannerTimeReference.DescendingNode:
                return Loc.T("ManeuverPlanner_TimeDN", "Descending Node");
            case PlannerTimeReference.CheapestNode:
                return Loc.T("ManeuverPlanner_TimeCheapNode", "Cheapest Node");
            default:
                return Loc.T("ManeuverPlanner_TimeNow", "Now");
            }
        }

        static string ReferenceDescription(PlannerTimeReference reference) =>
            Loc.F("ManeuverPlanner_TimeTip", "Create the node at <<1>>.", ReferenceName(reference));

        static double NextApsideUT(Orbit orbit, double fromUT, bool apoapsis)
        {
            if(orbit.eccentricity >= 1)
                return fromUT;
            var ut = orbit.StartUT + (apoapsis ? orbit.timeToAp : orbit.timeToPe);
            while(ut < fromUT)
                ut += orbit.period;
            return ut;
        }
    }
}
