//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//

using System.Collections.Generic;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    public class InfoProps : VesselProps
    {
        public InfoProps(VesselWrapper vsl) : base(vsl) {}

        public bool ElectricChargeAvailible
        {
            get
            {
                if(CheatOptions.InfiniteElectricity) return true;
                double amount, max_amount;
                vessel.GetConnectedResourceTotals(Utils.ElectricCharge.id, out amount, out max_amount);
                return amount > 0;
            }
        }

        public Vector3 Destination;
        public double Countdown = -1;
        public float  TTB = -1;

        public HashSet<Vector3d> CustomMarkersVec = new HashSet<Vector3d>();
        public HashSet<WayPoint> CustomMarkersWP  = new HashSet<WayPoint>();

        public void AddCustopWaypoint(Vector3d pos, string name = null)
        {
            var wp = new WayPoint(pos, VSL.vessel.mainBody);
            wp.Name = name ?? Loc.T("Info_CustomWaypoint", "Custom WayPoint");
            CustomMarkersWP.Add(wp);
        }

        public void AddCustopWaypoint(Coordinates pos, string name = null)
        {
            var wp = new WayPoint(pos);
            wp.Name = name ?? Loc.T("Info_CustomWaypoint", "Custom WayPoint");
            CustomMarkersWP.Add(wp);
        }

        public override void ClearFrameState()
        {
            CustomMarkersWP.Clear();
            CustomMarkersVec.Clear();
            Destination = Vector3.zero;
            Countdown = -1;
            TTB = -1;
        }

        public override void Update() {}

        public void Draw()
        {
            if(!VSL.Controls.PhysicsReady)
            {
                if(VSL.Controls.PhysicsReadyCountdown > 0)
                    GUILayout.Label(new GUIContent(
                        string.Format(Loc.T("PhysicsReady_Cooldown", "Physics cooldown: {0:F1}s"),
                                      VSL.Controls.PhysicsReadyCountdown),
                        Loc.T("PhysicsReady_CooldownTip", "Waiting for physics to settle after time warp")),
                        Styles.danger, GUILayout.Width(180));
                else
                    GUILayout.Label(new GUIContent(
                        Loc.T("PhysicsReady_WaitWarp", "Waiting for time warp to end..."),
                        Loc.T("PhysicsReady_WaitWarpTip", "TCA is waiting for time warp to return to 1x")),
                        Styles.danger, GUILayout.Width(180));
            }
            GUILayout.Label(new GUIContent(VSL.Info.Countdown >= 0? 
                                           string.Format("{0:F1}s", VSL.Info.Countdown) : "", 
                                           Loc.T("Info_Countdown", "Countdown") ),
                            VSL.Info.Countdown > 10? Styles.white : Styles.danger, 
                            GUILayout.Width(90));
            GUILayout.Label(new GUIContent(VSL.Info.TTB >= 0 && VSL.Info.TTB < float.MaxValue? 
                                           string.Format("{0:F1}s", VSL.Info.TTB) : "",
                                           Loc.T("Info_ThrustDuration", "Thrust Duration")), 
                            Styles.active, GUILayout.Width(90));
        }
    }
}

