//   OrbitalTab.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2017 Allis Tauri

using UnityEngine;
using AT_Utils;


namespace ThrottleControlledAvionics
{
    public class OrbitalTab : ControlTab
    {
        public OrbitalTab(ModuleTCA tca) : base(tca) {}

        MatchVelocityAutopilot MVA;
        ManeuverPlanner MPL;
        DeorbitAutopilot DEO;
        RendezvousAutopilot REN;
        ToOrbitAutopilot ORB;
        [InternalModule]
        PointNavigator PN;

        public override void OnRenderObject()
        {
            if(DEO != null)
                DEO.DrawTrajectory();
        }

        public override void Draw()
        {
            GUILayout.BeginHorizontal();
            if(MVA != null) MVA.Draw();
            else Utils.EnsureLayoutControl();
            GUILayout.EndHorizontal();
            if(PN  != null && UI.NAV != null) 
                UI.NAV.TargetUI();
            GUILayout.BeginHorizontal();
            var drew_orbital = false;
            if(MPL != null) { MPL.Draw(); drew_orbital = true; }
            if(ORB != null) { ORB.Draw(); drew_orbital = true; }
            if(REN != null) { REN.Draw(); drew_orbital = true; }
            if(DEO != null) { DEO.Draw(); drew_orbital = true; }
            if(!drew_orbital) Utils.EnsureLayoutControl();
            GUILayout.EndHorizontal();
            if(MPL != null && MPL.ShowOptions && MPL.ControlsActive)
                MPL.DrawOptions();
            if(ORB != null && ORB.ShowOptions && ORB.ControlsActive)
                ORB.DrawOptions();
            if(REN != null && REN.ShowOptions && REN.ControlsActive)
            {
                REN.DrawOptions();
                REN.DrawBestTrajectories();
            }
            if(DEO != null && DEO.ShowOptions && DEO.ControlsActive)
                DEO.DrawOptions();
            #if DEBUG
            if(Utils.ButtonSwitch("DBG", ref TrajectoryCalculator.setp_by_step_computation, 
                                  "Toggles step-by-step trajectory computation", GUILayout.ExpandWidth(true)) &&
               TrajectoryCalculator.setp_by_step_computation)
                MapView.EnterMapView();
            #endif
        }
    }
}