//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Collections.Generic;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    [CareerPart(typeof(PointNavigator))]
    [RequireModules(typeof(HorizontalSpeedControl),
                    typeof(BearingControl))]
    [OptionalModules(typeof(AltitudeControl))]
    public class FollowAutopilot : TCAModule
    {
        public enum Stage
        {
            None,
            ResolveTarget,
            AssignFormationSlot,
            ApproachSlot,
            MatchVelocity,
            KeepFormation,
            RecoverFormation,
            Finished
        }

        public static PointNavigator.Config C => PointNavigator.C;

        [Persistent] public Stage stage;

        public bool Maneuvering { get; private set; }
        public List<FormationNode> Formation;

        readonly AsymmetricFiterF max_speed = new AsymmetricFiterF();
        readonly PIDf_Controller DistancePID = new PIDf_Controller();
        readonly PIDvd_Controller LateralPID = new PIDvd_Controller();
        readonly PIDf_Controller CorrectionPID = new PIDf_Controller();
        readonly Timer SharpTurnTimer = new Timer();

        Vessel tVSL;
        ModuleTCA tTCA;
        PointNavigator tPN;
        readonly SortedList<Guid, FollowAutopilot> all_followers = new SortedList<Guid, FollowAutopilot>();
        FormationNode fnode;
        Vector3 formation_offset => fnode == null ? Vector3.zero : fnode.Offset;
        bool CanManeuver = true;
        readonly Timer FormationBreakTimer = new Timer();
        readonly Timer FormationUpdateTimer = new Timer();
        bool keep_formation;

        #pragma warning disable 169
        HorizontalSpeedControl HSC;
        AltitudeControl ALT;
        Radar RAD;
        #pragma warning restore 169

        public FollowAutopilot(ModuleTCA tca) : base(tca) {}

        public override void Init()
        {
            base.Init();
            DistancePID.setPID(C.DistancePID);
            DistancePID.Reset();
            LateralPID.setPID(C.LateralPID);
            LateralPID.Reset();
            CorrectionPID.setPID(C.CorrectionPID);
            CorrectionPID.Reset();
            FormationBreakTimer.Period = C.FormationBreakTime;
            FormationUpdateTimer.Period = C.FormationUpdateTimer;
            CFG.Nav.AddCallback(FollowTargetCallback, Navigation.FollowTarget);
            max_speed.TauUp = C.MaxSpeedFilterUp;
            max_speed.TauDown = C.MaxSpeedFilterDown;
            max_speed.Set(CFG.MaxNavSpeed);
        }

        public override void Disable()
        {
            CFG.Nav.OffIfOn(Navigation.FollowTarget);
        }

        protected override void UpdateState()
        {
            base.UpdateState();
            IsActive &= CFG.Target && VSL.OnPlanet && CFG.Nav[Navigation.FollowTarget];
        }

        public void FollowTargetCallback(Multiplexer.Command cmd)
        {
            switch(cmd)
            {
            case Multiplexer.Command.Resume:
                if(CFG.Target)
                    start_follow(CFG.Target);
                else finish();
                break;

            case Multiplexer.Command.On:
                var wp = VSL.ResolveTarget() ?? VSL.TargetAsWP;
                if(wp == null) finish();
                else start_follow(wp);
                break;

            case Multiplexer.Command.Off:
                finish();
                break;
            }
        }

        void start_follow(WayPoint wp)
        {
            wp.Update(VSL);
            if(VSL.LandedOrSplashed)
            {
                CFG.AltitudeAboveTerrain = true;
                CFG.VF.OnIfNot(VFlight.AltitudeControl);
                CFG.DesiredAltitude = C.TakeoffAltitude + VSL.Geometry.H;
            }
            else if(CFG.VTOLAssistON)
                VSL.GearOn(false);
            max_speed.Set(CFG.MaxNavSpeed);
            reset_formation();
            SetTarget(wp);
            DistancePID.Reset();
            LateralPID.Reset();
            CorrectionPID.Reset();
            CFG.HF.OnIfNot(HFlight.NoseOnCourse);
            NeedCPS();
            stage = Stage.ResolveTarget;
        }

        void finish()
        {
            ReleaseCPS();
            reset_formation();
            SetTarget();
            stage = Stage.Finished;
            CFG.Nav.OffIfOn(Navigation.FollowTarget);
            CFG.HF.OnIfNot(HFlight.Stop);
        }

        void reset_formation()
        {
            Maneuvering = false;
            Formation = null;
            fnode = null;
            tVSL = null;
            tTCA = null;
            tPN = null;
            all_followers.Clear();
            keep_formation = false;
            CanManeuver = true;
        }

        public void UpdateFormation(List<FormationNode> formation)
        {
            if(formation == null) reset_formation();
            else { Formation = formation; fnode = null; }
        }

        void update_formation_info()
        {
            stage = Stage.ResolveTarget;
            tVSL = CFG.Target.GetVessel();
            if(tVSL == null)
            {
                reset_formation();
                CanManeuver = false;
                return;
            }
            if(tPN == null || !tPN.Valid)
            {
                tTCA = ModuleTCA.EnabledTCA(tVSL);
                tPN = tTCA != null ? tTCA.GetModule<PointNavigator>() : null;
            }
            var only_count = false;
            if(tVSL.srf_velocity.sqrMagnitude < C.FormationSpeedSqr)
            {
                reset_formation();
                CanManeuver = false;
                only_count = true;
            }
            var offset = 0f;
            var can_maneuver = true;
            all_followers.Clear();
            for(int i = 0, num_vessels = FlightGlobals.Vessels.Count; i < num_vessels; i++)
            {
                var v = FlightGlobals.Vessels[i];
                if(v == null || v.packed || !v.loaded) continue;
                var tca = ModuleTCA.EnabledTCA(v);
                if(tca != null &&
                   (tca.vessel == VSL.vessel ||
                    tca.CFG.Nav[Navigation.FollowTarget] &&
                    tca.CFG.Target.GetTarget() != null &&
                    tca.CFG.Target.GetTarget() == CFG.Target.GetTarget()))
                {
                    var follower = tca.GetModule<FollowAutopilot>();
                    if(follower == null) continue;
                    all_followers.Add(v.id, follower);
                    if(offset < follower.VSL.Geometry.R)
                        offset = follower.VSL.Geometry.R;
                    if(v.id != VSL.vessel.id)
                        can_maneuver &= !follower.Maneuvering ||
                                        (Maneuvering && VSL.vessel.id.CompareTo(v.id) > 0);
                }
            }
            if(only_count) return;
            CanManeuver = can_maneuver;
            var follower_index = all_followers.IndexOfKey(VSL.vessel.id);
            if(follower_index == 0)
            {
                var forward = tVSL == null ? Vector3d.zero : -tVSL.srf_velocity.normalized;
                var side = Vector3d.Cross(VSL.Physics.Up, forward).normalized;
                var num_offsets = all_followers.Count + (all_followers.Count % 2);
                offset *= 2;
                var target_size = tTCA != null ? tTCA.VSL.Geometry.D : Utils.ClampL(Math.Pow(tVSL.totalMass, 1 / 3f), 1);
                if(offset < target_size) offset = (float)target_size;
                offset *= C.MinDistance;
                if(Formation == null || Formation.Count != num_offsets || FormationUpdateTimer.TimePassed)
                {
                    FormationUpdateTimer.Reset();
                    Formation = new List<FormationNode>(num_offsets);
                    for(int i = 0; i < num_offsets; i++)
                        Formation.Add(new FormationNode(tVSL, i, forward, side, offset));
                    all_followers.ForEach(p => p.Value.UpdateFormation(Formation));
                }
                else for(int i = 0; i < num_offsets; i++)
                    Formation[i].Update(forward, side, offset);
            }
            keep_formation = Formation != null;
            if(Formation == null || fnode != null) return;
            stage = Stage.AssignFormationSlot;
            var min_d = -1f;
            var min_off = 0;
            for(int i = 0; i < Formation.Count; i++)
            {
                var node = Formation[i];
                if(node.Follower != null) continue;
                var d = node.Distance(VSL.vessel);
                if(min_d < 0 || min_d > d)
                {
                    min_d = d;
                    min_off = i;
                }
            }
            Formation[min_off].Follower = VSL.vessel;
            fnode = Formation[min_off];
        }

        protected override void Update()
        {
            if(CFG.Nav.Paused) return;
            update_formation_info();
            var vdistance = 0f;
            var vdir = SurfaceNavigationController.TargetVector(VSL, CFG.Target, formation_offset);
            var hdistance = SurfaceNavigationController.HorizontalDistance(VSL, vdir);
            var bearing_threshold = SurfaceNavigationController.BearingThreshold(VSL);
            if(tPN != null && tPN.Valid && !tPN.VSL.Info.Destination.IsZero())
                VSL.Info.Destination = tPN.VSL.Info.Destination;
            else VSL.Info.Destination = vdir;

            var tvel = Vector3.zero;
            var vel_is_set = false;
            var end_distance = CFG.Target.AbsRadius;
            var dvel = VSL.HorizontalSpeed.Vector;
            if(tVSL != null && tVSL.loaded)
            {
                if(formation_offset.IsZero()) end_distance *= all_followers.Count / 2f;
                tvel = Vector3d.Exclude(VSL.Physics.Up, tVSL.srf_velocity);
                dvel -= tvel;
                var tvel_m = tvel.magnitude;
                var dir2vel_cos = Vector3.Dot(vdir.normalized, tvel.normalized);
                var lat_dir = Vector3.ProjectOnPlane(vdir - VSL.HorizontalSpeed.Vector * C.LookAheadTime, tvel);
                var lat_dist = lat_dir.magnitude;
                FormationBreakTimer.RunIf(() => keep_formation = false,
                                          tvel_m < C.FormationSpeedCutoff);
                Maneuvering = CanManeuver && lat_dist > CFG.Target.AbsRadius && hdistance < CFG.Target.AbsRadius * 3;
                if(keep_formation && tvel_m > 0 &&
                   (!CanManeuver ||
                    dir2vel_cos <= bearing_threshold ||
                    lat_dist < CFG.Target.AbsRadius * 3))
                {
                    stage = Maneuvering ? Stage.RecoverFormation : Stage.KeepFormation;
                    if(CanManeuver)
                        HSC.AddWeightedCorrection(lat_dir.normalized * Utils.ClampH(lat_dist / CFG.Target.AbsRadius, 1) *
                                                  tvel_m * C.FormationFactor * (Maneuvering ? 1 : 0.5f));
                    hdistance = Utils.ClampL(Mathf.Abs(dir2vel_cos) * hdistance - VSL.Geometry.R, 0);
                    if(dir2vel_cos < 0)
                    {
                        if(hdistance < CFG.Target.AbsRadius)
                            HSC.AddRawCorrection(tvel * Utils.Clamp(-hdistance / CFG.Target.AbsRadius * C.FormationFactor,
                                                                    -C.FormationFactor, 0));
                        else if(Vector3.Dot(vdir, dvel) < 0 &&
                                (dvel.magnitude > C.FollowerMaxAwaySpeed ||
                                 hdistance > CFG.Target.AbsRadius * 5))
                        {
                            keep_formation = true;
                            VSL.HorizontalSpeed.SetNeeded(vdir);
                            return;
                        }
                        else HSC.AddRawCorrection(tvel * (C.FormationFactor - 1));
                        hdistance = 0;
                    }
                    vdir = tvel;
                }
            }
            if(hdistance / VSL.Body.Radius > C.DirectNavThreshold)
            {
                vdir = SurfaceNavigationController.GreatCircleDirection(VSL, CFG.Target, out hdistance);
                tvel = Vector3.zero;
            }
            else if(!VSL.IsActiveVessel && hdistance > GLB.UnpackDistance)
                VSL.SetUnpackDistance(hdistance * 1.2f);
            vdir.Normalize();

            if(hdistance < end_distance)
            {
                stage = Stage.MatchVelocity;
                var prev_needed_speed = VSL.HorizontalSpeed.NeededVector.magnitude;
                if(prev_needed_speed < 1 && !CFG.HF[HFlight.Move])
                    CFG.HF.OnIfNot(HFlight.Move);
                else if(prev_needed_speed > 10 && !CFG.HF[HFlight.NoseOnCourse])
                    CFG.HF.OnIfNot(HFlight.NoseOnCourse);
                if(tvel.sqrMagnitude > 1)
                {
                    keep_formation = true;
                    VSL.HorizontalSpeed.SetNeeded(tvel);
                    HSC.AddRawCorrection((tvel - VSL.HorizontalSpeed.Vector) * 0.9f);
                }
                else
                    VSL.HorizontalSpeed.SetNeeded(Vector3d.zero);
                vel_is_set = true;
            }
            else
            {
                stage = fnode == null ? Stage.ApproachSlot : Stage.KeepFormation;
                CFG.HF.OnIfNot(HFlight.NoseOnCourse);
                var heading_dir = Vector3.Dot(VSL.OnPlanetParams.Heading, vdir);
                var hvel_dir = Vector3d.Dot(VSL.HorizontalSpeed.normalized, vdir);
                var sharp_turn_allowed = hdistance < end_distance * Utils.ClampL(all_followers.Count / 2, 2);
                if(heading_dir < bearing_threshold &&
                   hvel_dir < bearing_threshold &&
                   sharp_turn_allowed)
                    SharpTurnTimer.Start();
                if(SharpTurnTimer.Started)
                {
                    stage = Stage.RecoverFormation;
                    VSL.HorizontalSpeed.SetNeeded(vdir);
                    Maneuvering = false;
                    vel_is_set = true;
                    if(heading_dir < bearing_threshold || sharp_turn_allowed &&
                       VSL.HorizontalSpeed.Absolute > 1 && Math.Abs(hvel_dir) < C.BearingCutoffCos)
                        SharpTurnTimer.Restart();
                    else if(SharpTurnTimer.TimePassed)
                        SharpTurnTimer.Reset();
                }
            }
            var cur_vel = (float)Vector3d.Dot(dvel, vdir);
            if(!vel_is_set)
            {
                if(CFG.MaxNavSpeed < 10) CFG.MaxNavSpeed = 10;
                DistancePID.Min = HorizontalSpeedControl.C.TranslationMinDeltaV + 0.1f;
                DistancePID.Max = CFG.MaxNavSpeed;
                DistancePID.P = C.DistancePID.P / 2;
                DistancePID.D = DistancePID.P / 2;
                if(cur_vel > 0)
                {
                    var mg2 = VSL.Physics.mg * VSL.Physics.mg;
                    var brake_thrust = Mathf.Min(VSL.Physics.mg, VSL.Engines.MaxThrustM / 2 * VSL.OnPlanetParams.TWRf);
                    var max_thrust = Mathf.Min(Mathf.Sqrt(brake_thrust * brake_thrust + mg2), VSL.Engines.MaxThrustM * 0.99f);
                    var horizontal_thrust = VSL.Engines.TranslationThrustLimits.Project(VSL.LocalDir(vdir)).magnitude;
                    if(horizontal_thrust > brake_thrust) brake_thrust = horizontal_thrust;
                    else horizontal_thrust = -1;
                    if(brake_thrust > 0)
                    {
                        var brake_accel = brake_thrust / VSL.Physics.M;
                        var prep_time = 0f;
                        if(horizontal_thrust < 0)
                        {
                            var brake_angle = Utils.Angle2(VSL.Engines.CurrentDefThrustDir, vdir) - 45;
                            if(brake_angle > 0)
                            {
                                var axis = Vector3.Cross(VSL.Engines.CurrentDefThrustDir, vdir);
                                if(VSL.Torque.Slow)
                                {
                                    prep_time = VSL.Torque.NoEngines.RotationTime3Phase(brake_angle, axis, C.RotationAccelPhase);
                                    prep_time += Utils.LerpTime(VSL.Engines.Thrust.magnitude, VSL.Engines.MaxThrustM, max_thrust, VSL.Engines.AccelerationSpeed);
                                }
                                else
                                    prep_time = VSL.Torque.MaxCurrent.RotationTime2Phase(brake_angle, axis, VSL.OnPlanetParams.GeeVSF);
                            }
                        }
                        var prep_dist = cur_vel * prep_time + CFG.Target.AbsRadius;
                        var eta = hdistance / cur_vel;
                        max_speed.TauUp = C.MaxSpeedFilterUp / eta / brake_accel;
                        max_speed.TauDown = eta * brake_accel / C.MaxSpeedFilterDown;
                        max_speed.Update(prep_dist < hdistance
                                             ? (1 + Mathf.Sqrt(1 + 2 / brake_accel * (hdistance - prep_dist))) * brake_accel
                                             : 2 * brake_accel);
                        CorrectionPID.Min = -VSL.HorizontalSpeed.Absolute;
                        if(max_speed < cur_vel)
                            CorrectionPID.Update(max_speed - cur_vel);
                        else
                        {
                            CorrectionPID.IntegralError *= (1 - TimeWarp.fixedDeltaTime * C.CorrectionEasingRate);
                            CorrectionPID.Update(0);
                        }
                        HSC.AddRawCorrection(CorrectionPID.Action * VSL.HorizontalSpeed.Vector.normalized);
                    }
                    if(max_speed < CFG.MaxNavSpeed)
                        DistancePID.Max = Mathf.Max(DistancePID.Min, max_speed);
                }
                var rel_ahead = VSL.Altitude.Ahead - VSL.Altitude.Absolute;
                vdistance = Mathf.Max(vdistance, rel_ahead);
                if(vdistance > 0)
                    hdistance *= Utils.ClampL(1 - Mathf.Atan(vdistance / hdistance) / (float)Utils.HalfPI, 0);
                if(RAD != null && rel_ahead > 0 && RAD.DistanceAhead > 0)
                    hdistance *= Utils.ClampL(1 - rel_ahead / RAD.DistanceAhead, 0);
                DistancePID.Update(hdistance);
                var nV = vdir * DistancePID.Action;
                if(Vector3d.Dot(tvel, vdir) > 0) nV += tvel;
                VSL.HorizontalSpeed.SetNeeded(nV);
            }
            var latV = -Vector3d.Exclude(vdir, VSL.HorizontalSpeed.Vector);
            var latF = (float)Math.Min((latV.magnitude / Math.Max(VSL.HorizontalSpeed.Absolute, 0.1)), 1);
            LateralPID.P = C.LateralPID.P * latF;
            LateralPID.I = Math.Min(C.LateralPID.I, latF);
            LateralPID.D = C.LateralPID.D * latF;
            LateralPID.Update(latV);
            HSC.AddWeightedCorrection(LateralPID.Action);
        }

#if DEBUG
        public void RadarBeam()
        {
            if(VSL == null || VSL.vessel == null) return;
            if(CFG.Target && CFG.Target.GetTransform() != null && CFG.Nav[Navigation.FollowTarget])
                Utils.GLLine(VSL.Physics.wCoM, CFG.Target.GetTransform().position + formation_offset,
                             Maneuvering ? Color.yellow : Color.cyan);
        }
#endif
    }

    public class FormationNode
    {
        public readonly Vessel Target;
        public readonly int Index;
        public Vector3 Offset { get; private set; }
        public Vessel Follower;

        public FormationNode(Vessel target, int i, Vector3 forward, Vector3 side, float dist)
        {
            Index = i + 1;
            Target = target;
            Update(forward, side, dist);
        }

        public void Update(Vector3 forward, Vector3 side, float dist)
        {
            if(Index % 2 == 0) Offset = (forward + side) * dist * Index / 2;
            else Offset = (forward - side) * dist * (Index + 1) / 2;
        }

        public float Distance(Vessel vsl)
        { return (vsl.transform.position - Target.transform.position - Offset).magnitude; }

        public override string ToString()
        {
            return string.Format("[{0}]: Target {1}, Follower {2}, Offset {3}",
                                 Index, Target.vesselName, Follower == null ? "empty" : Follower.vesselName, Offset);
        }
    }
}
