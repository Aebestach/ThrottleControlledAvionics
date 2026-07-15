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
    [CareerPart]
    [RequireModules(typeof(HorizontalSpeedControl), 
                    typeof(BearingControl))]
    [OptionalModules(typeof(AltitudeControl))]
    public class PointNavigator : TCAModule
    {
        public class Config : ComponentConfig<Config>
        {
            [Persistent] public float MinDistance          = 3;
            [Persistent] public float MinTime              = 10;
            [Persistent] public float OnPathMinDistance    = 10;
            [Persistent] public float MinSpeed             = 0;
            [Persistent] public float MaxSpeed             = 500;
            [Persistent] public float AngularAccelFactor   = 0.2f;

            [Persistent] public float DirectNavThreshold   = 1;
            [Persistent] public float GCNavStep            = 0.1f;
            [Persistent] public float LookAheadTime        = 2f;
            [Persistent] public float BearingCutoff        = 60f;
            [Persistent] public float FormationSpeedCutoff = 5f;
            [Persistent] public float FormationFactor      = 0.2f;
            [Persistent] public float FormationBreakTime   = 10f;
            [Persistent] public float FormationUpdateTimer = 60f;
            [Persistent] public float TakeoffAltitude      = 100f;
            [Persistent] public float BrakeOffset          = 1.5f;
            [Persistent] public float PitchRollAAf         = 100f;
            [Persistent] public float FollowerMaxAwaySpeed = 15f;

            [Persistent] public float MaxSpeedFilterUp     = 1000f;
            [Persistent] public float MaxSpeedFilterDown   = 100f;
            [Persistent] public float RotationAccelPhase   = 0.1f;
            [Persistent] public float CorrectionEasingRate = 0.8f;

            [Persistent] public PIDf_Controller DistancePID = new PIDf_Controller(0.5f, 0f, 0.5f, 0, 100);
            [Persistent] public PIDvd_Controller LateralPID = new PIDvd_Controller(0.5f, 0f, 0.5f, -100, 100);
            [Persistent] public PIDf_Controller CorrectionPID = new PIDf_Controller(0.5f, 0f, 0.5f, -100, 0);

            public float BearingCutoffCos;
            public float FormationSpeedSqr;

            public override void Init()
            {
                base.Init();
                DirectNavThreshold *= Mathf.Deg2Rad;
                GCNavStep *= Mathf.Deg2Rad;
                BearingCutoffCos = Mathf.Cos(BearingCutoff*Mathf.Deg2Rad);
                FormationSpeedSqr = FormationSpeedCutoff*FormationSpeedCutoff;
            }
        }
        public static Config C => Config.INST;

        public PointNavigator(ModuleTCA tca) : base(tca) {}

        public enum Stage
        {
            None,
            ResolveDestination,
            Takeoff,
            Navigate,
            GreatCircle,
            DirectApproach,
            SharpTurn,
            ArrivedWait,
            AdvancePath,
            HandoffLand,
            HandoffAnchor,
            Finished
        }

        [Persistent] public Stage stage;

        AsymmetricFiterF max_speed = new AsymmetricFiterF();
        readonly PIDf_Controller DistancePID = new PIDf_Controller();
        readonly PIDvd_Controller LateralPID = new PIDvd_Controller();
        readonly PIDf_Controller CorrectionPID = new PIDf_Controller();
        PredictiveBrakingController Braking;
        readonly Timer ArrivedTimer = new Timer();
        readonly Timer SharpTurnTimer = new Timer();
        bool verticalArrivalReady;

        #pragma warning disable 169
        HorizontalSpeedControl HSC;
        AltitudeControl ALT;
        AutoLander LND;
        Radar RAD;
        #pragma warning restore 169

        public override void Init()
        {
            base.Init();
            DistancePID.setPID(C.DistancePID);
            DistancePID.Reset();
            LateralPID.setPID(C.LateralPID);
            LateralPID.Reset();
            CorrectionPID.setPID(C.CorrectionPID);
            CorrectionPID.Reset();
            Braking = new PredictiveBrakingController(VSL, HSC);
            ArrivedTimer.Period = C.MinTime;
            CFG.Nav.AddCallback(GoToTargetCallback, Navigation.GoToTarget);
            CFG.Nav.AddCallback(FollowPathCallback, Navigation.FollowPath);
            max_speed.TauUp = C.MaxSpeedFilterUp;
            max_speed.TauDown = C.MaxSpeedFilterDown;
            max_speed.Set(CFG.MaxNavSpeed);
        }

        public override void Disable()
        {
            CFG.Nav.OffIfOn(Navigation.GoToTarget, Navigation.FollowPath);
        }

        protected override void UpdateState() 
        { 
            base.UpdateState();
            IsActive &= CFG.Target  && VSL.OnPlanet &&
                CFG.Nav.Any(Navigation.GoToTarget, Navigation.FollowPath);
        }

        public void GoToTargetCallback(Multiplexer.Command cmd)
        {
            switch(cmd)
            {
            case Multiplexer.Command.Resume:
                if(CFG.Target) 
                    start_to(CFG.Target);
                else finish();
                break;

            case Multiplexer.Command.On:
                var wp = VSL.TargetAsWP;
                if(wp == null) finish();
                else start_to(wp);
                break;

            case Multiplexer.Command.Off:
                finish(); break;
            }
        }

        public void FollowPathCallback(Multiplexer.Command cmd)
        {
            switch(cmd)
            {
            case Multiplexer.Command.Resume:
            case Multiplexer.Command.On:
                if(CFG.Path.Count > 0)
                    start_to(CFG.Path.Peek());
                else finish();
                break;

            case Multiplexer.Command.Off:
                finish(); 
                break;
            }
        }

        void start_to(WayPoint wp)
        {
            wp.Update(VSL);
            if(CFG.Nav[Navigation.GoToTarget] && wp.CloseEnough(VSL))
            { 
                CFG.Nav.Off(); 
                return; 
            }
            if(VSL.LandedOrSplashed) 
            {
                CFG.AltitudeAboveTerrain = true;
                CFG.VF.OnIfNot(VFlight.AltitudeControl);
                if(CFG.DesiredAltitude < C.TakeoffAltitude + VSL.Geometry.H)
                    CFG.DesiredAltitude = C.TakeoffAltitude + VSL.Geometry.H;
                stage = Stage.Takeoff;
            }                
            else if(CFG.VTOLAssistON) 
            {
                VSL.GearOn(false);
                stage = Stage.Navigate;
            }
            else stage = Stage.Navigate;
            max_speed.Set(CFG.MaxNavSpeed);
            SetTarget(wp);
            DistancePID.Reset();
            LateralPID.Reset();
            CorrectionPID.Reset();
            Braking?.Reset(CFG.MaxNavSpeed);
            verticalArrivalReady = false;
            CFG.HF.OnIfNot(HFlight.NoseOnCourse);
            NeedCPS();
        }

        void finish()
        {
            ReleaseCPS();
            SetTarget();
            stage = Stage.Finished;
            CFG.Nav.Off();
            CFG.HF.OnIfNot(HFlight.Stop);
        }

        private void anchor_at_target()
        {
            stage = Stage.HandoffAnchor;
            CFG.Anchor = CFG.Target;
            CFG.Nav.XOn(Navigation.Anchor);
        }

        bool on_arrival()
        {
            if(CFG.Target == null || !CFG.Target.Valid) return false;
            if(CFG.Target.Land && LND != null)    
            { 
                stage = Stage.HandoffLand;
                if(!CFG.Target.IsVessel)
                    LND.StartFromTarget();
                VSL.Controls.PauseWhenStopped = CFG.Target.Pause;
                CFG.Target.Pause = false;
                CFG.AP1.XOn(Autopilot1.Land);
                return true; 
            }
            if(CFG.Target.Pause) 
            { 
                stage = Stage.HandoffAnchor;
                CFG.Target.Pause = false;
                VSL.Controls.PauseWhenStopped = true;
                anchor_at_target();
                return true;
            }
            return false;
        }

        protected override void Update()
        {
            if(CFG.Nav.Paused) return;
            if(stage == Stage.None || stage == Stage.Finished)
                stage = Stage.ResolveDestination;
            if(stage == Stage.Takeoff)
            {
                CFG.HF.OnIfNot(HFlight.Level);
                VSL.HorizontalSpeed.SetNeeded(Vector3d.zero);
                if(!VSL.LandedOrSplashed && VSL.Altitude.Relative > C.TakeoffAltitude * 0.8f)
                {
                    stage = Stage.Navigate;
                    CFG.HF.OnIfNot(HFlight.NoseOnCourse);
                }
                else
                    return;
            }
            var vdistance = 0f; //vertical distance to the target
            VSL.Altitude.LowerThreshold = (float)CFG.Target.Pos.Alt;
            if(ALT != null && CFG.VF[VFlight.AltitudeControl] && CFG.AltitudeAboveTerrain)
                vdistance = VSL.Altitude.LowerThreshold+CFG.DesiredAltitude-VSL.Altitude.Absolute;
            //calculate direct distance
            var vdir = SurfaceNavigationController.TargetVector(VSL, CFG.Target, Vector3.zero);
            var hdistance = SurfaceNavigationController.HorizontalDistance(VSL, vdir);
            var bearing_threshold = SurfaceNavigationController.BearingThreshold(VSL); //10deg yaw error
            //update destination
            VSL.Info.Destination = vdir;
            var vel_is_set = false;
            var end_distance = CFG.Target.AbsRadius;
            var dvel = VSL.HorizontalSpeed.Vector;
            if(CFG.Target.Land) end_distance /= 4;
            //if the distance is greater that the threshold (in radians), use the Great Circle navigation
            if(hdistance/VSL.Body.Radius > C.DirectNavThreshold)
            {
                stage = Stage.GreatCircle;
                vdir = SurfaceNavigationController.GreatCircleDirection(VSL, CFG.Target, out hdistance);
            }
            else if(!VSL.IsActiveVessel && hdistance > GLB.UnpackDistance) 
            {
                stage = Stage.DirectApproach;
                VSL.SetUnpackDistance(hdistance*1.2f);
            }
            else if(stage != Stage.SharpTurn)
                stage = Stage.DirectApproach;
            vdir.Normalize();
            var vertical_tolerance = Mathf.Max(VSL.Geometry.R, C.MinDistance);
            if(!CFG.AltitudeAboveTerrain || !CFG.VF[VFlight.AltitudeControl])
                verticalArrivalReady = true;
            else if(verticalArrivalReady)
                verticalArrivalReady &= vdistance <= Mathf.Max(vertical_tolerance * 1.5f, vertical_tolerance + 2);
            else
                verticalArrivalReady = vdistance <= vertical_tolerance;
            var vertically_ready = verticalArrivalReady;
            //check if we have arrived to the target and stayed long enough
            if(hdistance < end_distance)
            {
                stage = Stage.ArrivedWait;
                if(vertically_ready)
                    CFG.HF.OnIfNot(HFlight.Move);
                else
                    CFG.HF.OnIfNot(VSL.LandedOrSplashed ? HFlight.Level : HFlight.NoseOnCourse);
                if(vertically_ready)
                    VSL.Altitude.DontCorrectIfSlow();
                VSL.HorizontalSpeed.SetNeeded(Vector3d.zero);
                vel_is_set = true;
                if(vertically_ready && vdistance <= 0 && vdistance > -VSL.Geometry.R)
                {
                    if(CFG.Nav[Navigation.FollowPath] && CFG.Path.Count > 0)
                    {
                        if(CFG.Path.Peek() == CFG.Target)
                        {
                            if(CFG.Path.Count > 1)
                            {
                                stage = Stage.AdvancePath;
                                CFG.Path.Dequeue();
                                if(on_arrival()) return;
                                start_to(CFG.Path.Peek());
                                return;
                            }
                            if(ArrivedTimer.TimePassed)
                            {
                                stage = Stage.AdvancePath;
                                CFG.Path.Clear();
                                if(on_arrival()) return;
                                anchor_at_target();
                                return;
                            }
                        }
                        else
                        {
                            stage = Stage.AdvancePath;
                            if(on_arrival()) return;
                            start_to(CFG.Path.Peek());
                            return;
                        }
                    }
                    else if(ArrivedTimer.TimePassed)
                    {
                        if(on_arrival()) return;
                        finish();
                        return;
                    }
                }
            }
            else 
            {
                ArrivedTimer.Reset();
                CFG.HF.OnIfNot(HFlight.NoseOnCourse);
                //if we need to make a sharp turn, stop and turn, then go on
                var heading_dir = Vector3.Dot(VSL.OnPlanetParams.Heading, vdir);
                var hvel_dir = Vector3d.Dot(VSL.HorizontalSpeed.normalized, vdir);
                const bool sharp_turn_allowed = true;
                if(heading_dir < bearing_threshold && 
                   hvel_dir < bearing_threshold &&
                   sharp_turn_allowed)
                    SharpTurnTimer.Start();
                if(SharpTurnTimer.Started)
                {
                    stage = Stage.SharpTurn;
                    VSL.HorizontalSpeed.SetNeeded(vdir);
                    vel_is_set = true;
                    if(heading_dir < bearing_threshold || sharp_turn_allowed &&
                       VSL.HorizontalSpeed.Absolute > 1 && Math.Abs(hvel_dir) < C.BearingCutoffCos)
                        SharpTurnTimer.Restart();
                    else if(SharpTurnTimer.TimePassed) 
                        SharpTurnTimer.Reset();
                }
//                Log("timer: {}\nheading*dir {} < {}, vel {} > 1, vel*dir {} < {}",
//                    SharpTurnTimer,
//                    Vector3.Dot(VSL.OnPlanetParams.Heading, vdir), bearing_threshold,
//                    VSL.HorizontalSpeed.Absolute, Vector3d.Dot(VSL.HorizontalSpeed.normalized, vdir), bearing_threshold);//debug
            }
            var cur_vel = (float)Vector3d.Dot(dvel, vdir);
            if(!vel_is_set)
            {
                //don't slow down on intermediate waypoints too much
                var min_dist = C.OnPathMinDistance*VSL.Geometry.R;
                if(!CFG.Target.Land && CFG.Nav[Navigation.FollowPath] && 
                   CFG.Path.Count > 1 && hdistance < min_dist)
                {
                    WayPoint next_wp = null;
                    if(CFG.Path.Peek() == CFG.Target)
                    {
                        using(var iwp = CFG.Path.GetEnumerator())
                        {
                            if(iwp.MoveNext()
                               && iwp.MoveNext())
                                next_wp = iwp.Current;
                        }
                    }
                    else next_wp = CFG.Path.Peek();
                    if(next_wp != null)
                    {
                        next_wp.Update(VSL);
                        var next_dist = Vector3.ProjectOnPlane(next_wp.GetTransform().position-CFG.Target.GetTransform().position, VSL.Physics.Up);
                        var angle2next = Utils.Angle2(vdir, next_dist);
                        var minD = Utils.ClampL(min_dist*(1-angle2next/180/VSL.Torque.MaxPitchRoll.AA_rad*C.PitchRollAAf), CFG.Target.AbsRadius);
                        if(minD > hdistance) hdistance = minD;
                    }
                    else hdistance = min_dist;
                }
                else
                    hdistance = Utils.ClampL(hdistance-end_distance+VSL.Geometry.D, 0);
                //take into account vertical distance and obstacle
                var rel_ahead = VSL.Altitude.Ahead-VSL.Altitude.Absolute;
//                Log("vdist {}, rel.ahead {}, vF {}, aF {}", vdistance, rel_ahead,
//                    Utils.ClampL(1 - Mathf.Atan(vdistance/hdistance)/Utils.HalfPI, 0),
//                    Utils.ClampL(1 - rel_ahead/RAD.DistanceAhead, 0));//debug
                vdistance = Mathf.Max(vdistance, rel_ahead);
                if(CFG.MaxNavSpeed < 10) CFG.MaxNavSpeed = 10;
                var targetVelocity = CFG.Target != null ? (Vector3d)CFG.Target.GetSrfVelocity() : Vector3d.zero;
                if(Braking != null)
                {
                    Braking.Apply(vdir * hdistance,
                        targetVelocity,
                        CFG.MaxNavSpeed,
                        0,
                        vdistance,
                        RAD != null && rel_ahead > 0 && RAD.DistanceAhead > 0 ? rel_ahead : 0);
                    return;
                }
                DistancePID.Update(hdistance);
                VSL.HorizontalSpeed.SetNeeded(vdir*DistancePID.Action);
            }
//            Log("\ndir v {}\nlat v {}\nact v {}\nlatPID {}", 
//                 VSL.HorizontalSpeed.NeededVector, latV,
//                 LateralPID.Action, LateralPID);//debug
        }

    }
}

