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
    [CareerPart(typeof(ManeuverAutopilot))]
    public class TimeWarpControl : TCAModule
    {
        public class Config : ComponentConfig<Config>
        {
            [Persistent] public float DewarpTime        = 20f;  //sec, safety margin subtracted from warp target
            [Persistent] public float MaxWarp           = 10000f;
            [Persistent] public int   FramesToSkip      = 3;
            [Persistent] public bool  UseQuickWarp      = true;
            [Persistent] public float WarpIncreaseDelay = 2f;   //sec game time between warp rate increases
            [Persistent] public float WarpRateTolerance   = 0.02f; //relative
            [Persistent] public float PostWarpSettleTime  = 5f;    //sec
        }
        public static Config C => Config.INST;

        public TimeWarpControl(ModuleTCA tca) : base(tca) {}

        protected override bool UpdateRequiresPhysicsReady => false;

        int last_warp_index;
        int last_asked_index;
        int frames_to_skip;
        double warp_increase_attempt_time;
        bool require_settle_after_warp;
        float physics_ready_at = -1;

        public bool PhysicsReady { get; private set; } = true;
        public float PhysicsReadyCountdown { get; private set; }

        public void AbortWarp(bool instant = false)
        {
            VSL.Controls.AbortWarp(instant);
            require_settle_after_warp = true;
            physics_ready_at = -1;
            Reset();
        }

        public override void Disable()
        {
            AbortWarp();
        }

        protected override void Reset()
        {
            base.Reset();
            last_warp_index = TimeWarp.CurrentRateIndex;
            VSL.Controls.NoDewarpOffset = false;
            frames_to_skip = -1;
        }

        public override void Init()
        {
            base.Init();
            frames_to_skip = -1;
            last_warp_index = TimeWarp.CurrentRateIndex;
            last_asked_index = TimeWarp.CurrentRateIndex;
            require_settle_after_warp = false;
            physics_ready_at = -1;
            warp_increase_attempt_time = 0;
            PhysicsReady = true;
            PhysicsReadyCountdown = 0;
            GameEvents.onVesselSOIChanged.Add(OnSOIChanged);
        }

        void OnSOIChanged(GameEvents.HostedFromToAction<Vessel, CelestialBody> action)
        {
            if(TCA == null || action.host != VSL.vessel) return;
            if(TimeWarp.CurrentRate > 1)
                Message("Disengaging Time Warp on SOI change...");
            AbortWarp();
        }

        bool rate_settled(float target_rate)
        {
            var tolerance = Mathf.Max(0.01f, Mathf.Abs(target_rate)*C.WarpRateTolerance);
            return Mathf.Abs(TimeWarp.CurrentRate-target_rate) <= tolerance;
        }

        bool actively_warping =>
            TimeWarp.CurrentRateIndex > 0
            || !rate_settled(1f)
            || VSL.Controls.WarpToTime >= 0;

        public void UpdatePhysicsReadyState()
        {
            if(actively_warping)
            {
                require_settle_after_warp = true;
                physics_ready_at = -1;
                PhysicsReady = false;
                PhysicsReadyCountdown = 0;
                VSL.Controls.SetPhysicsReady(false, 0);
                return;
            }
            if(!require_settle_after_warp)
            {
                PhysicsReady = true;
                PhysicsReadyCountdown = 0;
                VSL.Controls.SetPhysicsReady(true, 0);
                return;
            }
            if(physics_ready_at < 0)
                physics_ready_at = Time.realtimeSinceStartup;
            var settle_time = Mathf.Max(C.PostWarpSettleTime, 0);
            PhysicsReadyCountdown = Mathf.Max(0, settle_time-(Time.realtimeSinceStartup-physics_ready_at));
            PhysicsReady = PhysicsReadyCountdown <= 0;
            if(PhysicsReady)
                require_settle_after_warp = false;
            VSL.Controls.SetPhysicsReady(PhysicsReady, PhysicsReadyCountdown);
        }

        public static void UpdatePhysicsReadyFallback(ControlProps ctrl)
        {
            var tolerance = Mathf.Max(0.01f, C.WarpRateTolerance);
            var at_1x = TimeWarp.CurrentRateIndex == 0 && Mathf.Abs(TimeWarp.CurrentRate-1f) <= tolerance;
            ctrl.SetPhysicsReady(at_1x, 0);
        }

        double effective_warp_target(double target_ut)
        {
            if(VSL.Controls.NoDewarpOffset)
                return target_ut;
            return target_ut-C.DewarpTime/(VSL.LandedOrSplashed? 2 : 1);
        }

        void set_warp_rate(int rate_index, bool instant)
        {
            if(rate_index == TimeWarp.CurrentRateIndex)
                return;
            last_asked_index = rate_index;
            TimeWarp.SetRate(rate_index, instant);
            frames_to_skip = -1;
        }

        bool CheckRegularWarp()
        {
            if(TimeWarp.WarpMode != TimeWarp.Modes.HIGH)
            {
                var instant_altitude_asl = VSL.orbit.radius-VSL.Body.Radius;
                if(!VSL.Body.atmosphere || instant_altitude_asl > VSL.Body.atmosphereDepth)
                {
                    TimeWarp.fetch.Mode = TimeWarp.Modes.HIGH;
                    set_warp_rate(0, true);
                }
                return false;
            }
            return true;
        }

        bool CheckPhysicsWarp()
        {
            if(TimeWarp.WarpMode != TimeWarp.Modes.LOW)
            {
                TimeWarp.fetch.Mode = TimeWarp.Modes.LOW;
                set_warp_rate(0, true);
                return false;
            }
            return true;
        }

        bool IncreaseRegularWarp(bool instant = false)
        {
            if(!CheckRegularWarp()) return false;
            if(TimeWarp.CurrentRateIndex+1 == TimeWarp.fetch.warpRates.Length) return false;
            if(!VSL.LandedOrSplashed)
            {
                var altitude = VSL.orbit.radius-VSL.Body.Radius;
                if(TimeWarp.fetch.GetAltitudeLimit(TimeWarp.CurrentRateIndex+1, VSL.Body) > altitude)
                    return false;
            }
            if(TimeWarp.fetch.warpRates[TimeWarp.CurrentRateIndex] != TimeWarp.CurrentRate)
                return false;
            if(VSL.Physics.UT-warp_increase_attempt_time < C.WarpIncreaseDelay)
                return false;
            warp_increase_attempt_time = VSL.Physics.UT;
            set_warp_rate(TimeWarp.CurrentRateIndex+1, instant);
            return true;
        }

        bool DecreaseRegularWarp(bool instant = false)
        {
            if(!CheckRegularWarp()) return false;
            if(TimeWarp.CurrentRateIndex == 0) return false;
            set_warp_rate(TimeWarp.CurrentRateIndex-1, instant);
            return true;
        }

        bool IncreasePhysicsWarp(bool instant = false)
        {
            if(!CheckPhysicsWarp()) return false;
            if(TimeWarp.CurrentRateIndex+1 == TimeWarp.fetch.physicsWarpRates.Length) return false;
            if(TimeWarp.fetch.physicsWarpRates[TimeWarp.CurrentRateIndex] != TimeWarp.CurrentRate)
                return false;
            if(VSL.Physics.UT-warp_increase_attempt_time < C.WarpIncreaseDelay)
                return false;
            warp_increase_attempt_time = VSL.Physics.UT;
            set_warp_rate(TimeWarp.CurrentRateIndex+1, instant);
            return true;
        }

        bool DecreasePhysicsWarp(bool instant = false)
        {
            if(!CheckPhysicsWarp()) return false;
            if(TimeWarp.CurrentRateIndex == 0) return false;
            set_warp_rate(TimeWarp.CurrentRateIndex-1, instant);
            return true;
        }

        void WarpRegularAtRate(float max_rate, bool instant_on_increase = false, bool instant_on_decrease = true)
        {
            if(!CheckRegularWarp()) return;
            if(TimeWarp.fetch.warpRates[TimeWarp.CurrentRateIndex] > max_rate)
                DecreaseRegularWarp(instant_on_decrease);
            else if(TimeWarp.CurrentRateIndex+1 < TimeWarp.fetch.warpRates.Length &&
                    TimeWarp.fetch.warpRates[TimeWarp.CurrentRateIndex+1] <= max_rate)
                IncreaseRegularWarp(instant_on_increase);
        }

        void WarpPhysicsAtRate(float max_rate, bool instant_on_increase = false, bool instant_on_decrease = true)
        {
            if(!CheckPhysicsWarp()) return;
            if(TimeWarp.fetch.physicsWarpRates[TimeWarp.CurrentRateIndex] > max_rate)
                DecreasePhysicsWarp(instant_on_decrease);
            else if(TimeWarp.CurrentRateIndex+1 < TimeWarp.fetch.physicsWarpRates.Length &&
                    TimeWarp.fetch.physicsWarpRates[TimeWarp.CurrentRateIndex+1] <= max_rate)
                IncreasePhysicsWarp(instant_on_increase);
        }

        void WarpToUT(double ut, double max_rate = -1)
        {
            var target_ut = effective_warp_target(ut);
            if(target_ut <= VSL.Physics.UT)
                return;

            if(max_rate < 0)
                max_rate = Math.Min(C.MaxWarp, TimeWarp.fetch.warpRates[TimeWarp.fetch.warpRates.Length-1]);

            double desired_rate;
            if(C.UseQuickWarp)
            {
                desired_rate = 1;
                if(VSL.orbit.patchEndTransition != Orbit.PatchTransitionType.FINAL &&
                   VSL.orbit.EndUT < target_ut)
                {
                    for(var i = 0; i < TimeWarp.fetch.warpRates.Length; i++)
                    {
                        if(i*Time.fixedDeltaTime*TimeWarp.fetch.warpRates[i] <= VSL.orbit.EndUT-VSL.Physics.UT)
                            desired_rate = TimeWarp.fetch.warpRates[i]+0.1;
                        else break;
                    }
                }
                else
                {
                    for(var i = 0; i < TimeWarp.fetch.warpRates.Length; i++)
                    {
                        if(i*Time.fixedDeltaTime*TimeWarp.fetch.warpRates[i] <= target_ut-VSL.Physics.UT)
                            desired_rate = TimeWarp.fetch.warpRates[i]+0.1;
                        else break;
                    }
                }
            }
            else
                desired_rate = target_ut-(VSL.Physics.UT+Time.fixedDeltaTime*TimeWarp.CurrentRateIndex);

            desired_rate = Utils.Clamp(desired_rate, 1, max_rate);

            if(!VSL.LandedOrSplashed &&
               VSL.orbit.radius-VSL.Body.Radius < TimeWarp.fetch.GetAltitudeLimit(1, VSL.Body))
                WarpPhysicsAtRate((float)Math.Min(desired_rate, 2), C.UseQuickWarp, true);
            else
                WarpRegularAtRate((float)desired_rate, C.UseQuickWarp, true);
        }

        bool altitude_rate_limited =>
            !VSL.LandedOrSplashed &&
            TimeWarp.WarpMode == TimeWarp.Modes.HIGH &&
            TimeWarp.CurrentRateIndex == TimeWarp.fetch.GetMaxRateForAltitude(VSL.orbit.radius-VSL.Body.Radius, VSL.Body);

        public override void ProcessKeys()
        {
            if(GameSettings.TIME_WARP_STOP.GetKey())
            {
                if(CFG.WarpToNode && VSL.Controls.WarpToTime > 0)
                    AbortWarp();
            }
        }

        protected override void Update()
        {
            var warp_to_time = VSL.Controls.WarpToTime;
            if(warp_to_time < 0)
            {
                Reset();
                return;
            }

            // try to catch the moment KSP or some other mod sets warp besides us
            if(TimeWarp.CurrentRateIndex < last_warp_index && !altitude_rate_limited &&
               last_asked_index > 0 && last_asked_index != TimeWarp.CurrentRateIndex)
            {
                if(frames_to_skip < 0)
                    frames_to_skip = TimeWarp.CurrentRateIndex*C.FramesToSkip;
                if(frames_to_skip-- > 0) return;
                Message("TCA Time Warp was overridden.");
                VSL.Controls.WarpToTime = -1;
                CFG.WarpToNode = false;
                Reset();
                return;
            }

            // dewarp if the warp was disabled, or LOW mode
            if(warp_to_time > 0 &&
               (!CFG.WarpToNode || TimeWarp.WarpMode == TimeWarp.Modes.LOW))
            {
                VSL.Controls.WarpToTime = 0;
                warp_to_time = 0;
            }

            if(warp_to_time <= VSL.Physics.UT && rate_settled(1f))
            {
                VSL.Controls.WarpToTime = -1;
                Reset();
                return;
            }

            if(warp_to_time > 0)
                WarpToUT(warp_to_time);
            else if(TimeWarp.CurrentRateIndex > 0)
                DecreaseRegularWarp(true);
            else if(!rate_settled(1f))
                set_warp_rate(0, true);

            Reset();
        }
    }
}
