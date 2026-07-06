//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    [CareerPart(typeof(ManeuverAutopilot))]
    public class TimeWarpControl : TCAModule
    {
        public class Config : ComponentConfig<Config>
        {
            [Persistent] public float DewarpTime   = 20f;  //sec
            [Persistent] public float MaxWarp      = 10000f;
            [Persistent] public int   FramesToSkip = 3;
            [Persistent] public float WarpStepRealTime   = 1f;    //sec
            [Persistent] public float WarpRateTolerance   = 0.02f; //relative
            [Persistent] public float DewarpSafetyMargin  = 2f;    //sec
        }
        public static Config C => Config.INST;

        public TimeWarpControl(ModuleTCA tca) : base(tca) {}

        int last_warp_index;
        int frames_to_skip;
        bool waiting_for_warp_step;
        int commanded_warp_index = -1;
        float commanded_warp_rate = 1;
        float warp_step_started_at;

        public void AbortWarp(bool instant = false)
        {
            VSL.Controls.AbortWarp(instant);
            reset_warp_step();
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
            reset_warp_step();
            GameEvents.onVesselSOIChanged.Add(OnSOIChanged);
        }

        void OnSOIChanged(GameEvents.HostedFromToAction<Vessel, CelestialBody> action)
        {
            if(TCA == null || action.host != VSL.vessel) return;
            if(TimeWarp.CurrentRate > 1)
                Message("Disengaging Time Warp on SOI change...");
            AbortWarp();
        }

        void reset_warp_step()
        {
            waiting_for_warp_step = false;
            commanded_warp_index = -1;
            commanded_warp_rate = 1;
            warp_step_started_at = 0;
        }

        bool rate_settled(float target_rate)
        {
            var tolerance = Mathf.Max(0.01f, Mathf.Abs(target_rate)*C.WarpRateTolerance);
            return Mathf.Abs(TimeWarp.CurrentRate-target_rate) <= tolerance;
        }

        bool warp_step_settled
        {
            get
            {
                if(!waiting_for_warp_step)
                    return true;
                if(TimeWarp.fetch == null ||
                   commanded_warp_index < 0 ||
                   commanded_warp_index >= TimeWarp.fetch.warpRates.Length)
                {
                    reset_warp_step();
                    return true;
                }
                if(Time.realtimeSinceStartup-warp_step_started_at < Mathf.Max(C.WarpStepRealTime, 0))
                    return false;
                if(TimeWarp.CurrentRateIndex != commanded_warp_index)
                    return false;
                if(!rate_settled(commanded_warp_rate))
                    return false;
                reset_warp_step();
                return true;
            }
        }

        bool expected_warp_decrease =>
            waiting_for_warp_step &&
            commanded_warp_index >= 0 &&
            TimeWarp.CurrentRateIndex == commanded_warp_index;

        void set_warp_rate(int rate_index)
        {
            var current_index = TimeWarp.CurrentRateIndex;
            if(rate_index == current_index)
                return;
            if(rate_index > current_index)
                TimeWarp.SetRate(rate_index, false, false);
            else
                TimeWarp.SetRate(rate_index, false);
            commanded_warp_index = rate_index;
            commanded_warp_rate = TimeWarp.fetch.warpRates[rate_index];
            warp_step_started_at = Time.realtimeSinceStartup;
            waiting_for_warp_step = true;
            frames_to_skip = -1;
        }

        double TimeNeededToDewarpFrom(int rate_index)
        {
            var safety_margin = Mathf.Max(C.DewarpSafetyMargin, 0);
            if(TimeWarp.fetch == null)
                return safety_margin;
            var rates = TimeWarp.fetch.warpRates;
            if(rates == null || rates.Length == 0)
                return safety_margin;
            var index = Mathf.Clamp(rate_index, 0, rates.Length-1);
            var step_time = Mathf.Max(C.WarpStepRealTime, 0);
            double time_needed = safety_margin;
            for(var i = index; i > 0; i--)
                time_needed += (rates[i]+rates[i-1])*0.5*step_time;
            return time_needed;
        }

        // TimeWarp changes timescale from one rate to the next in real time.
        // Budget every step so TCA does not climb into a rate it cannot dewarp from in time.
        double TimeToDewarp(int rate_index)
        { 
            var offset = VSL.Controls.NoDewarpOffset? 0 : C.DewarpTime/(VSL.LandedOrSplashed? 2 : 1);
            return VSL.Controls.WarpToTime-(offset+TimeNeededToDewarpFrom(rate_index))-VSL.Physics.UT;
        }

        public override void ProcessKeys()
        {
            if(GameSettings.TIME_WARP_STOP.GetKey())
            {
                if(CFG.WarpToNode && VSL.Controls.WarpToTime > 0)
                    AbortWarp();
            }
        }

        bool can_increase_rate
        { 
            get 
            { 
                return TimeWarp.CurrentRateIndex < 
                    TimeWarp.fetch.GetMaxRateForAltitude(VSL.orbit.radius-VSL.Body.Radius, VSL.Body); 
            } 
        }

        protected override void Update()
        {
            if(VSL.Controls.WarpToTime < 0)
            {
                reset_warp_step();
                goto end;
            }
            //try to catch the moment KSP or some other mod sets warp besides us
            if(TimeWarp.CurrentRateIndex < last_warp_index && can_increase_rate && !expected_warp_decrease)
            { 
                if(frames_to_skip < 0)
                    frames_to_skip = TimeWarp.CurrentRateIndex * C.FramesToSkip;
//                Log("current index {}, max index at alt {}, frames_to_skip {}",
//                    TimeWarp.CurrentRateIndex, 
//                    TimeWarp.fetch.GetMaxRateForAltitude(VSL.vessel.altitude, VSL.Body),
//                    frames_to_skip);
                if(frames_to_skip-- > 0) return;
                Message("TCA Time Warp was overridden.");
                VSL.Controls.WarpToTime = -1;
                CFG.WarpToNode = false; 
                reset_warp_step();
                goto end; 
            }
            //dewarp if the warp was disabled, or LOW mode
            if(VSL.Controls.WarpToTime > 0 && 
               (!CFG.WarpToNode || 
                TimeWarp.WarpMode == TimeWarp.Modes.LOW)) 
                VSL.Controls.WarpToTime = 0;
            if(VSL.Controls.WarpToTime <= VSL.Physics.UT && TimeWarp.CurrentRate.Equals(1))
            { 
                VSL.Controls.WarpToTime = -1;
                reset_warp_step();
                goto end;
            }
            if(!warp_step_settled)
                goto end;
            if(TimeToDewarp(TimeWarp.CurrentRateIndex) < 0)
            {
                if(TimeWarp.CurrentRateIndex > 0)
                    set_warp_rate(TimeWarp.CurrentRateIndex-1);
                else if(TimeWarp.CurrentRate.Equals(1))
                    VSL.Controls.WarpToTime = 0;
            }
            else if(TimeWarp.CurrentRateIndex < TimeWarp.fetch.warpRates.Length-1 && 
                    TimeWarp.fetch.warpRates[TimeWarp.CurrentRateIndex+1] <= C.MaxWarp &&
                    (VSL.LandedOrSplashed || can_increase_rate) &&
                    TimeToDewarp(TimeWarp.CurrentRateIndex+1) > 0)
                set_warp_rate(TimeWarp.CurrentRateIndex+1);
            end: Reset();
        }
    }
}

