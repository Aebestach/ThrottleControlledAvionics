//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License.

using UnityEngine;

namespace ThrottleControlledAvionics
{
    public static class RangePlannerValidation
    {
        static MissionPerformanceSnapshot synthetic_snapshot(float thrust, float massFlow, float mass, float fuel)
        {
            return new MissionPerformanceSnapshot
            {
                Valid = true,
                VesselName = "RangePlannerValidation",
                WetMass = mass,
                DryMass = Mathf.Max(mass - fuel, 0.01f),
                Mass = mass,
                FuelMass = fuel,
                Thrust = thrust,
                MassFlow = massFlow,
                EffectiveExhaustVelocity = thrust / massFlow,
                Area = 1,
                AreaWithBrakes = 1,
                MinArea = 1
            };
        }

        public static bool VacuumSanity(CelestialBody body)
        {
            if(body == null)
                body = FlightGlobals.GetHomeBody();
            var snapshot = synthetic_snapshot(200, 0.5f, 10, 4);
            var result = MissionRangePlanner.Evaluate(snapshot, new MissionProfile
            {
                Body = body,
                StartAltitude = body.atmosphere ? (float)body.atmosphereDepth + 1000 : 0,
                TargetDistance = 1000,
                MaxCruiseSpeed = 50,
                HoverReserveTime = 5,
                Scenario = MissionScenario.TargetRange
            });
            return result.Valid
                   && result.DeltaV > 0
                   && result.HoverTime > 0
                   && result.BestHopDistance > 0;
        }

        public static bool LowTwrSanity(CelestialBody body)
        {
            if(body == null)
                body = FlightGlobals.GetHomeBody();
            var snapshot = synthetic_snapshot(1, 0.5f, 10, 4);
            var result = MissionRangePlanner.Evaluate(snapshot, new MissionProfile
            {
                Body = body,
                StartAltitude = 0,
                TargetDistance = 1000,
                MaxCruiseSpeed = 50,
                HoverReserveTime = 5,
                Scenario = MissionScenario.TargetRange
            });
            return result.Valid
                   && result.HoverTime <= 0
                   && result.PoweredCruiseRange <= 0
                   && result.HasWarnings;
        }
    }
}
