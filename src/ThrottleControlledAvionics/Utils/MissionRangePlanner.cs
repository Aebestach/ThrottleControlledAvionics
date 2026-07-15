//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License.
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    public enum MissionScenario
    {
        Hover,
        PoweredCruise,
        BallisticHop,
        TargetRange,
        RepeatedHops
    }

    public enum MissionRecommendation
    {
        None,
        GoTo,
        BallisticJump
    }

    public class BodyEnvironment
    {
        public CelestialBody Body { get; private set; }
        public float Altitude { get; private set; }
        public bool HasAtmosphere { get; private set; }
        public bool InAtmosphere { get; private set; }
        public float Gravity { get; private set; }
        public float Radius { get; private set; }
        public float Density { get; private set; }
        public float Pressure { get; private set; }
        public float Mach1 { get; private set; }

        public static BodyEnvironment FromBody(CelestialBody body, float altitude)
        {
            if(body == null)
                return null;
            altitude = Mathf.Max(altitude, 0);
            var radius = body.Radius + altitude;
            var env = new BodyEnvironment
            {
                Body = body,
                Altitude = altitude,
                HasAtmosphere = body.atmosphere,
                InAtmosphere = body.atmosphere && altitude < body.atmosphereDepth,
                Gravity = (float)(body.gMagnitudeAtCenter / radius / radius),
                Radius = (float)body.Radius
            };
            if(env.InAtmosphere)
            {
                var atm = body.AtmoParamsAtAltitude(altitude);
                env.Density = (float)atm.Rho;
                env.Pressure = (float)atm.P;
                env.Mach1 = (float)atm.Mach1;
            }
            return env;
        }
    }

    public class MissionProfile
    {
        public CelestialBody Body;
        public MissionScenario Scenario = MissionScenario.TargetRange;
        public float StartAltitude;
        public float TargetDistance;
        public float MaxCruiseSpeed = 100;
        public float HoverReserveTime = LandingTrajectoryAutopilot.C.HoverTimeThreshold;
        public bool AllowParachutes = true;
        public bool AllowStaging = true;
        public bool AllowAtmosphericAssist = true;
    }

    public class MissionStageSegment
    {
        public int Stage;
        public float StartMass;
        public float FuelMass;
        public float Thrust;
        public float MassFlow;
        public float DeltaV;
        public float BurnTime;
    }

    public class MissionPredictionResult
    {
        public bool Valid;
        public string Status = "";
        public float Confidence = 1;
        public MissionRecommendation Recommendation = MissionRecommendation.None;
        public BodyEnvironment Environment;
        public float AvailableFuel;
        public float ReserveFuel;
        public float DeltaV;
        public float HoverTime;
        public float PoweredCruiseRange;
        public float PoweredCruiseFuel;
        public float BestHopDistance;
        public float BestHopFuel;
        public float TargetJumpFuel;
        public float TargetGoToFuel;
        public int HopCount;
        public float TotalHopRange;
        public float MaxDynamicPressure;
        public readonly List<string> Warnings = new List<string>();

        public bool HasWarnings => Warnings.Count > 0;

        public void Warn(string warning, float confidencePenalty = 0.1f)
        {
            if(!string.IsNullOrEmpty(warning))
                Warnings.Add(warning);
            Confidence = Mathf.Clamp01(Confidence - confidencePenalty);
        }
    }

    public class MissionPerformanceSnapshot
    {
        public class Engine
        {
            public ModuleEngines Module;
            public string Name;
            public int Stage;
            public float Limit = 1;
            public float NominalThrust;
            public float NominalMassFlow;
            public bool AirBreathing;

            public float ThrustAt(CelestialBody body, float speed, float altitude, out float massFlow)
            {
                massFlow = 0;
                if(Module == null)
                    return 0;
                var limit = Mathf.Clamp01(Limit);
                massFlow = Module.maxFuelFlow * limit;
                var pressureAtm = 0f;
                var relDensity = 0f;
                var mach = 0f;
                if(body != null && body.atmosphere && altitude < body.atmosphereDepth)
                {
                    var atm = body.AtmoParamsAtAltitude(altitude);
                    pressureAtm = (float)(atm.P / 1013.25);
                    relDensity = (float)(atm.Rho / 1.225);
                    mach = atm.Mach1 > 0 ? (float)(speed / atm.Mach1) : 0;
                }
                var isp = Module.atmosphereCurve.Evaluate(pressureAtm) * Module.multIsp;
                if(Module.useAtmCurveIsp)
                    isp *= Module.atmCurveIsp.Evaluate(relDensity);
                if(Module.useVelCurveIsp)
                    isp *= Module.velCurveIsp.Evaluate(mach);
                var flowMod = Module.multFlow;
                if(Module.atmChangeFlow)
                {
                    flowMod = relDensity;
                    if(Module.useAtmCurve)
                        flowMod = Module.atmCurve.Evaluate(flowMod);
                }
                if(Module.useVelCurve)
                    flowMod *= Module.velCurve.Evaluate(mach);
                if(flowMod > Module.flowMultCap)
                {
                    var toCap = flowMod - Module.flowMultCap;
                    flowMod = Module.flowMultCap + toCap / (Module.flowMultCapSharpness + toCap / Module.flowMultCap);
                }
                if(flowMod < Module.CLAMP && Module.CLAMP < 1)
                    flowMod = Module.CLAMP;
                massFlow *= flowMod;
                return massFlow * isp * Utils.G0;
            }

            public static Engine FromWrapper(EngineWrapper e)
            {
                if(e == null || e.engine == null)
                    return null;
                return new Engine
                {
                    Module = e.engine,
                    Name = e.name,
                    Stage = e.part != null ? e.part.inverseStage : -1,
                    Limit = Mathf.Clamp01(e.limit),
                    NominalThrust = e.nominalCurrentThrust(e.limit),
                    NominalMassFlow = e.engine.maxFuelFlow * Mathf.Clamp01(e.limit),
                    AirBreathing = e.engine.propellants.Any(p => p.name == "IntakeAir")
                };
            }
        }

        public bool Valid;
        public bool EditorSnapshot;
        public string VesselName = "";
        public float WetMass;
        public float DryMass;
        public float Mass;
        public float FuelMass;
        public float Thrust;
        public float MassFlow;
        public float EffectiveExhaustVelocity;
        public float Area = 1;
        public float AreaWithBrakes = 1;
        public float MinArea = 1;
        public bool HaveParachutes;
        public bool HaveStagedResources;
        public bool HaveAirBreathingEngines;
        public readonly List<Engine> Engines = new List<Engine>();
        public readonly List<MissionStageSegment> StageSegments = new List<MissionStageSegment>();
        public readonly List<string> Warnings = new List<string>();

        public float DryishMass => Mathf.Max(Mass - FuelMass, DryMass, 0.01f);

        public void ThrustAt(CelestialBody body, float speed, float altitude, out float thrust, out float massFlow)
        {
            thrust = 0;
            massFlow = 0;
            for(int i = 0; i < Engines.Count; i++)
            {
                float flow;
                thrust += Engines[i].ThrustAt(body, speed, altitude, out flow);
                massFlow += flow;
            }
            if(Engines.Count == 0)
            {
                thrust = Thrust;
                massFlow = MassFlow;
            }
        }

        public float FuelNeeded(float dV)
        {
            if(dV <= 0)
                return 0;
            if(EffectiveExhaustVelocity <= 0)
                return float.PositiveInfinity;
            return Mass * (1 - Mathf.Exp(-dV / EffectiveExhaustVelocity));
        }

        public float DeltaV(float fuelMass)
        {
            fuelMass = Mathf.Clamp(fuelMass, 0, Mass - 0.01f);
            if(fuelMass <= 0 || EffectiveExhaustVelocity <= 0)
                return 0;
            return EffectiveExhaustVelocity * Mathf.Log(Mass / (Mass - fuelMass));
        }

        public static MissionPerformanceSnapshot FromVessel(VesselWrapper vsl)
        {
            var s = new MissionPerformanceSnapshot();
            if(vsl == null || vsl.vessel == null || vsl.Engines == null || vsl.Engines.NoActiveEngines)
            {
                s.Valid = false;
                s.Warnings.Add(Loc.T("RangePlanner_NoActiveEngines", "No active engines."));
                return s;
            }
            s.Valid = true;
            s.VesselName = vsl.vessel.vesselName;
            s.WetMass = vsl.Physics.M;
            s.Mass = vsl.Physics.M;
            s.FuelMass = vsl.Engines.AvailableFuelMass;
            s.DryMass = Mathf.Max(s.Mass - s.FuelMass, 0.01f);
            s.Thrust = vsl.Engines.MaxThrustM;
            s.MassFlow = vsl.Engines.MaxMassFlow;
            s.EffectiveExhaustVelocity = vsl.Engines.MaxVe;
            s.Area = Mathf.Max(vsl.Geometry.Area, 1);
            s.AreaWithBrakes = Mathf.Max(vsl.Geometry.AreaWithBrakes, vsl.Geometry.MinArea, 1);
            s.MinArea = Mathf.Max(vsl.Geometry.MinArea, 1);
            s.HaveParachutes = vsl.OnPlanetParams.HaveParachutes;
            for(int i = 0; i < vsl.Engines.Active.Count; i++)
            {
                var e = vsl.Engines.Active[i];
                if(!e.isThruster)
                    continue;
                var engine = Engine.FromWrapper(e);
                if(engine != null)
                {
                    s.Engines.Add(engine);
                    s.HaveAirBreathingEngines |= engine.AirBreathing;
                }
            }
            s.BuildStageSegments(null);
            return s;
        }

        public static MissionPerformanceSnapshot FromEditor(ShipConstruct ship, EnginesDB activeEngines,
                                                            float wetMass, float dryMass, float area)
        {
            var s = new MissionPerformanceSnapshot { EditorSnapshot = true };
            if(ship == null || activeEngines == null || activeEngines.Count == 0)
            {
                s.Valid = false;
                s.Warnings.Add(Loc.T("RangePlanner_NoActiveEngines", "No active engines."));
                return s;
            }
            s.Valid = true;
            s.VesselName = ship.shipName;
            s.WetMass = Mathf.Max(wetMass, 0.01f);
            s.DryMass = Mathf.Max(dryMass, 0.01f);
            s.Mass = s.WetMass;
            s.Area = Mathf.Max(area, 1);
            s.AreaWithBrakes = s.Area;
            s.MinArea = Mathf.Max(area / 6, 1);
            s.HaveParachutes = ship.Parts.Any(p => p.Modules.Contains<ModuleParachute>());
            for(int i = 0; i < activeEngines.Count; i++)
            {
                var e = activeEngines[i];
                if(!e.isThruster)
                    continue;
                var engine = Engine.FromWrapper(e);
                if(engine == null)
                    continue;
                s.Engines.Add(engine);
                s.Thrust += engine.NominalThrust;
                s.MassFlow += engine.NominalMassFlow;
                s.HaveAirBreathingEngines |= engine.AirBreathing;
            }
            s.FuelMass = collect_editor_fuel_mass(ship, s.Engines);
            if(s.FuelMass <= 0)
                s.FuelMass = Mathf.Max(s.WetMass - s.DryMass, 0);
            s.EffectiveExhaustVelocity = s.MassFlow > 0 ? s.Thrust / s.MassFlow : 0;
            s.BuildStageSegments(ship);
            return s;
        }

        static float collect_editor_fuel_mass(ShipConstruct ship, List<Engine> engines)
        {
            if(ship == null || engines == null || engines.Count == 0)
                return 0;
            var fuels = new Dictionary<int, PartResourceDefinition>();
            for(int i = 0; i < engines.Count; i++)
            {
                var engine = engines[i].Module;
                if(engine == null)
                    continue;
                engine.GetConsumedResources().ForEach(r =>
                {
                    if(!fuels.ContainsKey(r.id))
                        fuels.Add(r.id, r);
                });
            }
            var fuelMass = 0.0;
            for(int i = 0; i < ship.Parts.Count; i++)
            {
                var resources = ship.Parts[i].Resources;
                for(int j = 0; j < resources.Count; j++)
                {
                    var r = resources[j];
                    if(r == null || r.info == null || !fuels.ContainsKey(r.info.id))
                        continue;
                    fuelMass += r.amount * r.info.density;
                }
            }
            return (float)fuelMass;
        }

        void BuildStageSegments(ShipConstruct ship)
        {
            StageSegments.Clear();
            if(Engines.Count == 0)
                return;
            var stages = Engines.Select(e => e.Stage).Distinct().OrderByDescending(s => s).ToList();
            var fuelPerStage = FuelMass / Mathf.Max(stages.Count, 1);
            for(int i = 0; i < stages.Count; i++)
            {
                var stage = stages[i];
                var engines = Engines.Where(e => e.Stage == stage).ToList();
                var thrust = engines.Sum(e => e.NominalThrust);
                var flow = engines.Sum(e => e.NominalMassFlow);
                var fuel = fuelPerStage;
                if(ship != null)
                    fuel = Mathf.Max(fuel, resource_mass_for_stage(ship, engines));
                var ve = flow > 0 ? thrust / flow : 0;
                var startMass = Mathf.Max(Mass - StageSegments.Sum(s => s.FuelMass), DryishMass);
                StageSegments.Add(new MissionStageSegment
                {
                    Stage = stage,
                    StartMass = startMass,
                    FuelMass = fuel,
                    Thrust = thrust,
                    MassFlow = flow,
                    DeltaV = ve > 0 && fuel > 0 && fuel < startMass ? ve * Mathf.Log(startMass / (startMass - fuel)) : 0,
                    BurnTime = flow > 0 ? fuel / flow : 0
                });
            }
            HaveStagedResources = StageSegments.Count > 1;
            if(ship != null && StageSegments.Count > 1)
                Warnings.Add(Loc.T("RangePlanner_StageApproximation", "Stage fuel is estimated conservatively; crossfeed and decouplers may change the result."));
        }

        static float resource_mass_for_stage(ShipConstruct ship, List<Engine> engines)
        {
            if(ship == null || engines == null || engines.Count == 0)
                return 0;
            var fuels = new Dictionary<int, PartResourceDefinition>();
            engines.ForEach(e =>
            {
                if(e.Module == null)
                    return;
                e.Module.GetConsumedResources().ForEach(r =>
                {
                    if(!fuels.ContainsKey(r.id))
                        fuels.Add(r.id, r);
                });
            });
            var minStage = engines.Min(e => e.Stage);
            var mass = 0.0;
            for(int i = 0; i < ship.Parts.Count; i++)
            {
                var p = ship.Parts[i];
                if(p.inverseStage < minStage)
                    continue;
                for(int j = 0; j < p.Resources.Count; j++)
                {
                    var r = p.Resources[j];
                    if(r == null || r.info == null || !fuels.ContainsKey(r.info.id))
                        continue;
                    mass += r.amount * r.info.density;
                }
            }
            return (float)mass;
        }
    }

    public static class MissionRangePlanner
    {
        const float MinCruiseSpeed = 1;
        const float BallisticLossFactorVacuum = 2.15f;
        const float BallisticLossFactorAtmosphere = 2.65f;

        public static MissionPredictionResult Evaluate(MissionPerformanceSnapshot snapshot, MissionProfile profile)
        {
            var result = new MissionPredictionResult();
            if(snapshot == null || !snapshot.Valid)
            {
                result.Valid = false;
                result.Status = Loc.T("RangePlanner_InvalidSnapshot", "No usable vessel performance data.");
                return result;
            }
            if(profile == null || profile.Body == null)
            {
                result.Valid = false;
                result.Status = Loc.T("RangePlanner_NoBody", "No body selected.");
                return result;
            }
            result.Valid = true;
            result.Environment = BodyEnvironment.FromBody(profile.Body, profile.StartAltitude);
            snapshot.ThrustAt(profile.Body, 0, profile.StartAltitude, out var thrust0, out var flow0);
            if(thrust0 <= 0 || flow0 <= 0)
            {
                result.Valid = false;
                result.Status = Loc.T("RangePlanner_NoThrust", "No usable thrust in the selected environment.");
                return result;
            }
            result.DeltaV = snapshot.DeltaV(snapshot.FuelMass);
            if(profile.AllowStaging && snapshot.StageSegments.Count > 1)
                result.DeltaV = Mathf.Max(result.DeltaV, snapshot.StageSegments.Sum(s => s.DeltaV));
            result.HoverTime = hover_time(snapshot.Mass, snapshot.FuelMass, thrust0, flow0, result.Environment.Gravity);
            result.ReserveFuel = fuel_for_hover(snapshot.Mass, thrust0, flow0, result.Environment.Gravity, profile.HoverReserveTime);
            if(profile.AllowParachutes && snapshot.HaveParachutes && result.Environment.InAtmosphere)
                result.ReserveFuel *= 0.5f;
            result.AvailableFuel = Mathf.Clamp(snapshot.FuelMass - result.ReserveFuel, 0, snapshot.FuelMass);
            result.PoweredCruiseRange = powered_cruise_range(snapshot, profile, result, result.AvailableFuel, out var cruiseFuel);
            result.PoweredCruiseFuel = cruiseFuel;
            find_best_hop(snapshot, profile, result, result.AvailableFuel);
            if(profile.TargetDistance > 0)
                compare_target_modes(snapshot, profile, result, result.AvailableFuel);
            result.TotalHopRange = result.BestHopDistance * result.HopCount;
            snapshot.Warnings.ForEach(w => result.Warn(w, 0.05f));
            if(snapshot.HaveAirBreathingEngines)
                result.Warn(Loc.T("RangePlanner_AirBreathingWarning", "Air-breathing engine range depends on intake air and speed; estimate is conservative."), 0.15f);
            if(result.Environment.InAtmosphere)
            {
                result.Warn(Loc.T("RangePlanner_AtmosphereWarning", "Atmospheric drag, lift and attitude are approximated."), 0.15f);
                if(!profile.AllowAtmosphericAssist)
                    result.Warn(Loc.T("RangePlanner_AtmosphereAssistOff", "Atmospheric assist is disabled; powered reserves are used conservatively."), 0.05f);
            }
            result.Status = result.Valid
                ? Loc.T("RangePlanner_Ready", "Prediction ready.")
                : Loc.T("RangePlanner_InvalidSnapshot", "No usable vessel performance data.");
            return result;
        }

        static float hover_time(float mass, float fuel, float thrust, float flow, float g)
        {
            if(fuel <= 0 || thrust <= mass * g || flow <= 0)
                return 0;
            var averageMass = Mathf.Max(mass - fuel / 2, 0.01f);
            var throttle = Mathf.Clamp01(averageMass * g / thrust);
            return throttle > 0 ? fuel / flow / throttle : 0;
        }

        static float fuel_for_hover(float mass, float thrust, float flow, float g, float seconds)
        {
            if(seconds <= 0 || thrust <= 0 || flow <= 0)
                return 0;
            var throttle = Mathf.Clamp01(mass * g / thrust);
            return flow * throttle * seconds;
        }

        static float powered_cruise_range(MissionPerformanceSnapshot snapshot, MissionProfile profile,
                                          MissionPredictionResult result, float fuel, out float fuelUsed)
        {
            fuelUsed = 0;
            if(fuel <= 0)
                return 0;
            var env = result.Environment;
            var speed = Mathf.Clamp(profile.MaxCruiseSpeed, MinCruiseSpeed, 1000);
            snapshot.ThrustAt(profile.Body, speed, profile.StartAltitude, out var thrust, out var flow);
            if(thrust <= 0 || flow <= 0)
                return 0;
            var weight = snapshot.Mass * env.Gravity;
            var drag = 0f;
            if(env.InAtmosphere)
            {
                var mach = env.Mach1 > 0 ? speed / env.Mach1 : 0;
                var dragCoeff = (float)(AtmoSim.Cd
                                        * PhysicsGlobals.DragCurveValue(PhysicsGlobals.SurfaceCurves,
                                                                       AtmoSim.C.DragCurveK, mach)
                                        * PhysicsGlobals.DragCurvePseudoReynolds.Evaluate(env.Density * speed));
                drag = env.Density * speed * speed / 2 * dragCoeff * Mathf.Max(snapshot.MinArea, 1);
                result.MaxDynamicPressure = env.Density * speed * speed / 2000;
            }
            var requiredForce = Mathf.Sqrt(weight * weight + drag * drag);
            if(requiredForce >= thrust)
            {
                result.Warn(Loc.T("RangePlanner_CruiseTWRWarning", "Not enough thrust for sustained powered cruise at the selected speed."), 0.1f);
                return 0;
            }
            var throttle = Mathf.Clamp01(requiredForce / thrust);
            var fuelPerSecond = flow * throttle;
            if(fuelPerSecond <= 0)
                return 0;
            var time = fuel / fuelPerSecond;
            fuelUsed = fuel;
            return time * speed;
        }

        static void find_best_hop(MissionPerformanceSnapshot snapshot, MissionProfile profile,
                                  MissionPredictionResult result, float fuel)
        {
            result.BestHopDistance = 0;
            result.BestHopFuel = 0;
            result.HopCount = 0;
            if(fuel <= 0)
                return;
            var env = result.Environment;
            var maxSurfaceDistance = Mathf.Max((float)(Math.PI * env.Radius), 1000);
            var maxDv = snapshot.DeltaV(fuel);
            var roughMaxRange = maxDv * maxDv / Mathf.Max(env.Gravity, 1e-3f)
                                / Mathf.Pow(env.InAtmosphere ? BallisticLossFactorAtmosphere : BallisticLossFactorVacuum, 2);
            var scanMax = Mathf.Clamp(roughMaxRange, 250, maxSurfaceDistance);
            var bestScore = float.PositiveInfinity;
            for(int i = 1; i <= 24; i++)
            {
                var distance = scanMax * i / 24;
                var hopFuel = ballistic_hop_fuel(snapshot, env, distance);
                if(hopFuel <= 0 || hopFuel > fuel)
                    continue;
                var score = hopFuel / Mathf.Max(distance, 1);
                if(score < bestScore)
                {
                    bestScore = score;
                    result.BestHopDistance = distance;
                    result.BestHopFuel = hopFuel;
                }
            }
            if(result.BestHopFuel > 0)
                result.HopCount = Mathf.FloorToInt(fuel / result.BestHopFuel);
            else
                result.Warn(Loc.T("RangePlanner_NoHopWarning", "No feasible ballistic hop found with the selected reserve."), 0.1f);
        }

        static float ballistic_hop_fuel(MissionPerformanceSnapshot snapshot, BodyEnvironment env, float distance)
        {
            if(distance <= 0 || env == null)
                return 0;
            var centralAngle = Mathf.Clamp(distance / Mathf.Max(env.Radius, 1), 0, Mathf.PI);
            var curvedDistance = Mathf.Max(env.Radius * centralAngle, distance);
            var loss = env.InAtmosphere ? BallisticLossFactorAtmosphere : BallisticLossFactorVacuum;
            var jumpDV = Mathf.Sqrt(Mathf.Max(env.Gravity * curvedDistance, 0)) * loss;
            return snapshot.FuelNeeded(jumpDV);
        }

        static void compare_target_modes(MissionPerformanceSnapshot snapshot, MissionProfile profile,
                                         MissionPredictionResult result, float fuel)
        {
            result.TargetJumpFuel = ballistic_hop_fuel(snapshot, result.Environment, profile.TargetDistance);
            result.TargetGoToFuel = result.PoweredCruiseRange > 0
                ? fuel * profile.TargetDistance / result.PoweredCruiseRange
                : float.PositiveInfinity;
            var jumpOk = result.TargetJumpFuel > 0 && result.TargetJumpFuel <= fuel;
            var goToOk = result.TargetGoToFuel > 0 && result.TargetGoToFuel <= fuel;
            if(jumpOk && (!goToOk || result.TargetJumpFuel < result.TargetGoToFuel))
                result.Recommendation = MissionRecommendation.BallisticJump;
            else if(goToOk)
                result.Recommendation = MissionRecommendation.GoTo;
            else
            {
                result.Recommendation = MissionRecommendation.None;
                result.Warn(Loc.T("RangePlanner_TargetUnreachable", "Target is outside the estimated fuel range."), 0.15f);
            }
        }
    }
}
