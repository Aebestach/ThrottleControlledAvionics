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
using System.Threading.Tasks;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    public class AdvancedToOrbitExecutor : TargetedToOrbitExecutor
    {
        public new class Config : ComponentConfig<Config>
        {
            [Persistent] public float ReplanPeriod = 5f;
            [Persistent] public float ReplanError = 1500f;
            [Persistent] public float OptimizerHorizon = 900f;
            [Persistent] public float OptimizerStep = 0.5f;
            [Persistent] public float TerminalPeTolerance = 1500f;
            [Persistent] public float TerminalApTolerance = 1500f;
            [Persistent] public float TerminalInclinationTolerance = 0.05f;
            [Persistent] public float TerminalStartF = 0.75f;
            [Persistent] public float DynamicPressureMargin = 0.9f;
            [Persistent] public float QAlphaLimit = 250f;
            [Persistent] public float GuidanceBlendTime = 20f;
        }

        public static new Config C => Config.INST;

        readonly AscentTrajectoryOptimizer optimizer = new AscentTrajectoryOptimizer();
        AscentSolution solution;
        double lastPlanUT = -1;
        int lastStage = int.MinValue;
        double lastPlanError = double.PositiveInfinity;
        float solutionAge;

        public override void Reset()
        {
            base.Reset();
            optimizer.Reset();
            solution = null;
            lastPlanUT = -1;
            lastStage = int.MinValue;
            lastPlanError = double.PositiveInfinity;
            solutionAge = 0;
        }

        public override void StartGravityTurn()
        {
            base.StartGravityTurn();
            request_solution(true);
        }

        public bool AdvancedGravityTurn(float Dtol, double targetInclination)
        {
            UpdateTargetPosition();
            VSL.Engines.ActivateNextStageOnFlameout();
            update_state(Dtol);
            optimizer.AcceptCompleted(ref solution);
            solutionAge = solution != null ? (float)(VSL.Physics.UT - solution.StartUT) : 0;
            request_solution(need_replan());

            var pgVel = get_pg_vel();
            currentAoA = Utils.Angle2(VSL.Engines.CurrentDefThrustDir, -(Vector3)pgVel.xzy);
            if(ShouldCoast && !terminal_guidance_active())
                return coast(pgVel, Dtol);
            if(terminal_orbit_satisfied(Dtol, targetInclination))
                return false;

            tune_THR();
            auto_ApA_offset();

            var command = guidance_command(pgVel, Dtol, targetInclination);
            THR.CorrectThrottle = false;
            THR.Throttle = apply_limits(command.Throttle, command.Direction, pgVel, Dtol);
            CFG.AT.OnIfNot(Attitude.Custom);
            ATC.SetThrustDirW(-command.Direction.xzy);

            if(CFG.AT.Not(Attitude.KillRotation))
            {
                if(command.UseBearing)
                {
                    CFG.BR.OnIfNot(BearingMode.Auto);
                    BRC.ForwardDirection = htdir.xzy;
                }
                else
                    CFG.BR.OffIfOn(BearingMode.Auto);
            }

            prevApA = VesselOrbit.ApA;
            Status(solution == null || !solution.IsValid
                       ? Loc.T("ToOrbit_AdvancedSolving", "Advanced guidance: solving...")
                       : Loc.T("ToOrbit_AdvancedGuidance", "Advanced guidance..."));
            return true;
        }

        void request_solution(bool force)
        {
            if(!force && solution != null && optimizer.Running)
                return;
            var model = AscentVesselModel.FromVessel(VSL, VesselOrbit, Body, target, TargetR,
                MaxG, MaxDynP, MaxAoA, MinThrottle, TimeToApA);
            if(!model.IsValid)
                return;
            if(optimizer.Request(model, solution, force))
            {
                lastPlanUT = VSL.Physics.UT;
                lastStage = VSL.vessel.currentStage;
                lastPlanError = ErrorThreshold.Value;
            }
        }

        bool need_replan()
        {
            if(solution == null || !solution.IsValid)
                return true;
            if(lastStage != VSL.vessel.currentStage)
                return true;
            if(lastPlanUT < 0 || VSL.Physics.UT - lastPlanUT > C.ReplanPeriod)
                return true;
            return Math.Abs(ErrorThreshold.Value - lastPlanError) > C.ReplanError;
        }

        bool terminal_guidance_active()
        {
            if(TargetR <= Body.Radius)
                return false;
            var progress = Utils.Clamp((VesselOrbit.ApR - Body.Radius)
                                       / (TargetR - Body.Radius), 0, 1);
            return progress >= C.TerminalStartF
                   || VesselOrbit.PeR > Body.Radius + Math.Min(Body.atmosphereDepth, TargetR - Body.Radius);
        }

        bool terminal_orbit_satisfied(float Dtol, double targetInclination)
        {
            if(VesselOrbit.PeR <= Body.Radius)
                return false;
            var apTol = Math.Max(Dtol, C.TerminalApTolerance);
            var peTol = Math.Max(Dtol, C.TerminalPeTolerance);
            var apOk = Math.Abs(VesselOrbit.ApR - TargetR) < apTol;
            var peOk = Math.Abs(VesselOrbit.PeR - TargetR) < peTol;
            var incOk = Math.Abs(VesselOrbit.inclination - targetInclination) < C.TerminalInclinationTolerance;
            var h = Vector3d.Cross(VesselOrbit.pos, VesselOrbit.vel).magnitude;
            var targetH = Math.Sqrt(Body.gravParameter * TargetR);
            var hTol = Math.Sqrt(Body.gravParameter / TargetR) * Math.Max(apTol, peTol);
            var hOk = Math.Abs(h - targetH) < hTol;
            return apOk && peOk && incOk && hOk;
        }

        AscentGuidanceCommand guidance_command(Vector3d pgVel, float Dtol, double targetInclination)
        {
            var command = AscentGuidanceCommand.FromDirection(pgVel, 1, true);
            if(solution != null && solution.IsValid)
            {
                var sample = solution.Sample(VSL.Physics.UT);
                if(sample.IsValid)
                    command = AscentGuidanceCommand.FromDirection(sample.ThrustDirection, sample.Throttle, true);
            }

            var terminal = terminal_command(pgVel, targetInclination);
            if(terminal.IsValid)
            {
                var terminalF = Utils.Clamp((VesselOrbit.ApR - Body.Radius)
                                            / Math.Max(TargetR - Body.Radius, 1), 0, 1);
                terminalF = Utils.Clamp((terminalF - C.TerminalStartF) / (1 - C.TerminalStartF), 0, 1);
                if(VesselOrbit.PeR > Body.Radius)
                    terminalF = Math.Max(terminalF, 0.5);
                command.Direction = blend(command.Direction, terminal.Direction, terminalF).normalized;
                command.Throttle = Mathf.Lerp(command.Throttle, terminal.Throttle, (float)terminalF);
                command.UseBearing = terminal.UseBearing;
            }
            else if(solution == null || !solution.IsValid)
            {
                var closedLoop = TrajectoryCalculator.dV4ApV(VesselOrbit, target, VSL.Physics.UT);
                if(closedLoop.IsInvalid() || closedLoop.IsZero())
                    closedLoop = pgVel;
                var blendF = Utils.ClampH(TimeToClosestApA / Math.Max(C.GuidanceBlendTime, 1), 1);
                command.Direction = blend(closedLoop, pgVel, blendF).normalized;
            }

            throttle.Update(TimeToApA - (float)TimeToClosestApA);
            command.Throttle = Utils.Clamp(command.Throttle + throttle, MinThrottle / 100, max_G_throttle());
            command.Direction = tune_needed_vel(command.Direction, pgVel, getStartF());
            return command;
        }

        AscentGuidanceCommand terminal_command(Vector3d pgVel, double targetInclination)
        {
            if(!terminal_guidance_active())
                return AscentGuidanceCommand.Invalid;
            var horizontal = Vector3d.Cross(VesselOrbit.GetOrbitNormal(), VesselOrbit.pos).normalized;
            var targetNormal = Vector3d.Cross(VesselOrbit.pos, target).normalized;
            if(!targetNormal.IsInvalid() && !targetNormal.IsZero())
            {
                var targetHorizontal = Vector3d.Cross(targetNormal, VesselOrbit.pos).normalized;
                var incError = Math.Abs(VesselOrbit.inclination - targetInclination);
                var incF = Utils.Clamp(incError / 2, 0, 1);
                horizontal = blend(horizontal, targetHorizontal, incF).normalized;
            }
            var circularVel = horizontal * Math.Sqrt(Body.gravParameter / VesselOrbit.radius);
            var dV = circularVel - VesselOrbit.vel;
            if(dV.IsInvalid() || dV.IsZero())
                dV = pgVel;
            var apErr = TargetR - VesselOrbit.ApR;
            var peErr = TargetR - VesselOrbit.PeR;
            var h = Vector3d.Cross(VesselOrbit.pos, VesselOrbit.vel).magnitude;
            var targetH = Math.Sqrt(Body.gravParameter * TargetR);
            var hTol = Math.Sqrt(Body.gravParameter / TargetR) * Math.Max(C.TerminalPeTolerance, 1);
            var hErr = targetH - h;
            var throttleF = Utils.Clamp((Math.Max(Math.Max(apErr, peErr), Math.Abs(hErr) / Math.Max(hTol, 1) * C.TerminalPeTolerance)
                                         + C.TerminalPeTolerance)
                                        / Math.Max(C.TerminalPeTolerance * 4, 1), 0, 1);
            if(hErr < -hTol && apErr < 0)
                throttleF = 0;
            if(peErr > C.TerminalPeTolerance)
                throttleF = Math.Max(throttleF, MinThrottle / 100);
            return AscentGuidanceCommand.FromDirection(dV, (float)throttleF, false);
        }

        float apply_limits(float requestedThrottle, Vector3d commandDirection, Vector3d pgVel, float Dtol)
        {
            var maxThrottle = max_G_throttle();
            if(MaxDynP > 0 && VSL.vessel.dynamicPressurekPa > MaxDynP * C.DynamicPressureMargin)
            {
                var overQ = ((float)VSL.vessel.dynamicPressurekPa - MaxDynP * C.DynamicPressureMargin)
                            / Math.Max(MaxDynP * (1 - C.DynamicPressureMargin), 1);
                maxThrottle = Mathf.Min(maxThrottle, Utils.Clamp(1 - overQ, 0, 1));
            }
            var qAlpha = (float)VSL.vessel.dynamicPressurekPa * Utils.Angle2(commandDirection, pgVel);
            if(C.QAlphaLimit > 0 && qAlpha > C.QAlphaLimit)
                maxThrottle = Mathf.Min(maxThrottle, (float)Utils.Clamp(C.QAlphaLimit / qAlpha, 0, 1));
            var terminalF = terminal_guidance_active() ? 1 : (float)Utils.ClampH(dApA / Dtol / Math.Max(VSL.Engines.TMR, 1e-3), 1);
            return Utils.Clamp(requestedThrottle * terminalF, 0, maxThrottle);
        }

        static Vector3d blend(Vector3d a, Vector3d b, double t)
        {
            if(a.IsInvalid() || a.IsZero())
                return b;
            if(b.IsInvalid() || b.IsZero())
                return a;
            return Vector3d.Lerp(a.normalized, b.normalized, (float)Utils.Clamp(t, 0, 1)).normalized;
        }

        public void DrawAdvancedInfo()
        {
            GUILayout.BeginHorizontal();
            GUILayout.BeginVertical();
            GUILayout.Label(Loc.T("ToOrbit_AdvancedStatus", "Advanced Status:"));
            GUILayout.Label(Loc.T("ToOrbit_AdvancedSolutionAge", "Solution Age:"));
            GUILayout.EndVertical();
            GUILayout.BeginVertical();
            GUILayout.Label(optimizer.Running
                                ? Loc.T("ToOrbit_AdvancedSolvingShort", "Solving")
                                : solution != null && solution.IsValid
                                    ? Loc.T("ToOrbit_AdvancedReady", "Ready")
                                    : Loc.T("ToOrbit_AdvancedFallback", "Fallback"));
            GUILayout.Label($"{solutionAge:F1}s");
            GUILayout.EndVertical();
            GUILayout.EndHorizontal();
        }
    }

    public struct AscentGuidanceCommand
    {
        public Vector3d Direction;
        public float Throttle;
        public bool UseBearing;
        public bool IsValid;

        public static AscentGuidanceCommand Invalid => new AscentGuidanceCommand();

        public static AscentGuidanceCommand FromDirection(Vector3d direction, float throttle, bool useBearing)
        {
            return new AscentGuidanceCommand
            {
                Direction = direction.IsInvalid() || direction.IsZero() ? Vector3d.zero : direction.normalized,
                Throttle = Utils.Clamp(throttle, 0, 1),
                UseBearing = useBearing,
                IsValid = !(direction.IsInvalid() || direction.IsZero())
            };
        }
    }

    public class AscentVesselModel
    {
        public Vector3d Position;
        public Vector3d Velocity;
        public Vector3d Target;
        public Vector3d Up;
        public Vector3d Horizontal;
        public double UT;
        public double BodyRadius;
        public double Mu;
        public double AtmosphereDepth;
        public double RhoASL;
        public double ScaleHeight;
        public double Mass;
        public double DryMass;
        public double Thrust;
        public double MassFlow;
        public double ReferenceArea;
        public double TargetR;
        public int CurrentStage;
        public readonly List<AscentStageModel> Stages = new List<AscentStageModel>();
        public float MaxG;
        public float MaxDynPressure;
        public float MaxAoA;
        public float MinThrottle;
        public float TimeToApA;
        public bool IsValid;

        public static AscentVesselModel FromVessel(VesselWrapper vsl, Orbit orbit, CelestialBody body,
                                                   Vector3d target, double targetR,
                                                   float maxG, float maxDynPressure,
                                                   float maxAoA, float minThrottle,
                                                   float timeToApA)
        {
            var thrust = vsl.Engines.MaxThrustM;
            var massFlow = vsl.Engines.MaxMassFlow;
            var mass = vsl.Physics.M;
            var dryMass = Math.Max(1, mass - vsl.Engines.AvailableFuelMass);
            var position = orbit.pos;
            var velocity = orbit.vel;
            var up = position.normalized;
            var horizontal = Vector3d.Exclude(position, target - position).normalized;
            if(horizontal.IsInvalid() || horizontal.IsZero())
                horizontal = Vector3d.Exclude(position, velocity).normalized;
            var atmosphereDepth = body.atmosphere ? body.atmosphereDepth : 0;
            var scaleHeight = atmosphereDepth > 0 ? atmosphereDepth / 5.5 : 1;
            var model = new AscentVesselModel
            {
                Position = position,
                Velocity = velocity,
                Target = target,
                Up = up,
                Horizontal = horizontal,
                UT = vsl.Physics.UT,
                BodyRadius = body.Radius,
                Mu = body.gravParameter,
                AtmosphereDepth = atmosphereDepth,
                RhoASL = body.atmosphere ? Math.Max(body.atmDensityASL, 1e-6) : 0,
                ScaleHeight = scaleHeight,
                Mass = mass,
                DryMass = dryMass,
                Thrust = thrust,
                MassFlow = massFlow,
                ReferenceArea = Math.Max(vsl.Geometry.AreaInDirection(vsl.Engines.CurrentMaxThrustDir), 0.1f),
                TargetR = targetR,
                CurrentStage = vsl.vessel.currentStage,
                MaxG = maxG,
                MaxDynPressure = maxDynPressure,
                MaxAoA = maxAoA,
                MinThrottle = minThrottle,
                TimeToApA = timeToApA,
                IsValid = thrust > 0 && massFlow > 0 && mass > 0 && targetR > body.Radius
            };
            model.Stages.Add(new AscentStageModel
            {
                Stage = vsl.vessel.currentStage,
                Thrust = thrust,
                MassFlow = massFlow,
                FuelMass = Math.Max(0, mass - dryMass),
                MinThrottle = minThrottle / 100
            });
            if(vsl.Engines.HaveNextStageEngines)
            {
                var next = vsl.Engines.GetNearestEnginedStageStats();
                if(next.MaxThrust.magnitude > 0 && next.MaxMassFlow > 0)
                    model.Stages.Add(new AscentStageModel
                    {
                        Stage = vsl.Engines.NearestEnginedStage,
                        Thrust = next.MaxThrust.magnitude,
                        MassFlow = next.MaxMassFlow,
                        FuelMass = Math.Max(0, mass - dryMass),
                        MinThrottle = minThrottle / 100
                    });
            }
            return model;
        }
    }

    public class AscentStageModel
    {
        public int Stage;
        public double Thrust;
        public double MassFlow;
        public double FuelMass;
        public double MinThrottle;
    }

    public class AscentSolution
    {
        public readonly List<AscentSolutionPoint> Points = new List<AscentSolutionPoint>();
        public double StartUT;
        public double Score;
        public bool IsValid => Points.Count > 1;

        public AscentSolutionPoint Sample(double UT)
        {
            if(!IsValid)
                return AscentSolutionPoint.Invalid;
            if(UT <= Points[0].UT)
                return Points[0];
            for(int i = 1; i < Points.Count; i++)
            {
                var p = Points[i];
                if(UT > p.UT)
                    continue;
                var p0 = Points[i - 1];
                var t = (UT - p0.UT) / Math.Max(p.UT - p0.UT, 1e-6);
                return new AscentSolutionPoint
                {
                    UT = UT,
                    ThrustDirection = Vector3d.Lerp(p0.ThrustDirection, p.ThrustDirection, (float)t).normalized,
                    Throttle = Mathf.Lerp(p0.Throttle, p.Throttle, (float)t),
                    IsValid = true
                };
            }
            return Points[Points.Count - 1];
        }
    }

    public struct AscentSolutionPoint
    {
        public double UT;
        public Vector3d ThrustDirection;
        public float Throttle;
        public bool IsValid;

        public static AscentSolutionPoint Invalid => new AscentSolutionPoint();
    }

    public class AscentTrajectoryOptimizer
    {
        Task<AscentSolution> task;
        readonly object sync = new object();

        public bool Running
        {
            get
            {
                lock(sync)
                    return task != null && !task.IsCompleted;
            }
        }

        public void Reset()
        {
            lock(sync)
                task = null;
        }

        public bool Request(AscentVesselModel model, AscentSolution previous, bool force)
        {
            lock(sync)
            {
                if(task != null && !task.IsCompleted && !force)
                    return false;
                if(task != null && !task.IsCompleted)
                    return false;
                task = Task.Factory.StartNew(() => Optimize(model, previous));
                return true;
            }
        }

        public void AcceptCompleted(ref AscentSolution solution)
        {
            lock(sync)
            {
                if(task == null || !task.IsCompleted)
                    return;
                if(!task.IsFaulted && !task.IsCanceled && task.Result != null && task.Result.IsValid)
                    solution = task.Result;
                task = null;
            }
        }

        static AscentSolution Optimize(AscentVesselModel model, AscentSolution previous)
        {
            var best = previous != null && previous.IsValid ? previous : null;
            var bestScore = best != null ? best.Score : double.PositiveInfinity;
            var turnPowers = new[] { 0.65, 0.8, 1.0, 1.25, 1.55 };
            var turnStarts = new[] { 0.02, 0.05, 0.08, 0.12 };
            for(int i = 0; i < turnPowers.Length; i++)
            for(int j = 0; j < turnStarts.Length; j++)
            {
                var solution = Simulate(model, turnPowers[i], turnStarts[j]);
                if(solution != null && solution.IsValid && solution.Score < bestScore)
                {
                    best = solution;
                    bestScore = solution.Score;
                }
            }
            return best;
        }

        static AscentSolution Simulate(AscentVesselModel model, double turnPower, double turnStart)
        {
            var solution = new AscentSolution { StartUT = model.UT };
            var r = model.Position;
            var v = model.Velocity;
            var m = model.Mass;
            var dt = Math.Max(AdvancedToOrbitExecutor.C.OptimizerStep, 0.1f);
            var maxT = Math.Max(AdvancedToOrbitExecutor.C.OptimizerHorizon, 60f);
            var targetAltitude = model.TargetR - model.BodyRadius;
            var initialAltitude = Math.Max(r.magnitude - model.BodyRadius, 0);
            var burn = true;
            double fuelUsed = 0;
            OrbitalEstimate estimate = OrbitalEstimate.FromState(r, v, model.Mu);
            for(double t = 0; t <= maxT && r.magnitude > model.BodyRadius; t += dt)
            {
                var up = r.normalized;
                var horizontal = Vector3d.Exclude(r, model.Target - r).normalized;
                if(horizontal.IsInvalid() || horizontal.IsZero())
                    horizontal = Vector3d.Exclude(r, v).normalized;
                var altitude = r.magnitude - model.BodyRadius;
                var progress = Utils.Clamp((altitude - initialAltitude)
                                           / Math.Max(targetAltitude - initialAltitude, 1), 0, 1);
                var turnF = Utils.Clamp((progress - turnStart) / Math.Max(1 - turnStart, 1e-3), 0, 1);
                turnF = Math.Pow(turnF, turnPower);
                var direction = Vector3d.Lerp(up, horizontal, (float)turnF).normalized;
                var prograde = Vector3d.Exclude(r, v).normalized;
                if(!prograde.IsInvalid() && !prograde.IsZero())
                {
                    var aoaLimit = model.MaxAoA * Math.Max(0, 1 - DynamicPressureKPa(model, r, v) / Math.Max(model.MaxDynPressure, 1));
                    direction = ClampDirection(direction, prograde, aoaLimit);
                }
                estimate = OrbitalEstimate.FromState(r, v, model.Mu);
                burn &= estimate.ApR < model.TargetR || estimate.PeR < model.TargetR * 0.98;
                var throttle = burn ? MaxThrottle(model, r, v, m, direction) : 0;
                solution.Points.Add(new AscentSolutionPoint
                {
                    UT = model.UT + t,
                    ThrustDirection = direction,
                    Throttle = throttle,
                    IsValid = true
                });
                if(!burn && estimate.ApR >= model.TargetR * 0.995)
                    break;
                var gravity = r * (model.Mu / Math.Pow(r.magnitude, 3));
                var drag = DragAcceleration(model, r, v, m);
                if(throttle > 0 && m > model.DryMass)
                {
                    var dm = Math.Min(model.MassFlow * throttle * dt, m - model.DryMass);
                    m -= dm;
                    fuelUsed += dm;
                    v += direction * (model.Thrust / m * throttle * dt);
                }
                v -= (gravity + drag) * dt;
                r += v * dt;
            }
            estimate = OrbitalEstimate.FromState(r, v, model.Mu);
            var apErr = Math.Abs(estimate.ApR - model.TargetR);
            var peErr = estimate.PeR > 0 ? Math.Abs(estimate.PeR - model.TargetR) : model.TargetR;
            var incErr = Math.Abs(estimate.Inclination - OrbitalEstimate.TargetInclination(model.Position, model.Target));
            solution.Score = apErr + peErr * 2 + incErr * model.BodyRadius / 10 + fuelUsed * 0.1;
            return solution;
        }

        static float MaxThrottle(AscentVesselModel model, Vector3d r, Vector3d v, double mass, Vector3d direction)
        {
            var maxThrottle = 1f;
            var g = model.Mu / r.sqrMagnitude;
            var accelG = (model.Thrust / mass - g) / Math.Max(g, 1e-6);
            if(accelG > model.MaxG)
                maxThrottle = Mathf.Min(maxThrottle, (float)(model.MaxG / accelG));
            var q = DynamicPressureKPa(model, r, v);
            if(model.MaxDynPressure > 0 && q > model.MaxDynPressure)
                maxThrottle = Mathf.Min(maxThrottle, (float)Utils.Clamp(model.MaxDynPressure / q, 0, 1));
            return Utils.Clamp(maxThrottle, model.MinThrottle / 100, 1);
        }

        static Vector3d DragAcceleration(AscentVesselModel model, Vector3d r, Vector3d v, double mass)
        {
            var altitude = r.magnitude - model.BodyRadius;
            if(altitude > model.AtmosphereDepth || model.RhoASL <= 0)
                return Vector3d.zero;
            var speed = v.magnitude;
            if(speed <= 0)
                return Vector3d.zero;
            var rho = Density(model, altitude);
            var drag = 0.5 * rho * speed * speed * AtmoSim.Cd * model.ReferenceArea / Math.Max(mass, 1);
            return v.normalized * drag;
        }

        static double DynamicPressureKPa(AscentVesselModel model, Vector3d r, Vector3d v)
        {
            var altitude = r.magnitude - model.BodyRadius;
            if(altitude > model.AtmosphereDepth || model.RhoASL <= 0)
                return 0;
            return 0.5 * Density(model, altitude) * v.sqrMagnitude / 1000;
        }

        static double Density(AscentVesselModel model, double altitude) =>
            model.RhoASL * Math.Exp(-Math.Max(altitude, 0) / Math.Max(model.ScaleHeight, 1));

        static Vector3d ClampDirection(Vector3d direction, Vector3d reference, double maxAngle)
        {
            if(maxAngle >= 90)
                return direction;
            var angle = Utils.Angle2(direction, reference);
            if(angle <= maxAngle)
                return direction;
            return Vector3d.Lerp(reference.normalized, direction.normalized, (float)(maxAngle / Math.Max(angle, 1e-6))).normalized;
        }
    }

    struct OrbitalEstimate
    {
        public double ApR;
        public double PeR;
        public double Inclination;

        public static OrbitalEstimate FromState(Vector3d r, Vector3d v, double mu)
        {
            var h = Vector3d.Cross(r, v);
            var hM = h.magnitude;
            if(hM <= 0 || r.magnitude <= 0)
                return new OrbitalEstimate { ApR = 0, PeR = 0, Inclination = 0 };
            var eV = Vector3d.Cross(v, h) / mu - r / r.magnitude;
            var e = eV.magnitude;
            var energy = v.sqrMagnitude / 2 - mu / r.magnitude;
            var a = energy < 0 ? -mu / (2 * energy) : r.magnitude;
            var ap = e < 1 ? a * (1 + e) : double.PositiveInfinity;
            var pe = a * (1 - e);
            return new OrbitalEstimate
            {
                ApR = ap,
                PeR = pe,
                Inclination = Math.Acos(Utils.Clamp(h.z / hM, -1, 1)) * Mathf.Rad2Deg
            };
        }

        public static double TargetInclination(Vector3d position, Vector3d target)
        {
            var normal = Vector3d.Cross(position, target);
            return normal.IsInvalid() || normal.IsZero()
                       ? 0
                       : Math.Acos(Utils.Clamp(normal.normalized.z, -1, 1)) * Mathf.Rad2Deg;
        }
    }
}
