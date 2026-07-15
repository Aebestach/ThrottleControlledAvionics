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
    public class PredictiveBrakingController
    {
        readonly VesselWrapper VSL;
        readonly HorizontalSpeedControl HSC;
        readonly PIDf_Controller DistancePID = new PIDf_Controller();
        readonly PIDf_Controller CorrectionPID = new PIDf_Controller();
        readonly PIDvd_Controller LateralPID = new PIDvd_Controller();
        readonly AsymmetricFiterF MaxSpeed = new AsymmetricFiterF();

        public PredictiveBrakingController(VesselWrapper vsl, HorizontalSpeedControl hsc)
        {
            VSL = vsl;
            HSC = hsc;
            DistancePID.setPID(PointNavigator.C.DistancePID);
            CorrectionPID.setPID(PointNavigator.C.CorrectionPID);
            LateralPID.setPID(PointNavigator.C.LateralPID);
            MaxSpeed.TauUp = PointNavigator.C.MaxSpeedFilterUp;
            MaxSpeed.TauDown = PointNavigator.C.MaxSpeedFilterDown;
        }

        public void Reset(float maxSpeed)
        {
            DistancePID.Reset();
            CorrectionPID.Reset();
            LateralPID.Reset();
            MaxSpeed.Set(maxSpeed);
        }

        public Vector3d Apply(Vector3d error,
                              Vector3d targetVelocity,
                              float maxSpeed,
                              float stopDistance,
                              float verticalDistance = 0,
                              float obstacleDistance = 0)
        {
            var up = VSL.Physics.Up;
            var hError = Vector3d.Exclude(up, error);
            var hdist = hError.magnitude;
            var targetHorizontalVelocity = Vector3d.Exclude(up, targetVelocity);
            if(hdist < 0.1)
            {
                VSL.HorizontalSpeed.SetNeeded(targetHorizontalVelocity);
                return targetHorizontalVelocity;
            }
            var vdir = hError / hdist;
            var relVelocity = VSL.HorizontalSpeed.Vector - targetHorizontalVelocity;
            var curVel = (float)Vector3d.Dot(relVelocity, vdir);
            var brakingDistance = (float)Utils.ClampL(hdist - stopDistance, 0);
            var speedLimit = maxSpeed;
            if(curVel > 0.1f)
            {
                var brakeAccel = brake_acceleration(vdir, out var horizontalThrust);
                if(brakeAccel > 1e-5f)
                {
                    var prepTime = preparation_time(vdir, horizontalThrust);
                    var prepDistance = curVel * prepTime + stopDistance;
                    var eta = brakingDistance / Mathf.Max(curVel, 0.1f);
                    MaxSpeed.TauUp = PointNavigator.C.MaxSpeedFilterUp / eta / brakeAccel;
                    MaxSpeed.TauDown = eta * brakeAccel / PointNavigator.C.MaxSpeedFilterDown;
                    MaxSpeed.Update(prepDistance < brakingDistance
                        ? (1 + Mathf.Sqrt(1 + 2 / brakeAccel * (brakingDistance - prepDistance))) * brakeAccel
                        : 2 * brakeAccel);
                    CorrectionPID.Min = -VSL.HorizontalSpeed.Absolute;
                    if(MaxSpeed < curVel)
                        CorrectionPID.Update(MaxSpeed - curVel);
                    else
                    {
                        CorrectionPID.IntegralError *= 1 - TimeWarp.fixedDeltaTime * PointNavigator.C.CorrectionEasingRate;
                        CorrectionPID.Update(0);
                    }
                    if(HSC != null && relVelocity.sqrMagnitude > 1e-6)
                        HSC.AddRawCorrection(CorrectionPID.Action * relVelocity.normalized);
                }
                if(MaxSpeed < maxSpeed)
                    speedLimit = Mathf.Max(HorizontalSpeedControl.C.TranslationMinDeltaV + 0.1f, MaxSpeed);
            }
            var effectiveDistance = brakingDistance;
            if(verticalDistance > 0 && effectiveDistance > 0)
                effectiveDistance *= Utils.ClampL(1 - Mathf.Atan(verticalDistance / effectiveDistance) / (float)Utils.HalfPI, 0);
            if(obstacleDistance > 0 && effectiveDistance > 0)
                effectiveDistance *= Utils.ClampL(1 - obstacleDistance / effectiveDistance, 0);
            DistancePID.Min = HorizontalSpeedControl.C.TranslationMinDeltaV + 0.1f;
            DistancePID.Max = speedLimit;
            DistancePID.P = PointNavigator.C.DistancePID.P;
            DistancePID.D = PointNavigator.C.DistancePID.D;
            DistancePID.Update(effectiveDistance);
            var needed = targetHorizontalVelocity + vdir * DistancePID.Action;
            VSL.HorizontalSpeed.SetNeeded(needed);
            apply_lateral_correction(vdir, targetHorizontalVelocity);
            return needed;
        }

        float brake_acceleration(Vector3d vdir, out float horizontalThrust)
        {
            var mg2 = VSL.Physics.mg * VSL.Physics.mg;
            var brakeThrust = Mathf.Min(VSL.Physics.mg, VSL.Engines.MaxThrustM / 2 * VSL.OnPlanetParams.TWRf);
            horizontalThrust = VSL.Engines.TranslationThrustLimits.Project(VSL.LocalDir(vdir)).magnitude;
            if(horizontalThrust > brakeThrust)
                brakeThrust = horizontalThrust;
            else
                horizontalThrust = -1;
            if(brakeThrust <= 0 || VSL.Physics.M <= 0)
                return 0;
            return brakeThrust / VSL.Physics.M;
        }

        float preparation_time(Vector3d vdir, float horizontalThrust)
        {
            if(horizontalThrust >= 0)
                return 0;
            var direction = (Vector3)vdir;
            var brakeAngle = Utils.Angle2(VSL.Engines.CurrentDefThrustDir, direction) - 45;
            if(brakeAngle <= 0)
                return 0;
            var axis = Vector3.Cross(VSL.Engines.CurrentDefThrustDir, direction);
            if(VSL.Torque.Slow)
            {
                var maxThrust = Mathf.Min(Mathf.Sqrt(VSL.Physics.mg * VSL.Physics.mg
                                                     + VSL.Engines.MaxThrustM * VSL.Engines.MaxThrustM / 4),
                    VSL.Engines.MaxThrustM * 0.99f);
                return VSL.Torque.NoEngines.RotationTime3Phase(brakeAngle, axis, PointNavigator.C.RotationAccelPhase)
                       + Utils.LerpTime(VSL.Engines.Thrust.magnitude, VSL.Engines.MaxThrustM, maxThrust,
                           VSL.Engines.AccelerationSpeed);
            }
            return VSL.Torque.MaxCurrent.RotationTime2Phase(brakeAngle, axis, VSL.OnPlanetParams.GeeVSF);
        }

        void apply_lateral_correction(Vector3d vdir, Vector3d targetHorizontalVelocity)
        {
            if(HSC == null)
                return;
            var currentRelativeVelocity = VSL.HorizontalSpeed.Vector - targetHorizontalVelocity;
            var latV = -Vector3d.Exclude(vdir, currentRelativeVelocity);
            var latF = (float)Math.Min(latV.magnitude / Math.Max(VSL.HorizontalSpeed.Absolute, 0.1), 1);
            LateralPID.P = PointNavigator.C.LateralPID.P * latF;
            LateralPID.I = Math.Min(PointNavigator.C.LateralPID.I, latF);
            LateralPID.D = PointNavigator.C.LateralPID.D * latF;
            LateralPID.Update(latV);
            HSC.AddWeightedCorrection(LateralPID.Action);
        }
    }
}
