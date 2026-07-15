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
    public struct DockingPoseMetrics
    {
        public float Distance;
        public float LateralError;
        public float VerticalError;
        public float AxialDistance;
        public float LateralSpeed;
        public float ClosingSpeed;
        public float AlignmentAngle;
        public float RollError;
        public float Corridor;
    }

    public class DockingPoseController
    {
        readonly VesselWrapper VSL;
        readonly TranslationControl TRA;
        readonly AttitudeControl ATC;
        Vector3d lastAxis;

        public DockingPoseController(VesselWrapper vsl, TranslationControl tra, AttitudeControl atc)
        {
            VSL = vsl;
            TRA = tra;
            ATC = atc;
        }

        public void Reset()
        { lastAxis = Vector3d.zero; }

        public DockingPoseMetrics Metrics(DockingTargetFrame frame,
                                          float contactOffset,
                                          float maxLateralError,
                                          float finalLateralFunnel,
                                          float holdStandoff,
                                          bool alignRoll,
                                          float rollOffset)
        {
            var metrics = new DockingPoseMetrics();
            if(frame == null || !frame.Valid)
                return metrics;
            var axis = stable_axis(frame.ApproachAxis);
            var activeFromTarget = frame.ActivePosition - frame.TargetPosition;
            var relVel = (Vector3d)VSL.vessel.srf_velocity - frame.TargetVelocity;
            metrics.Distance = (float)activeFromTarget.magnitude;
            metrics.LateralError = (float)Vector3d.Exclude(axis, activeFromTarget).magnitude;
            metrics.VerticalError = (float)Vector3d.Dot(activeFromTarget, VSL.Physics.Up);
            metrics.AxialDistance = (float)Vector3d.Dot(activeFromTarget, axis) - contactOffset;
            metrics.LateralSpeed = (float)Vector3d.Exclude(axis, relVel).magnitude;
            metrics.ClosingSpeed = (float)Vector3d.Dot(relVel, -axis);
            metrics.Corridor = Corridor(metrics.AxialDistance, maxLateralError, finalLateralFunnel, holdStandoff, contactOffset);
            metrics.AlignmentAngle = frame.ActiveTransform != null
                ? Utils.Angle2(frame.ActiveTransform.forward, (Vector3)(-axis))
                : Utils.Angle2(VSL.refT != null ? VSL.refT.forward : VSL.Engines.CurrentDefThrustDir, (Vector3)(-axis));
            metrics.RollError = roll_error(frame, axis, alignRoll, rollOffset);
            return metrics;
        }

        public static float Corridor(float axialDistance,
                                     float maxLateralError,
                                     float finalLateralFunnel,
                                     float holdStandoff,
                                     float contactOffset)
        {
            var finalCorridor = Utils.ClampL(maxLateralError * 0.5f, 0.1f);
            var holdCorridor = Utils.ClampL(maxLateralError * finalLateralFunnel, finalCorridor);
            var span = Utils.ClampL(holdStandoff - contactOffset, 0.5f);
            return Mathf.Lerp(finalCorridor, holdCorridor, Utils.Clamp(axialDistance / span, 0, 1));
        }

        public static float ClosingSpeed(float axialDistance, float minSpeed, float maxSpeed)
        {
            if(axialDistance <= minSpeed)
                return minSpeed;
            return Utils.Clamp(axialDistance * 0.25f, minSpeed, maxSpeed);
        }

        public void Align(DockingTargetFrame frame, bool alignRoll, float rollOffset)
        { Align(frame, alignRoll, rollOffset, Vector3d.zero, 0); }

        public void Align(DockingTargetFrame frame,
                          bool alignRoll,
                          float rollOffset,
                          Vector3d lateralAccel,
                          float maxTiltDeg)
        {
            if(frame == null || !frame.Valid || ATC == null || VSL.refT == null)
                return;
            var axis = stable_axis(frame.ApproachAxis);
            var neededForward = hover_tilted_forward((Vector3)(-axis).normalized, lateralAccel, maxTiltDeg);
            var currentForward = frame.ActiveTransform != null ? frame.ActiveTransform.forward : VSL.refT.forward;
            if(!alignRoll || frame.TargetTransform == null)
            {
                ATC.SetCustomRotationW(currentForward, neededForward);
                return;
            }
            var neededUp = docking_up(frame, neededForward, rollOffset);
            var currentUp = frame.ActiveTransform != null ? frame.ActiveTransform.up : VSL.refT.up;
            var forwardAligned = Utils.Angle2(currentForward, neededForward);
            if(forwardAligned < 10)
            {
                var currentUpProjected = Vector3.ProjectOnPlane(currentUp, neededForward).normalized;
                if(currentUpProjected.sqrMagnitude > 1e-4f)
                {
                    ATC.SetCustomRotationW(currentUpProjected, neededUp);
                    return;
                }
            }
            ATC.SetCustomRotationW(currentForward, neededForward);
        }

        public void TranslateTo(DockingTargetFrame frame,
                                Vector3d desiredPosition,
                                Vector3d desiredRelativeVelocity,
                                float maxSpeed,
                                bool includeVertical)
        {
            if(frame == null || !frame.Valid || TRA == null)
                return;
            var desiredVelocity = frame.TargetVelocity + desiredRelativeVelocity;
            var positionError = desiredPosition - frame.ActivePosition;
            var correction = minimum_jerk_velocity(positionError, maxSpeed);
            correction = limit_by_braking_authority(correction, positionError);
            desiredVelocity += correction;
            var current = (Vector3d)VSL.vessel.srf_velocity;
            var deltaV = current - desiredVelocity;
            var localDelta = VSL.LocalDir(deltaV);
            if(!includeVertical)
            {
                var upLocal = VSL.LocalDir(VSL.Physics.Up);
                localDelta -= Vector3.Project(localDelta, upLocal);
            }
            TRA.AddDeltaV(localDelta);
        }

        Vector3d stable_axis(Vector3d axis)
        {
            if(axis.sqrMagnitude < 1e-6)
                return VSL.Physics.Up;
            axis = axis.normalized;
            if(lastAxis.sqrMagnitude > 1e-6 && Vector3d.Dot(axis, lastAxis) < 0)
                axis = -axis;
            lastAxis = axis;
            return axis;
        }

        /// <summary>
        /// Tilt the alignment attitude so hover engines produce the requested lateral
        /// acceleration (a = g*tan(tilt)). This gives lateral control authority that does
        /// not depend on RCS placement and works both in vacuum and in atmosphere.
        /// </summary>
        Vector3 hover_tilted_forward(Vector3 neededForward, Vector3d lateralAccel, float maxTiltDeg)
        {
            if(maxTiltDeg <= 0 || lateralAccel.sqrMagnitude < 1e-8)
                return neededForward;
            var g = (float)VSL.Physics.G;
            if(g < 1e-3f)
                return neededForward;
            var support = VSL.Physics.Up * g + lateralAccel;
            var rot = Quaternion.RotateTowards(Quaternion.identity,
                Quaternion.FromToRotation((Vector3)VSL.Physics.Up, (Vector3)support.normalized),
                maxTiltDeg);
            return (rot * neededForward).normalized;
        }

        /// <summary>
        /// The minimum-jerk profile alone can command speeds RCS cannot brake from
        /// (v²/2a overshoot), which makes the vessel oscillate around the hold point.
        /// Limit the correction to what the available translation authority can stop.
        /// </summary>
        Vector3d limit_by_braking_authority(Vector3d correction, Vector3d positionError)
        {
            var speed = correction.magnitude;
            var dist = positionError.magnitude;
            if(speed < 1e-4 || dist < 1e-4)
                return correction;
            var brakeDir = VSL.LocalDir((Vector3)(-positionError / dist)).normalized;
            var accel = VSL.Engines.MaxThrustRCS.Project(brakeDir).magnitude / VSL.Physics.M;
            if(accel < 0.02f)
                accel = 0.02f;
            var vMax = Math.Sqrt(2 * accel * dist) * 0.85;
            if(speed > vMax)
                correction *= vMax / speed;
            return correction;
        }

        static Vector3d minimum_jerk_velocity(Vector3d error, float maxSpeed)
        {
            var dist = error.magnitude;
            if(dist < 1e-5)
                return Vector3d.zero;
            var t = Utils.Clamp((float)(dist / Math.Max(maxSpeed, 0.01f)), 0, 1);
            var smooth = t * t * t * (10 + t * (-15 + 6 * t));
            return error / dist * maxSpeed * smooth;
        }

        static Vector3 docking_up(DockingTargetFrame frame, Vector3 neededForward, float rollOffset)
        {
            var neededUp = frame.TargetTransform != null ? frame.TargetTransform.up : Vector3.up;
            neededUp = Vector3.ProjectOnPlane(neededUp, neededForward).normalized;
            if(neededUp.sqrMagnitude < 1e-4f)
                neededUp = Vector3.ProjectOnPlane(Vector3.up, neededForward).normalized;
            if(Math.Abs(rollOffset) > 0.01f)
                neededUp = (Quaternion.AngleAxis(rollOffset, neededForward) * neededUp).normalized;
            return neededUp;
        }

        static float roll_error(DockingTargetFrame frame, Vector3d axis, bool alignRoll, float rollOffset)
        {
            if(!alignRoll || frame == null || frame.TargetTransform == null || frame.ActiveTransform == null)
                return 0;
            var neededForward = (Vector3)(-axis).normalized;
            var neededUp = docking_up(frame, neededForward, rollOffset);
            var activeUp = Vector3.ProjectOnPlane(frame.ActiveTransform.up, neededForward).normalized;
            if(activeUp.sqrMagnitude < 1e-4f)
                return 180;
            return Utils.Angle2(activeUp, neededUp);
        }
    }
}
