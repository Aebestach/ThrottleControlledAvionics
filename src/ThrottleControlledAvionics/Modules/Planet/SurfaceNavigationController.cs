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
    public static class SurfaceNavigationController
    {
        public static Vector3 TargetVector(VesselWrapper vsl, WayPoint target, Vector3 offset)
        {
            return Vector3.ProjectOnPlane(target.GetTransform().position + offset - vsl.Physics.wCoM,
                                          vsl.Physics.Up);
        }

        public static float HorizontalDistance(VesselWrapper vsl, Vector3 targetVector)
        {
            return Utils.ClampL(targetVector.magnitude - vsl.Geometry.R, 0);
        }

        public static float BearingThreshold(VesselWrapper vsl)
        {
            return Utils.Clamp(1 / vsl.Torque.MaxCurrent.AngularAccelerationAroundAxis(vsl.Engines.CurrentDefThrustDir),
                               PointNavigator.C.BearingCutoffCos, 0.98480775f);
        }

        public static Vector3 GreatCircleDirection(VesselWrapper vsl, WayPoint target, out float horizontalDistance)
        {
            var next = target.PointFrom(vsl.vessel, PointNavigator.C.GCNavStep);
            horizontalDistance = (float)target.DistanceTo(vsl.vessel);
            return Vector3.ProjectOnPlane(vsl.Body.GetWorldSurfacePosition(next.Lat, next.Lon, vsl.vessel.altitude)
                                          - vsl.vessel.transform.position, vsl.Physics.Up);
        }
    }
}
