//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License.
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    public class TransferCandidate
    {
        public double DepartureUT;
        public double ArrivalUT;
        public Vector3d DeltaV;
        public double Score;
        public Orbit TransferOrbit;
    }

    public static class ManeuverPlannerMath
    {
        public static double NextRadiusUT(Orbit orbit, double fromUT, double radius)
        {
            if(orbit.eccentricity >= 1 || orbit.period <= 0)
                return fromUT;
            var period = orbit.period;
            var bestUT = fromUT;
            var bestErr = double.MaxValue;
            var samples = 96;
            for(var i = 0; i <= samples; i++)
            {
                var ut = fromUT + period*i/samples;
                var err = Math.Abs(orbit.getRelativePositionAtUT(ut).magnitude - radius);
                if(err < bestErr)
                {
                    bestErr = err;
                    bestUT = ut;
                }
            }
            return RefineMinimum(fromUT, fromUT + period, bestUT,
                ut => Math.Abs(orbit.getRelativePositionAtUT(ut).magnitude - radius));
        }

        public static double NextEquatorialNodeUT(Orbit orbit, double fromUT, bool ascending)
        {
            var bodyNormal = orbit.referenceBody.zUpAngularVelocity.normalized;
            var node = Vector3d.Cross(bodyNormal, orbit.GetOrbitNormal()).normalized;
            if(!ascending)
                node = -node;
            return NextDirectionUT(orbit, fromUT, node);
        }

        public static double CheapestEquatorialNodeUT(Orbit orbit, double fromUT)
        {
            var an = NextEquatorialNodeUT(orbit, fromUT, true);
            var dn = NextEquatorialNodeUT(orbit, fromUT, false);
            return orbit.hV(an).magnitude < orbit.hV(dn).magnitude ? an : dn;
        }

        public static double NextPlaneNodeUT(Orbit orbit, Orbit target, double fromUT, bool ascending)
        {
            var node = Vector3d.Cross(orbit.GetOrbitNormal(), target.GetOrbitNormal()).normalized;
            if(!ascending)
                node = -node;
            return NextDirectionUT(orbit, fromUT, node);
        }

        public static double CheapestPlaneNodeUT(Orbit orbit, Orbit target, double fromUT)
        {
            var an = NextPlaneNodeUT(orbit, target, fromUT, true);
            var dn = NextPlaneNodeUT(orbit, target, fromUT, false);
            return orbit.hV(an).magnitude < orbit.hV(dn).magnitude ? an : dn;
        }

        public static double NextDirectionUT(Orbit orbit, double fromUT, Vector3d direction)
        {
            if(orbit.period <= 0 || direction.sqrMagnitude <= 0)
                return fromUT;
            direction.Normalize();
            var period = orbit.period;
            var bestUT = fromUT;
            var bestErr = double.MaxValue;
            var samples = 96;
            for(var i = 0; i <= samples; i++)
            {
                var ut = fromUT + period*i/samples;
                var pos = orbit.getRelativePositionAtUT(ut).normalized;
                var err = 1 - Vector3d.Dot(pos, direction);
                if(err < bestErr)
                {
                    bestErr = err;
                    bestUT = ut;
                }
            }
            return RefineMinimum(fromUT, fromUT + period, bestUT,
                ut => 1 - Vector3d.Dot(orbit.getRelativePositionAtUT(ut).normalized, direction));
        }

        static double RefineMinimum(double minUT, double maxUT, double centerUT, Func<double, double> metric)
        {
            var span = (maxUT - minUT)/24;
            var a = Math.Max(minUT, centerUT - span);
            var b = Math.Min(maxUT, centerUT + span);
            for(var i = 0; i < 32; i++)
            {
                var m1 = a + (b - a)/3;
                var m2 = b - (b - a)/3;
                if(metric(m1) < metric(m2))
                    b = m2;
                else
                    a = m1;
            }
            return (a + b)/2;
        }

        public static Vector3d DeltaVToChangeInclination(Orbit orbit, double ut, double targetInclination)
        {
            var incDelta = (targetInclination - orbit.inclination)*Math.PI/180;
            if(Math.Abs(incDelta) < 1e-6)
                return Vector3d.zero;
            var speed = orbit.hV(ut).magnitude;
            var normal = orbit.GetOrbitNormal().normalized;
            var sign = Math.Sign(incDelta);
            return normal*sign*(2*speed*Math.Sin(Math.Abs(incDelta)/2));
        }

        public static Vector3d DeltaVToMatchPlane(Orbit orbit, Orbit target, double ut)
        {
            var pos = orbit.getRelativePositionAtUT(ut);
            var vel = orbit.getOrbitalVelocityAtUT(ut);
            var hvel = Vector3d.Exclude(pos, vel);
            var targetNormal = target.GetOrbitNormal().normalized;
            var desired = Vector3d.Cross(targetNormal, pos).normalized*hvel.magnitude;
            if(Vector3d.Dot(desired, hvel) < 0)
                desired = -desired;
            return desired - hvel;
        }

        public static TransferCandidate BestLambertTransfer(Orbit origin, Orbit target, double startUT,
                                                            double maxStartDelay, double minTransferTime,
                                                            double maxTransferTime, int samples,
                                                            LambertSolver solver)
        {
            TransferCandidate best = null;
            samples = Math.Max(samples, 4);
            for(var i = 0; i < samples; i++)
            {
                var departure = startUT + maxStartDelay*i/Math.Max(samples - 1, 1);
                for(var j = 0; j < samples; j++)
                {
                    var transfer = minTransferTime +
                                   (maxTransferTime - minTransferTime)*j/Math.Max(samples - 1, 1);
                    if(transfer <= 0)
                        continue;
                    var destination = target.getRelativePositionAtUT(departure + transfer);
                    solver.Init(origin, destination, departure);
                    var dV = solver.dV4Transfer(transfer);
                    if(dV.sqrMagnitude <= 0 || double.IsNaN(dV.sqrMagnitude))
                        continue;
                    var transferOrbit = TrajectoryCalculator.NewOrbit(origin, dV, departure);
                    var miss = (transferOrbit.getRelativePositionAtUT(departure + transfer) - destination).magnitude;
                    var score = dV.magnitude + miss/1000;
                    if(best == null || score < best.Score)
                        best = new TransferCandidate
                        {
                            DepartureUT = departure,
                            ArrivalUT = departure + transfer,
                            DeltaV = dV,
                            Score = score,
                            TransferOrbit = transferOrbit
                        };
                }
            }
            return best;
        }

        public static Vector3d DeltaVToEject(Orbit localOrbit, Vector3d vInf, double ut)
        {
            var r = localOrbit.getRelativePositionAtUT(ut);
            var vel = localOrbit.getOrbitalVelocityAtUT(ut);
            var hvel = Vector3d.Exclude(r, vel);
            var dir = vInf.sqrMagnitude > 0 ? vInf.normalized : hvel.normalized;
            if(Vector3d.Dot(dir, hvel) < 0)
                dir = -dir;
            var requiredSpeed = Math.Sqrt(vInf.sqrMagnitude + 2*localOrbit.referenceBody.gravParameter/r.magnitude);
            return dir*requiredSpeed - vel;
        }
    }
}
