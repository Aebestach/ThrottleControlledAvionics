//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License.
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{
    public class DockingTargetFrame
    {
        public ITargetable OriginalTarget;
        public ITargetable TargetObject;
        public Vessel TargetVessel;
        public ModuleDockingNode TargetPort;
        public ModuleDockingNode ActivePort;
        public Transform TargetTransform;
        public Transform ActiveTransform;
        public Vector3d ApproachAxis;
        public Vector3d TargetVelocity;
        public bool HasTargetPort => TargetPort != null || !(TargetObject is Vessel) && TargetTransform != null;
        public bool HasActivePort => ActivePort != null;
        public bool Valid => TargetVessel != null && TargetTransform != null;
        public Vector3d TargetPosition => TargetTransform != null ? (Vector3d)TargetTransform.position : Vector3d.zero;
        public Vector3d ActivePosition => ActiveTransform != null ? (Vector3d)ActiveTransform.position : Vector3d.zero;
    }

    public class DockingTargetResolver
    {
        readonly VesselWrapper VSL;
        readonly List<ModuleDockingNode> targetPorts = new List<ModuleDockingNode>();
        readonly List<ModuleDockingNode> activePorts = new List<ModuleDockingNode>();
        readonly DockingTargetFrame frame = new DockingTargetFrame();
        Vessel cachedTargetVessel;
        Vessel cachedActiveVessel;
        ITargetable cachedOriginalTarget;
        Part cachedReferencePart;
        Vector3d approachAxis;

        public DockingTargetResolver(VesselWrapper vsl)
        { VSL = vsl; }

        public void Reset()
        {
            cachedTargetVessel = null;
            cachedActiveVessel = null;
            cachedOriginalTarget = null;
            cachedReferencePart = null;
            targetPorts.Clear();
            activePorts.Clear();
            approachAxis = Vector3d.zero;
            clear_frame();
        }

        void clear_frame()
        {
            frame.OriginalTarget = null;
            frame.TargetObject = null;
            frame.TargetVessel = null;
            frame.TargetPort = null;
            frame.ActivePort = null;
            frame.TargetTransform = null;
            frame.ActiveTransform = null;
            frame.ApproachAxis = Vector3d.zero;
            frame.TargetVelocity = Vector3d.zero;
        }

        static void collect_ports(Vessel vessel, List<ModuleDockingNode> ports)
        {
            ports.Clear();
            if(vessel == null || vessel.Parts == null)
                return;
            for(int i = 0, count = vessel.Parts.Count; i < count; i++)
            {
                var part = vessel.Parts[i];
                if(part == null)
                    continue;
                var port = part.FindModuleImplementing<ModuleDockingNode>();
                if(port != null)
                    ports.Add(port);
            }
        }

        void update_target_ports(Vessel vessel, ITargetable originalTarget)
        {
            if(vessel == cachedTargetVessel && originalTarget == cachedOriginalTarget && targetPorts.Count > 0)
                return;
            cachedTargetVessel = vessel;
            cachedOriginalTarget = originalTarget;
            collect_ports(vessel, targetPorts);
        }

        void update_active_ports()
        {
            var refPart = VSL.vessel.GetReferenceTransformPart();
            if(VSL.vessel == cachedActiveVessel && refPart == cachedReferencePart && activePorts.Count > 0)
                return;
            cachedActiveVessel = VSL.vessel;
            cachedReferencePart = refPart;
            collect_ports(VSL.vessel, activePorts);
        }

        static Transform port_transform(ModuleDockingNode port)
        {
            if(port == null)
                return null;
            if(port.part != null && !string.IsNullOrEmpty(port.nodeTransformName))
            {
                var nodeTransform = port.part.FindModelTransform(port.nodeTransformName);
                if(nodeTransform != null)
                    return nodeTransform;
            }
            return port.transform;
        }

        static Vector3 port_forward(ModuleDockingNode port)
        {
            var tr = port_transform(port);
            return tr != null ? tr.forward : Vector3.zero;
        }

        static ModuleDockingNode port_from_target(ITargetable target)
        { return target as ModuleDockingNode; }

        ModuleDockingNode best_target_port(Vector3d incomingPosition)
        {
            ModuleDockingNode best = null;
            var bestScore = double.MaxValue;
            for(int i = 0, count = targetPorts.Count; i < count; i++)
            {
                var port = targetPorts[i];
                var tr = port_transform(port);
                if(tr == null)
                    continue;
                var portPos = (Vector3d)tr.position;
                var toIncoming = incomingPosition - portPos;
                var dist = toIncoming.magnitude;
                if(dist < 1e-3)
                    continue;
                toIncoming /= dist;
                var forward = (Vector3d)port_forward(port);
                var facing = forward.sqrMagnitude > 1e-6 ? Vector3d.Dot(forward.normalized, toIncoming) : 0;
                var upFacing = forward.sqrMagnitude > 1e-6
                    ? Vector3d.Dot(forward.normalized, VSL.Physics.Up)
                    : 0;
                // Prefer the port that faces the approaching craft. For landed hover-docking
                // this strongly favors upward-facing ports over side ports at similar distance.
                var score = dist - facing * System.Math.Max(dist, 50.0);
                score -= upFacing * 25.0;
                if(score < bestScore)
                {
                    bestScore = score;
                    best = port;
                }
            }
            return best;
        }

        ModuleDockingNode best_active_port(Vector3d targetPosition)
        {
            var refPart = VSL.vessel.GetReferenceTransformPart();
            if(refPart != null)
            {
                var refPort = refPart.FindModuleImplementing<ModuleDockingNode>();
                if(refPort != null)
                    return refPort;
            }
            ModuleDockingNode best = null;
            var bestScore = double.MaxValue;
            for(int i = 0, count = activePorts.Count; i < count; i++)
            {
                var port = activePorts[i];
                var tr = port_transform(port);
                if(tr == null)
                    continue;
                var portPos = (Vector3d)tr.position;
                var toTarget = (targetPosition - portPos).normalized;
                var forward = (Vector3d)port_forward(port);
                var facing = forward.sqrMagnitude > 1e-6 ? Vector3d.Dot(forward.normalized, toTarget) : 0;
                var score = (portPos - targetPosition).magnitude - facing * 25;
                if(score < bestScore)
                {
                    bestScore = score;
                    best = port;
                }
            }
            return best;
        }

        Vector3d resolve_axis(Transform targetTransform, Vector3d incomingPosition)
        {
            if(targetTransform == null)
                return VSL.Physics.Up;
            var axis = targetTransform.forward;
            if(axis.sqrMagnitude < 1e-6f)
                return VSL.Physics.Up;
            var axisW = ((Vector3d)axis).normalized;
            // The target port's forward vector is the docking approach side: the active
            // port should sit at target + forward * distance and point back along -forward.
            // Do not flip it based on current vessel position, or the hold point moves to
            // the wrong side and the craft "descends" through the target.
            approachAxis = axisW;
            return axisW;
        }

        static ModuleDockingNode ksp_docking_port_target(Vessel targetVessel)
        {
            var kspTarget = FlightGlobals.fetch != null ? FlightGlobals.fetch.VesselTarget : null;
            var port = kspTarget as ModuleDockingNode;
            if(port == null || port.GetVessel() != targetVessel)
                return null;
            return port;
        }

        public bool Resolve(WayPoint target)
        {
            clear_frame();
            if(target == null)
                return false;
            target.Update(VSL);
            if(!target)
                return false;
            var targetObject = target.GetTarget();
            if(targetObject == null)
                return false;
            var targetVessel = targetObject.GetVessel();
            if(targetVessel == null || targetVessel == VSL.vessel)
                return false;
            if(targetVessel.mainBody != VSL.Body || !targetVessel.loaded || targetVessel.packed)
                return false;
            update_target_ports(targetVessel, targetObject);
            update_active_ports();
            var targetPort = port_from_target(targetObject);
            if(targetPort == null)
            {
                var kspPort = ksp_docking_port_target(targetVessel);
                if(kspPort != null)
                {
                    targetPort = kspPort;
                    targetObject = kspPort;
                }
            }
            var refPart = VSL.vessel.GetReferenceTransformPart();
            var activePort = refPart != null ? refPart.FindModuleImplementing<ModuleDockingNode>() : null;
            if(activePort == null)
            {
                Vector3d preTargetPos;
                if(targetPort != null)
                    preTargetPos = (Vector3d)port_transform(targetPort).position;
                else if(targetObject.GetTransform() != null)
                    preTargetPos = (Vector3d)targetObject.GetTransform().position;
                else
                    preTargetPos = (Vector3d)targetVessel.CurrentCoM;
                activePort = best_active_port(preTargetPos);
            }
            var incomingPosition = activePort != null
                ? (Vector3d)port_transform(activePort).position
                : (Vector3d)VSL.Physics.wCoM;
            if(targetPort != null)
                targetObject = targetPort;
            else if(targetObject is Vessel)
            {
                targetPort = best_target_port(incomingPosition);
                if(targetPort != null)
                    targetObject = targetPort;
            }
            var targetTransform = targetPort != null
                ? port_transform(targetPort)
                : targetObject.GetTransform();
            if(targetTransform == null)
                targetTransform = targetVessel.ReferenceTransform;
            if(activePort == null)
                activePort = best_active_port(targetTransform.position);
            frame.OriginalTarget = target.GetTarget();
            frame.TargetObject = targetObject;
            frame.TargetVessel = targetVessel;
            frame.TargetPort = targetPort;
            frame.ActivePort = activePort;
            frame.TargetTransform = targetTransform;
            frame.ActiveTransform = activePort != null ? port_transform(activePort) : VSL.refT;
            frame.ApproachAxis = resolve_axis(targetTransform, incomingPosition);
            frame.TargetVelocity = targetVessel.srf_velocity;
            return frame.Valid;
        }

        public DockingTargetFrame Frame => frame;
    }
}
