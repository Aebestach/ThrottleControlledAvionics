//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//
using System.Collections.Generic;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{

    public class GeometryProps : VesselProps
    {
        public GeometryProps(VesselWrapper vsl) : base(vsl) {}

        public Bounds  B { get; private set; } //bounds, including exhaust tails
        public Vector3 C { get; private set; } //center
        public float   H { get; private set; } //height
        public float   BottomH { get; private set; } //CoM-to-lowest physical collider distance
        public bool    HavePhysicalBottom { get; private set; }
        public float   R { get; private set; } //radius
        public float   E { get; private set; } //radius including engines' exhaust
        public float   D { get; private set; } //diamiter
        public Vector3 RelC { get; private set; } //center relative to CoM
        public float   Area { get; private set; }
        public float   AreaWithBrakes { get; private set; }
        public Vector3 BoundsSideAreas { get; private set; }

        int cached_parts_count = -1;
        int cached_stage = -1;
        int cached_engines_count = -1;
        double next_bounds_update = -1;
        bool bounds_dirty = true;
        Bounds cached_physical_bounds;
        readonly List<Collider> cached_colliders = new List<Collider>();
        Transform cached_refT;
        const double BoundsUpdatePeriod = 0.25;

        public void InvalidateBounds() { bounds_dirty = true; }

        public float DistToBounds(Vector3 world_point)
        {
            if(refT == null)
                return 0;
            return Mathf.Sqrt(B.SqrDistance(refT.InverseTransformPoint(world_point)));
        }

        static float min_projection(Bounds b, Vector3 dir)
        {
            var min = b.min;
            var max = b.max;
            return Mathf.Min(
                Vector3.Dot(new Vector3(min.x, min.y, min.z), dir),
                Vector3.Dot(new Vector3(min.x, min.y, max.z), dir),
                Vector3.Dot(new Vector3(min.x, max.y, min.z), dir),
                Vector3.Dot(new Vector3(min.x, max.y, max.z), dir),
                Vector3.Dot(new Vector3(max.x, min.y, min.z), dir),
                Vector3.Dot(new Vector3(max.x, min.y, max.z), dir),
                Vector3.Dot(new Vector3(max.x, max.y, min.z), dir),
                Vector3.Dot(new Vector3(max.x, max.y, max.z), dir));
        }

        void update_physical_bottom()
        {
            BottomH = H;
            HavePhysicalBottom = false;
            if(cached_colliders.Count == 0)
                return;
            var up = (Vector3)VSL.Physics.Up;
            var com = Vector3.Dot(VSL.Physics.wCoM, up);
            var bottom = float.PositiveInfinity;
            for(int i = cached_colliders.Count - 1; i >= 0; i--)
            {
                var c = cached_colliders[i];
                if(c == null)
                {
                    cached_colliders.RemoveAt(i);
                    continue;
                }
                if(!c.enabled || c.isTrigger || !c.gameObject.activeInHierarchy)
                    continue;
                bottom = Mathf.Min(bottom, min_projection(c.bounds, up));
            }
            if(float.IsPositiveInfinity(bottom))
                return;
            BottomH = Utils.ClampL(com - bottom, 0);
            HavePhysicalBottom = true;
        }

        void update_physical_props(Bounds b)
        {
            if(refT == null || VSL.vessel == null)
                return;
            C = refT.TransformPoint(b.center);
            RelC = C-VSL.vessel.CoM;
            H = Mathf.Abs(Vector3.Dot(refT.TransformDirection(b.extents), VSL.Physics.Up)) -
                Vector3.Dot(RelC, VSL.Physics.Up);
            R = b.extents.magnitude;
            D = R*2;
            BoundsSideAreas = new Vector3(b.extents.y*b.extents.z, //right
                                          b.extents.x*b.extents.z, //up
                                          b.extents.x*b.extents.y);//forward
            Area = (BoundsSideAreas.x+BoundsSideAreas.y+BoundsSideAreas.z)*2;
            update_physical_bottom();
        }

        void update_colliders_cache()
        {
            cached_colliders.Clear();
            var parts = vessel.Parts;
            for(int i = 0, partsCount = parts.Count; i < partsCount; i++)
            {
                var p = parts[i];
                if(p == null)
                    continue;
                var colliders = p.GetComponentsInChildren<Collider>();
                for(int j = 0, collidersCount = colliders.Length; j < collidersCount; j++)
                {
                    var c = colliders[j];
                    if(c != null && !c.isTrigger)
                        cached_colliders.Add(c);
                }
            }
        }

        void update_bounds_cache()
        {
            if(refT == null || vessel == null || !vessel.loaded)
                return;
            //update physical bounds
            var b = vessel.Bounds(refT);
            cached_physical_bounds = b;
            cached_refT = refT;
            update_colliders_cache();
            //update exhaust bounds
            if(VSL.Engines?.All == null)
            {
                E = b.extents.magnitude;
                B = b;
                return;
            }
            foreach(var e in VSL.Engines.All)
            {
                if(e == null || e.engine == null || !e.Valid(VSL) || !e.engine.exhaustDamage)
                    continue;
                for(int k = 0, tCount = e.engine.thrustTransforms.Count; k < tCount; k++)
                {
                    var t = e.engine.thrustTransforms[k];
                    if(t == null)
                        continue;
                    var term = refT.InverseTransformPoint(t.position + t.forward * e.engine.exhaustDamageMaxRange*GLB.ExhaustSafeDist);
                    b.Encapsulate(term);
                }
            }
            E = b.extents.magnitude;
            B = b;
        }

        public override void Update()
        {
            if(refT == null || vessel == null || !vessel.loaded)
                return;
            var parts_count = vessel.Parts.Count;
            var stage = vessel.currentStage;
            var engines_count = VSL.Engines.All.Count;
            var now = Planetarium.GetUniversalTime();
            if(bounds_dirty
               || cached_refT != refT
               || parts_count != cached_parts_count
               || stage != cached_stage
               || engines_count != cached_engines_count
               || now >= next_bounds_update)
            {
                bounds_dirty = false;
                cached_parts_count = parts_count;
                cached_stage = stage;
                cached_engines_count = engines_count;
                next_bounds_update = now + BoundsUpdatePeriod;
                update_bounds_cache();
            }
            // keep world-space properties frame-accurate while bounds recomputation is throttled
            update_physical_props(cached_physical_bounds);
        }

        Timer brakes_measured_timer = new Timer();
        IEnumerator<YieldInstruction> measure_area_with_brakes_and_run(Callback action)
        {
            var brakes = VSL.vessel.ActionGroups[KSPActionGroup.Brakes];
            VSL.BrakesOn();
            brakes_measured_timer.Reset();
            AreaWithBrakes = BoundsSideAreas.MinComponentF();
            while(!brakes_measured_timer.TimePassed)
            {
                TCAGui.Status(0.1, "Testing aero-brakes...");
                var min_area = BoundsSideAreas.MinComponentF();
                if(min_area > AreaWithBrakes*1.001)
                {
                    AreaWithBrakes = min_area;
                    brakes_measured_timer.Reset();
                }
                yield return null;
            }
            if(!brakes)
                VSL.BrakesOn(false);
            action();
        }

        public void MeasureAreaWithBrakesAndRun(Callback action)
        {
            if(CFG.AutoBrakes)
            {
                VSL.TCA.StartCoroutine(measure_area_with_brakes_and_run(action));
                return;
            }
            Utils.Message("TCA is not allowed to use brakes. Check Advanced Tab.");
            action();
        }

        public void ResetAreaWithBrakes() { AreaWithBrakes = 0; }

        public float AreaInDirection(Vector3 wdir)
        {
            wdir.Normalize();
            return Vector3.Dot(BoundsSideAreas, new Vector3(
                Mathf.Abs(Vector3.Dot(wdir, VSL.refT.right)), 
                Mathf.Abs(Vector3.Dot(wdir, VSL.refT.up)),
                Mathf.Abs(Vector3.Dot(wdir, VSL.refT.forward))));
        }

        public float MinArea
        { get { return BoundsSideAreas.MinComponentF(); } }

        public float MaxArea
        { get { return BoundsSideAreas.MaxComponentF(); } }

        public Vector3 MinAreaDirection
        {
            get
            {
                var minI = BoundsSideAreas.MinI();
                switch(minI)
                {
                case 0:
                    return VSL.refT.right;
                case 1:
                    return VSL.refT.up;
                case 2:
                    return -VSL.refT.forward;
                default:
                    return VSL.refT.up;
                }
            }
        }

        public Vector3 MaxAreaDirection
        {
            get
            {
                var maxI = BoundsSideAreas.MaxI();
                switch(maxI)
                {
                case 0:
                    return VSL.refT.right;
                case 1:
                    return VSL.refT.up;
                case 2:
                    return VSL.refT.forward;
                default:
                    return VSL.refT.up;
                }
            }
        }

        public Vector3 MaxDragDirection
        {
            get
            {
                float[] drag = new float[6];
                for(int i = 0, parts = VSL.vessel.Parts.Count; i < parts; i++)
                {
                    var p = VSL.vessel.Parts[i];
                    for(int f = 0; f < 6; f++)
                        drag[f] += p.DragCubes.WeightedDrag[f];
                }
                int maxI = 0;
                float max = 0;
                for(int f = 0; f < 6; f++)
                {
                    if(drag[f] > max)
                    {
                        max = drag[f];
                        maxI = f;
                    }
                }
                switch((DragCube.DragFace)maxI)
                {
                case DragCube.DragFace.XP:
                    return VSL.refT.right;
                case DragCube.DragFace.XN:
                    return -VSL.refT.right;
                case DragCube.DragFace.YP:
                    return VSL.refT.up;
                case DragCube.DragFace.YN:
                    return -VSL.refT.up;
                case DragCube.DragFace.ZP:
                    return VSL.refT.forward;
                }
                return -VSL.refT.forward;
            }
        }

        public double MinDistance
        {
            get
            {
                var shift = RelC.magnitude;
                if(CFG.Target == null) return E+shift;
                var tgtVessel = CFG.Target.GetVessel();
                if(tgtVessel == null) return E+shift;
                return E+shift+tgtVessel.Radius(true);
            }
        }
    }
}

