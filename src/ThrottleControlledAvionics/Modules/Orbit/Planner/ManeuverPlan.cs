//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License.
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System.Collections.Generic;
using System.Linq;

namespace ThrottleControlledAvionics
{
    public class ManeuverPlanNode
    {
        public string Name;
        public Vector3d OrbitalDeltaV;
        public double UT;
        public CelestialBody Body;

        public double DeltaV => OrbitalDeltaV.magnitude;

        public ManeuverPlanNode(string name, Vector3d orbitalDeltaV, double ut, CelestialBody body)
        {
            Name = name;
            OrbitalDeltaV = orbitalDeltaV;
            UT = ut;
            Body = body;
        }
    }

    public class ManeuverPlan
    {
        public readonly List<ManeuverPlanNode> Nodes = new List<ManeuverPlanNode>();
        public readonly List<string> Warnings = new List<string>();
        public readonly List<string> Errors = new List<string>();

        public bool Valid => Nodes.Count > 0 && Errors.Count == 0;
        public double TotalDeltaV => Nodes.Sum(n => n.DeltaV);

        public void Add(string name, Vector3d orbitalDeltaV, double ut, CelestialBody body) =>
            Nodes.Add(new ManeuverPlanNode(name, orbitalDeltaV, ut, body));

        public void Warn(string message)
        {
            if(!string.IsNullOrEmpty(message))
                Warnings.Add(message);
        }

        public void Error(string message)
        {
            if(!string.IsNullOrEmpty(message))
                Errors.Add(message);
        }
    }
}
