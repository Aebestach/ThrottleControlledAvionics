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
    public abstract class TimedManeuverOperation : ManeuverOperation
    {
        protected readonly PlannerTimeSelector Time = new PlannerTimeSelector();

        public override void DrawParameters(ManeuverPlannerContext context) =>
            Time.Draw(IncludeNodeTimes, IncludeAltitudeTime);

        protected virtual bool IncludeNodeTimes => false;
        protected virtual bool IncludeAltitudeTime => true;
        protected double UT(ManeuverPlannerContext context) => Math.Max(context.UT, Time.ComputeUT(context));
    }

    public class CircularizeOperation : TimedManeuverOperation
    {
        public override string Title => Loc.T("ManeuverPlanner_Circularize", "Circularize");
        public override string Description => Loc.T("ManeuverPlanner_Circularize_Tip", "Create a node that circularizes the current orbit.");

        public override ManeuverPlan BuildPlan(ManeuverPlannerContext context)
        {
            var plan = new ManeuverPlan();
            if(context.Orbit.eccentricity >= 1)
            {
                plan.Error(Loc.T("ManeuverPlanner_ErrorHyperbolic", "This operation requires a closed orbit."));
                return plan;
            }
            var ut = UT(context);
            var dV = TrajectoryCalculator.dV4C(context.Orbit, context.Orbit.hV(ut), ut);
            plan.Add(Title, dV, ut, context.Orbit.referenceBody);
            ManeuverOperation.ValidateApsides(plan, TrajectoryCalculator.NewOrbit(context.Orbit, dV, ut), context.MinPeR);
            ManeuverOperation.ValidateDeltaV(plan, context.VSL);
            return plan;
        }
    }

    public class ChangeApoapsisOperation : TimedManeuverOperation
    {
        double targetApA = 100000;

        public ChangeApoapsisOperation()
        {
            Time.Reference = PlannerTimeReference.Periapsis;
        }

        public override string Title => Loc.T("ManeuverPlanner_ChangeAp", "Change ApA");
        public override string Description => Loc.T("ManeuverPlanner_ChangeAp_Tip", "Create a node that changes apoapsis.");

        public override void DrawParameters(ManeuverPlannerContext context)
        {
            targetApA = PlannerTimeSelector.NumericField(
                Loc.T("ManeuverPlanner_TargetApA", "Target ApA:"),
                targetApA,
                "m",
                Loc.T("ManeuverPlanner_TargetApA_Tip", "Desired apoapsis altitude above the current body."));
            base.DrawParameters(context);
        }

        public override ManeuverPlan BuildPlan(ManeuverPlannerContext context)
        {
            var plan = new ManeuverPlan();
            var ut = UT(context);
            var dV = TrajectoryCalculator.dV4Ap(context.Orbit, BodyRadius(context.Orbit.referenceBody, targetApA), ut);
            plan.Add(Title, dV, ut, context.Orbit.referenceBody);
            ManeuverOperation.ValidateApsides(plan, TrajectoryCalculator.NewOrbit(context.Orbit, dV, ut), context.MinPeR);
            ManeuverOperation.ValidateDeltaV(plan, context.VSL);
            return plan;
        }
    }

    public class ChangePeriapsisOperation : TimedManeuverOperation
    {
        double targetPeA = 80000;

        public ChangePeriapsisOperation()
        {
            Time.Reference = PlannerTimeReference.Apoapsis;
        }

        public override string Title => Loc.T("ManeuverPlanner_ChangePe", "Change PeA");
        public override string Description => Loc.T("ManeuverPlanner_ChangePe_Tip", "Create a node that changes periapsis.");

        public override void DrawParameters(ManeuverPlannerContext context)
        {
            targetPeA = PlannerTimeSelector.NumericField(
                Loc.T("ManeuverPlanner_TargetPeA", "Target PeA:"),
                targetPeA,
                "m",
                Loc.T("ManeuverPlanner_TargetPeA_Tip", "Desired periapsis altitude above the current body."));
            base.DrawParameters(context);
        }

        public override ManeuverPlan BuildPlan(ManeuverPlannerContext context)
        {
            var plan = new ManeuverPlan();
            var ut = UT(context);
            var dV = TrajectoryCalculator.dV4Pe(context.Orbit, BodyRadius(context.Orbit.referenceBody, targetPeA), ut);
            plan.Add(Title, dV, ut, context.Orbit.referenceBody);
            ManeuverOperation.ValidateApsides(plan, TrajectoryCalculator.NewOrbit(context.Orbit, dV, ut), context.MinPeR);
            ManeuverOperation.ValidateDeltaV(plan, context.VSL);
            return plan;
        }
    }

    public class EllipticizeOperation : TimedManeuverOperation
    {
        double targetPeA = 80000;
        double targetApA = 200000;

        public override string Title => Loc.T("ManeuverPlanner_Ellipticize", "Set PeA/ApA");
        public override string Description => Loc.T("ManeuverPlanner_Ellipticize_Tip", "Create a single node that targets both periapsis and apoapsis.");

        public override void DrawParameters(ManeuverPlannerContext context)
        {
            targetPeA = PlannerTimeSelector.NumericField(
                Loc.T("ManeuverPlanner_TargetPeA", "Target PeA:"),
                targetPeA,
                "m",
                Loc.T("ManeuverPlanner_TargetPeA_Tip", "Desired periapsis altitude above the current body."));
            targetApA = PlannerTimeSelector.NumericField(
                Loc.T("ManeuverPlanner_TargetApA", "Target ApA:"),
                targetApA,
                "m",
                Loc.T("ManeuverPlanner_TargetApA_Tip", "Desired apoapsis altitude above the current body."));
            base.DrawParameters(context);
        }

        public override ManeuverPlan BuildPlan(ManeuverPlannerContext context)
        {
            var plan = new ManeuverPlan();
            var ut = UT(context);
            var peR = BodyRadius(context.Orbit.referenceBody, Math.Min(targetPeA, targetApA));
            var apR = BodyRadius(context.Orbit.referenceBody, Math.Max(targetPeA, targetApA));
            var dVAp = TrajectoryCalculator.dV4Ap(context.Orbit, apR, ut);
            var dVApPe = TrajectoryCalculator.dV4Pe(context.Orbit, peR, ut, dVAp);
            var dVPe = TrajectoryCalculator.dV4Pe(context.Orbit, peR, ut);
            var dVPeAp = TrajectoryCalculator.dV4Ap(context.Orbit, apR, ut, dVPe);
            var dV = dVApPe.magnitude < dVPeAp.magnitude ? dVApPe : dVPeAp;
            plan.Add(Title, dV, ut, context.Orbit.referenceBody);
            ManeuverOperation.ValidateApsides(plan, TrajectoryCalculator.NewOrbit(context.Orbit, dV, ut), context.MinPeR);
            ManeuverOperation.ValidateDeltaV(plan, context.VSL);
            return plan;
        }
    }

    public class InclinationOperation : TimedManeuverOperation
    {
        double targetInclination;
        bool splitForLowTwr = true;

        public InclinationOperation()
        {
            Time.Reference = PlannerTimeReference.CheapestNode;
        }

        public override string Title => Loc.T("ManeuverPlanner_Inclination", "Inclination");
        public override string Description => Loc.T("ManeuverPlanner_Inclination_Tip", "Create one or more nodes that change orbital inclination.");
        protected override bool IncludeNodeTimes => true;
        protected override bool IncludeAltitudeTime => false;

        public override void DrawParameters(ManeuverPlannerContext context)
        {
            targetInclination = PlannerTimeSelector.NumericField(
                Loc.T("ManeuverPlanner_TargetInc", "Target Inclination:"),
                targetInclination,
                "deg",
                Loc.T("ManeuverPlanner_TargetInc_Tip", "Desired orbital inclination in degrees."));
            Utils.ButtonSwitch(Loc.T("ManeuverPlanner_SplitLowTWR", "Split Low TWR"),
                ref splitForLowTwr,
                Loc.T("ManeuverPlanner_SplitLowTWR_Tip", "Split long inclination burns over several nodes."),
                GUILayout.ExpandWidth(true));
            base.DrawParameters(context);
        }

        public override ManeuverPlan BuildPlan(ManeuverPlannerContext context)
        {
            var plan = new ManeuverPlan();
            var ut = UT(context);
            var dV = ManeuverPlannerMath.DeltaVToChangeInclination(context.Orbit, ut, targetInclination);
            var splits = 1;
            if(splitForLowTwr && context.Orbit.period > 0)
            {
                var burn = context.VSL.Engines.TTB_Precise((float)dV.magnitude);
                var maxBurn = context.Orbit.period*Math.Max(0.01, context.Config.LowTWRBurnPeriodFraction);
                splits = Math.Max(1, (int)Math.Ceiling(burn/maxBurn));
            }
            for(var i = 0; i < splits; i++)
            {
                var nodeUT = ut + context.Orbit.period*i;
                plan.Add(splits > 1
                        ? Loc.F("ManeuverPlanner_InclinationPart", "Inclination <<1>>/<<2>>", i + 1, splits)
                        : Title,
                    dV/splits,
                    nodeUT,
                    context.Orbit.referenceBody);
            }
            ManeuverOperation.ValidateDeltaV(plan, context.VSL);
            return plan;
        }
    }

    public class MatchPlaneOperation : TimedManeuverOperation
    {
        public MatchPlaneOperation()
        {
            Time.Reference = PlannerTimeReference.CheapestNode;
        }

        public override string Title => Loc.T("ManeuverPlanner_MatchPlane", "Match Plane");
        public override string Description => Loc.T("ManeuverPlanner_MatchPlane_Tip", "Create a node that matches the selected target orbit plane.");
        public override bool NeedsTarget => true;
        protected override bool IncludeNodeTimes => false;
        protected override bool IncludeAltitudeTime => false;

        public override ManeuverPlan BuildPlan(ManeuverPlannerContext context)
        {
            var plan = new ManeuverPlan();
            var targetOrbit = context.Target?.GetOrbit();
            if(targetOrbit == null)
            {
                plan.Error(Loc.T("ManeuverPlanner_ErrorTargetOrbit", "The selected target has no orbit."));
                return plan;
            }
            var ut = ManeuverPlannerMath.CheapestPlaneNodeUT(context.Orbit, targetOrbit, context.UT);
            var dV = ManeuverPlannerMath.DeltaVToMatchPlane(context.Orbit, targetOrbit, ut);
            plan.Add(Title, dV, ut, context.Orbit.referenceBody);
            ManeuverOperation.ValidateDeltaV(plan, context.VSL);
            return plan;
        }
    }

    public class ResonantOrbitOperation : TimedManeuverOperation
    {
        double numerator = 2;
        double denominator = 1;

        public override string Title => Loc.T("ManeuverPlanner_Resonant", "Resonant Orbit");
        public override string Description => Loc.T("ManeuverPlanner_Resonant_Tip",
            "Create a phasing orbit with the selected resonant period ratio (e.g. 2:1).");

        public override void DrawParameters(ManeuverPlannerContext context)
        {
            numerator = Math.Max(1, PlannerTimeSelector.NumericField(
                Loc.T("ManeuverPlanner_ResonanceNum", "Period Ratio · Num:"),
                numerator,
                "",
                Loc.T("ManeuverPlanner_ResonanceNum_Tip",
                    "The first number in a ratio like 2:1 (new period = current × num/den).")));
            denominator = Math.Max(1, PlannerTimeSelector.NumericField(
                Loc.T("ManeuverPlanner_ResonanceDen", "Period Ratio · Den:"),
                denominator,
                "",
                Loc.T("ManeuverPlanner_ResonanceDen_Tip",
                    "The second number in a ratio like 2:1.")));
            base.DrawParameters(context);
        }

        public override ManeuverPlan BuildPlan(ManeuverPlannerContext context)
        {
            var plan = new ManeuverPlan();
            if(context.Orbit.eccentricity >= 1)
            {
                plan.Error(Loc.T("ManeuverPlanner_ErrorHyperbolic", "This operation requires a closed orbit."));
                return plan;
            }
            var ut = UT(context);
            var dV = TrajectoryCalculator.dV4T2(context.Orbit, context.Orbit.period*numerator/denominator, ut);
            plan.Add(Title, dV, ut, context.Orbit.referenceBody);
            ManeuverOperation.ValidateApsides(plan, TrajectoryCalculator.NewOrbit(context.Orbit, dV, ut), context.MinPeR);
            ManeuverOperation.ValidateDeltaV(plan, context.VSL);
            return plan;
        }
    }

    public class MoonTransferOperation : TimedManeuverOperation
    {
        protected bool addCapture;
        protected bool flybyOnly;
        double maxStartDays = 5;
        double minTransferHours = 1;
        double maxTransferDays = 10;
        readonly LambertSolver solver = new LambertSolver();

        public override string Title => flybyOnly
            ? Loc.T("ManeuverPlanner_Slingshot", "Slingshot Setup")
            : Loc.T("ManeuverPlanner_MoonTransfer", "Moon Transfer");
        public override string Description => Loc.T("ManeuverPlanner_MoonTransfer_Tip", "Search a Lambert transfer to the selected moon or vessel.");
        public override bool NeedsTarget => true;
        public override bool LongSearch => true;

        public override void DrawParameters(ManeuverPlannerContext context)
        {
            maxStartDays = Math.Max(0, PlannerTimeSelector.NumericField(
                Loc.T("ManeuverPlanner_MaxStart", "Max Start:"),
                maxStartDays,
                "d",
                Loc.T("ManeuverPlanner_MaxStart_Tip", "Maximum time allowed before the departure burn.")));
            minTransferHours = Math.Max(0.1, PlannerTimeSelector.NumericField(
                Loc.T("ManeuverPlanner_MinTransfer", "Min Transfer:"),
                minTransferHours,
                "h",
                Loc.T("ManeuverPlanner_MinTransfer_Tip", "Minimum allowed transfer duration.")));
            maxTransferDays = Math.Max(0.1, PlannerTimeSelector.NumericField(
                Loc.T("ManeuverPlanner_MaxTransfer", "Max Transfer:"),
                maxTransferDays,
                "d",
                Loc.T("ManeuverPlanner_MaxTransfer_Tip", "Maximum allowed transfer duration.")));
            if(!flybyOnly)
                Utils.ButtonSwitch(Loc.T("ManeuverPlanner_AddCapture", "Add Capture"),
                    ref addCapture,
                    Loc.T("ManeuverPlanner_AddCapture_Tip", "Add an approximate arrival velocity-match node."),
                    GUILayout.ExpandWidth(true));
        }

        public override ManeuverPlan BuildPlan(ManeuverPlannerContext context)
        {
            var plan = new ManeuverPlan();
            var targetOrbit = context.Target?.GetOrbit();
            if(targetOrbit == null)
            {
                plan.Error(Loc.T("ManeuverPlanner_ErrorTargetOrbit", "The selected target has no orbit."));
                return plan;
            }
            if(targetOrbit.referenceBody != context.Orbit.referenceBody)
            {
                plan.Error(Loc.T("ManeuverPlanner_ErrorSameBody", "The target must orbit the same body for this operation."));
                return plan;
            }
            var candidate = ManeuverPlannerMath.BestLambertTransfer(context.Orbit,
                targetOrbit,
                context.UT,
                maxStartDays*86400,
                minTransferHours*3600,
                maxTransferDays*86400,
                context.Config.CoarseSearchSamples,
                solver);
            if(candidate == null)
            {
                plan.Error(Loc.T("ManeuverPlanner_ErrorNoTransfer", "No valid transfer was found."));
                return plan;
            }
            plan.Add(Title, candidate.DeltaV, candidate.DepartureUT, context.Orbit.referenceBody);
            if(addCapture && !flybyOnly)
            {
                var captureDV = targetOrbit.getOrbitalVelocityAtUT(candidate.ArrivalUT)
                                - candidate.TransferOrbit.getOrbitalVelocityAtUT(candidate.ArrivalUT);
                plan.Add(Loc.T("ManeuverPlanner_Capture", "Capture/Match Velocity"),
                    captureDV,
                    candidate.ArrivalUT,
                    targetOrbit.referenceBody);
            }
            ManeuverOperation.ValidateDeltaV(plan, context.VSL);
            return plan;
        }
    }

    public class SlingshotSetupOperation : MoonTransferOperation
    {
        public override string Description => Loc.T("ManeuverPlanner_Slingshot_Tip",
            "Search a flyby setup transfer to the selected moon or vessel.");

        public SlingshotSetupOperation()
        {
            flybyOnly = true;
            addCapture = false;
        }
    }

    public class MoonReturnOperation : TimedManeuverOperation
    {
        double parentPeA = 30000;
        bool targetInclinationFlag;
        double targetInclination;

        public override string Title => Loc.T("ManeuverPlanner_MoonReturn", "Moon Return");
        public override string Description => Loc.T("ManeuverPlanner_MoonReturn_Tip", "Leave a moon and target a periapsis around the parent body.");

        public override void DrawParameters(ManeuverPlannerContext context)
        {
            parentPeA = PlannerTimeSelector.NumericField(
                Loc.T("ManeuverPlanner_ParentPeA", "Parent PeA:"),
                parentPeA,
                "m",
                Loc.T("ManeuverPlanner_ParentPeA_Tip", "Target periapsis altitude around the parent body after leaving the moon."));
            Utils.ButtonSwitch(Loc.T("ManeuverPlanner_TargetIncToggle", "Target Inc."),
                ref targetInclinationFlag,
                Loc.T("ManeuverPlanner_TargetIncToggle_Tip", "Bias the ejection toward the requested parent-body inclination."),
                GUILayout.ExpandWidth(true));
            if(targetInclinationFlag)
                targetInclination = PlannerTimeSelector.NumericField(
                Loc.T("ManeuverPlanner_TargetInc", "Target Inclination:"),
                targetInclination,
                "deg",
                Loc.T("ManeuverPlanner_TargetInc_Tip", "Desired orbital inclination in degrees."));
            base.DrawParameters(context);
        }

        public override ManeuverPlan BuildPlan(ManeuverPlannerContext context)
        {
            var plan = new ManeuverPlan();
            var moon = context.Orbit.referenceBody;
            var parent = moon.referenceBody;
            if(parent == null || moon.orbit == null)
            {
                plan.Error(Loc.T("ManeuverPlanner_ErrorNoParent", "The current body does not orbit another body."));
                return plan;
            }
            var ut = UT(context);
            var parentPeR = parent.Radius + parentPeA;
            var parentDV = TrajectoryCalculator.dV4Pe(moon.orbit, parentPeR, ut);
            if(targetInclinationFlag)
                parentDV += ManeuverPlannerMath.DeltaVToChangeInclination(moon.orbit, ut, targetInclination);
            var localDV = ManeuverPlannerMath.DeltaVToEject(context.Orbit, parentDV, ut);
            plan.Add(Title, localDV, ut, moon);
            ManeuverOperation.ValidateDeltaV(plan, context.VSL);
            return plan;
        }
    }

    public class InterplanetaryTransferOperation : TimedManeuverOperation
    {
        double maxStartDays = 400;
        double minTransferDays = 20;
        double maxTransferDays = 800;
        readonly LambertSolver solver = new LambertSolver();

        public override string Title => Loc.T("ManeuverPlanner_Interplanetary", "Interplanetary");
        public override string Description => Loc.T("ManeuverPlanner_Interplanetary_Tip", "Search an approximate planet-to-planet transfer.");
        public override bool NeedsTarget => true;
        public override bool LongSearch => true;

        public override void DrawParameters(ManeuverPlannerContext context)
        {
            maxStartDays = Math.Max(1, PlannerTimeSelector.NumericField(
                Loc.T("ManeuverPlanner_MaxStart", "Max Start:"),
                maxStartDays,
                "d",
                Loc.T("ManeuverPlanner_MaxStart_Tip", "Maximum time allowed before the departure burn.")));
            minTransferDays = Math.Max(1, PlannerTimeSelector.NumericField(
                Loc.T("ManeuverPlanner_MinTransfer", "Min Transfer:"),
                minTransferDays,
                "d",
                Loc.T("ManeuverPlanner_MinTransfer_Tip", "Minimum allowed transfer duration.")));
            maxTransferDays = Math.Max(minTransferDays, PlannerTimeSelector.NumericField(
                Loc.T("ManeuverPlanner_MaxTransfer", "Max Transfer:"),
                maxTransferDays,
                "d",
                Loc.T("ManeuverPlanner_MaxTransfer_Tip", "Maximum allowed transfer duration.")));
        }

        public override ManeuverPlan BuildPlan(ManeuverPlannerContext context)
        {
            var plan = new ManeuverPlan();
            var targetOrbit = context.Target?.GetOrbit();
            var originBody = context.Orbit.referenceBody;
            if(targetOrbit == null || originBody.orbit == null)
            {
                plan.Error(Loc.T("ManeuverPlanner_ErrorInterplanetaryTarget", "Select a planet or moon target outside the current SOI."));
                return plan;
            }
            if(originBody.orbit.referenceBody != targetOrbit.referenceBody)
            {
                plan.Error(Loc.T("ManeuverPlanner_ErrorParentMismatch", "The target must orbit the same parent body as the current SOI."));
                return plan;
            }
            var candidate = ManeuverPlannerMath.BestLambertTransfer(originBody.orbit,
                targetOrbit,
                context.UT,
                maxStartDays*86400,
                minTransferDays*86400,
                maxTransferDays*86400,
                context.Config.CoarseSearchSamples,
                solver);
            if(candidate == null)
            {
                plan.Error(Loc.T("ManeuverPlanner_ErrorNoTransfer", "No valid transfer was found."));
                return plan;
            }
            var localUT = Math.Max(context.UT, candidate.DepartureUT);
            var localDV = ManeuverPlannerMath.DeltaVToEject(context.Orbit, candidate.DeltaV, localUT);
            plan.Add(Title, localDV, localUT, originBody);
            ManeuverOperation.ValidateDeltaV(plan, context.VSL);
            plan.Warn(Loc.T("ManeuverPlanner_WarnInterplanetaryApprox",
                "This is an approximate ejection node; add a mid-course correction after leaving the current SOI."));
            return plan;
        }
    }
}
