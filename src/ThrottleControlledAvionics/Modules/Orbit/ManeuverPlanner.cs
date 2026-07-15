//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License.
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Linq;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    public enum ManeuverPlannerMode
    {
        Create,
        Append,
        ReplaceLast
    }

    [CareerPart(typeof(ManeuverAutopilot))]
    [RequireModules(typeof(ManeuverAutopilot))]
    public class ManeuverPlanner : TCAModule
    {
        public class Config : ComponentConfig<Config>
        {
            [Persistent] public int MaxDisplayedPlans = 20;
            [Persistent] public double MinPeSafetyMargin = 1000;
            [Persistent] public double MaxSearchTime = 0.2;
            [Persistent] public int CoarseSearchSamples = 12;
            [Persistent] public int RefineSearchSamples = 20;
            [Persistent] public double LowTWRBurnPeriodFraction = 0.08;
            [Persistent] public bool AutoExecuteCreatedNode;
        }

        public static Config C => Config.INST;

        [Persistent] public int SelectedOperation;
        [Persistent] public ManeuverPlannerMode Mode = ManeuverPlannerMode.Create;

        ManeuverOperation[] operations;
        ManeuverPlan previewPlan;
        Vector2 planScroll;

        public bool ShowOptions { get; private set; }

        public ManeuverPlanner(ModuleTCA tca) : base(tca) {}

        public override void Init()
        {
            base.Init();
            operations = new ManeuverOperation[]
            {
                new CircularizeOperation(),
                new ChangeApoapsisOperation(),
                new ChangePeriapsisOperation(),
                new EllipticizeOperation(),
                new InclinationOperation(),
                new MatchPlaneOperation(),
                new ResonantOrbitOperation(),
                new SlingshotSetupOperation(),
                new MoonTransferOperation(),
                new MoonReturnOperation(),
                new InterplanetaryTransferOperation()
            };
        }

        public override void Disable()
        {
            ShowOptions = false;
        }

        protected override void UpdateState()
        {
            base.UpdateState();
            ControlsActive &= TCAScenario.HavePatchedConics && VSL != null && VSL.vessel.patchedConicSolver != null;
        }

        public override void Draw()
        {
            var content = Loc.Content("ManeuverPlanner_Button", "Planner",
                "ManeuverPlanner_Button_Tooltip", "Create standard maneuver nodes.");
            if(ControlsActive)
            {
                if(GUILayout.Button(content, ShowOptions ? Styles.enabled_button : Styles.active_button, GUILayout.ExpandWidth(true)))
                    ShowOptions = !ShowOptions;
            }
            else
                GUILayout.Label(content, Styles.inactive_button, GUILayout.ExpandWidth(true));
        }

        public void DrawOptions()
        {
            if(operations == null || operations.Length == 0)
                Init();
            SelectedOperation = Utils.Clamp(SelectedOperation, 0, operations.Length - 1);
            var operation = operations[SelectedOperation];
            var context = ContextForCurrentMode();

            GUILayout.BeginVertical(Styles.white);
            DrawOperationSelector();
            DrawModeSelector();
            if(operation.NeedsTarget && !HasPlannerTarget(context))
                GUILayout.Label(Loc.T("ManeuverPlanner_SelectTarget", "Select a target first."), Styles.warning, GUILayout.ExpandWidth(true));
            operation.DrawParameters(context);
            DrawActions(operation, context);
            DrawPreview(context);
            GUILayout.EndVertical();
        }

        void DrawOperationSelector()
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(Loc.T("ManeuverPlanner_Operation", "Operation:"), GUILayout.ExpandWidth(false));
            var operation = operations[SelectedOperation];
            var choice = Utils.LeftRightChooser(operation.Title, operation.Description);
            if(choice > 0)
                SelectedOperation = (SelectedOperation + 1)%operations.Length;
            else if(choice < 0)
                SelectedOperation = SelectedOperation > 0 ? SelectedOperation - 1 : operations.Length - 1;
            GUILayout.EndHorizontal();
        }

        void DrawModeSelector()
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(Loc.T("ManeuverPlanner_Mode", "Mode:"), GUILayout.ExpandWidth(false));
            var choice = Utils.LeftRightChooser(ModeName(Mode), ModeDescription(Mode));
            if(choice > 0)
                Mode = (ManeuverPlannerMode)(((int)Mode + 1)%Enum.GetValues(typeof(ManeuverPlannerMode)).Length);
            else if(choice < 0)
            {
                var next = (int)Mode - 1;
                if(next < 0) next = Enum.GetValues(typeof(ManeuverPlannerMode)).Length - 1;
                Mode = (ManeuverPlannerMode)next;
            }
            GUILayout.EndHorizontal();
        }

        void DrawActions(ManeuverOperation operation, ManeuverPlannerContext context)
        {
            GUILayout.BeginHorizontal();
            if(GUILayout.Button(Loc.T("ManeuverPlanner_Preview", "Preview"), Styles.active_button, GUILayout.ExpandWidth(true)))
                previewPlan = Build(operation, context);
            if(GUILayout.Button(Loc.T("ManeuverPlanner_Create", "Create"), Styles.confirm_button, GUILayout.ExpandWidth(true)))
            {
                previewPlan = Build(operation, context);
                Apply(previewPlan, false);
            }
            if(GUILayout.Button(Loc.T("ManeuverPlanner_CreateExecute", "Create + Execute"), Styles.enabled_button, GUILayout.ExpandWidth(true)))
            {
                previewPlan = Build(operation, context);
                Apply(previewPlan, true);
            }
            if(GUILayout.Button(Loc.T("ManeuverPlanner_Clear", "Clear"), Styles.danger_button, GUILayout.ExpandWidth(false)))
            {
                Utils.ClearManeuverNodes(VSL.vessel);
                previewPlan = null;
            }
            GUILayout.EndHorizontal();
        }

        ManeuverPlan Build(ManeuverOperation operation, ManeuverPlannerContext context)
        {
            var plan = operation.BuildPlan(context);
            if(plan.Valid && plan.Nodes.Any(n => n.UT < VSL.Physics.UT))
                plan.Error(Loc.T("ManeuverPlanner_ErrorPastNode", "The plan contains a maneuver node in the past."));
            return plan;
        }

        void DrawPreview(ManeuverPlannerContext context)
        {
            if(previewPlan == null)
                return;
            GUILayout.Label(Loc.F("ManeuverPlanner_TotalDV", "Total Δv: <<1>>",
                Utils.formatBigValue((float)previewPlan.TotalDeltaV, "m/s")), Styles.label, GUILayout.ExpandWidth(true));
            planScroll = GUILayout.BeginScrollView(planScroll, GUILayout.Height(140));
            GUILayout.BeginVertical();
            foreach(var error in previewPlan.Errors)
                GUILayout.Label(error, Styles.danger, GUILayout.ExpandWidth(true));
            foreach(var warning in previewPlan.Warnings)
                GUILayout.Label(warning, Styles.warning, GUILayout.ExpandWidth(true));
            var count = Math.Min(previewPlan.Nodes.Count, Math.Max(C.MaxDisplayedPlans, 1));
            for(var i = 0; i < count; i++)
            {
                var node = previewPlan.Nodes[i];
                GUILayout.Label(Loc.F("ManeuverPlanner_NodeLine",
                        "<<1>>: <<2>>, T-<<3>>, Δv <<4>>",
                        (i + 1).ToString(),
                        node.Name,
                        FormatDuration(node.UT - context.UT),
                        Utils.formatBigValue((float)node.DeltaV, "m/s")),
                    Styles.white,
                    GUILayout.ExpandWidth(true));
            }
            if(previewPlan.Nodes.Count > count)
                GUILayout.Label(Loc.F("ManeuverPlanner_MoreNodes", "...and <<1>> more nodes", previewPlan.Nodes.Count - count),
                    Styles.inactive,
                    GUILayout.ExpandWidth(true));
            Utils.EnsureLayoutControl();
            GUILayout.EndVertical();
            GUILayout.EndScrollView();
        }

        void Apply(ManeuverPlan plan, bool execute)
        {
            if(plan == null || !plan.Valid)
                return;
            if(Mode == ManeuverPlannerMode.Create)
                Utils.ClearManeuverNodes(VSL.vessel);
            else if(Mode == ManeuverPlannerMode.ReplaceLast)
                RemoveLastNode();
            foreach(var node in plan.Nodes.OrderBy(n => n.UT))
                ManeuverAutopilot.AddNode(VSL, node.OrbitalDeltaV, node.UT);
            if(execute || C.AutoExecuteCreatedNode)
                CFG.AP1.XOn(Autopilot1.Maneuver);
        }

        void RemoveLastNode()
        {
            var nodes = VSL.vessel.patchedConicSolver.maneuverNodes;
            if(nodes.Count > 0)
                nodes[nodes.Count - 1].RemoveSelf();
            VSL.vessel.patchedConicSolver.UpdateFlightPlan();
        }

        static bool HasPlannerTarget(ManeuverPlannerContext context) =>
            context.Target != null && context.Target.GetOrbit() != null;

        ManeuverPlannerContext ContextForCurrentMode()
        {
            var orbit = VSL.orbit;
            var ut = VSL.Physics.UT;
            var nodes = VSL.vessel.patchedConicSolver.maneuverNodes;
            if((Mode == ManeuverPlannerMode.Append || Mode == ManeuverPlannerMode.ReplaceLast) && nodes.Count > 0)
            {
                var index = Mode == ManeuverPlannerMode.ReplaceLast ? nodes.Count - 2 : nodes.Count - 1;
                if(index >= 0 && nodes[index].nextPatch != null)
                {
                    orbit = nodes[index].nextPatch;
                    ut = Math.Max(ut, nodes[index].UT);
                }
            }
            return new ManeuverPlannerContext
            {
                VSL = VSL,
                Orbit = orbit,
                UT = ut,
                Target = VSL.ResolveTarget(),
                MinPeR = orbit.MinPeR() + C.MinPeSafetyMargin,
                Config = C
            };
        }

        static string ModeName(ManeuverPlannerMode mode)
        {
            switch(mode)
            {
            case ManeuverPlannerMode.Append:
                return Loc.T("ManeuverPlanner_ModeAppend", "Append");
            case ManeuverPlannerMode.ReplaceLast:
                return Loc.T("ManeuverPlanner_ModeReplaceLast", "Replace Last");
            default:
                return Loc.T("ManeuverPlanner_ModeCreate", "Create");
            }
        }

        static string ModeDescription(ManeuverPlannerMode mode)
        {
            switch(mode)
            {
            case ManeuverPlannerMode.Append:
                return Loc.T("ManeuverPlanner_ModeAppend_Tip", "Append new nodes after the existing maneuver chain.");
            case ManeuverPlannerMode.ReplaceLast:
                return Loc.T("ManeuverPlanner_ModeReplaceLast_Tip", "Replace the last existing maneuver node.");
            default:
                return Loc.T("ManeuverPlanner_ModeCreate_Tip", "Clear existing maneuver nodes and create a new plan.");
            }
        }

        static string FormatDuration(double seconds)
        {
            if(seconds < 0)
                seconds = 0;
            if(seconds < 3600)
                return seconds.ToString("F0") + "s";
            if(seconds < 86400)
                return (seconds/3600).ToString("F1") + "h";
            return (seconds/86400).ToString("F1") + "d";
        }
    }
}
