//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//
using System;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    [CareerPart]
    [RequireModules(typeof(AttitudeControl),
                    typeof(BearingControl),
                    typeof(ThrottleControl),
                    typeof(ManeuverAutopilot))]
    public class ToOrbitAutopilot : TrajectoryCalculator
    {
        public new class Config : ComponentConfig<Config>
        {
            [Persistent] public float Dtol = 100f;
            [Persistent] public float LaunchSlope = 50f;
            [Persistent] public float TargetedAscentInclinationTolerance = 0.1f;
        }
        public static new Config C => Config.INST;

        public enum Stage { None, Start, Liftoff, GravityTurn, ChangeApA, Circularize }
        public enum LaunchGuidanceMode { Normal, Advanced }

        [Persistent] public TargetedToOrbitExecutor ToOrbit = new TargetedToOrbitExecutor();
        [Persistent] public AdvancedToOrbitExecutor AdvancedToOrbit = new AdvancedToOrbitExecutor();
        [Persistent] public TargetOrbitInfo TargetOrbit = new TargetOrbitInfo();
        [Persistent] public Stage stage;
        [Persistent] public LaunchGuidanceMode GuidanceMode;

        public bool ShowOptions;

        double ApR => TargetOrbit.ApA * 1000 + Body.Radius;
        TargetedToOrbitExecutor ActiveToOrbit =>
            GuidanceMode == LaunchGuidanceMode.Advanced ? AdvancedToOrbit : ToOrbit;

        public ToOrbitAutopilot(ModuleTCA tca) : base(tca) { }

        public override void Init()
        {
            base.Init();
            CFG.AP2.AddHandler(this, Autopilot2.ToOrbit);
            ToOrbit.AttachTCA(TCA);
            AdvancedToOrbit.AttachTCA(TCA);
        }

        protected override void UpdateState()
        {
            base.UpdateState();
            IsActive &= CFG.AP2[Autopilot2.ToOrbit] && stage != Stage.None;
        }

        public void ToOrbitCallback(Multiplexer.Command cmd)
        {
            switch(cmd)
            {
            case Multiplexer.Command.Resume:
                if(!check_patched_conics()) return;
                showOptions(true);
                configure_ascent_mode();
                break;

            case Multiplexer.Command.On:
                Reset();
                if(!check_patched_conics()) return;
                Vector3d hVdir;
                if(TargetOrbit.Inclination.Range > 1e-5f)
                {
                    var angle = Utils.Clamp((TargetOrbit.Inclination.Value - TargetOrbit.Inclination.Min) / TargetOrbit.Inclination.Range * 180, 0, 180);
                    if(TargetOrbit.DescendingNode) angle = -angle;
                    hVdir = QuaternionD.AngleAxis(angle, VesselOrbit.pos) * Vector3d.Cross(VesselOrbit.pos, Body.zUpAngularVelocity).normalized;
                }
                else hVdir = Vector3d.Cross(VesselOrbit.pos, Body.orbit.vel).normalized;
                if(TargetOrbit.RetrogradeOrbit) hVdir *= -1;
                var ApR0 = Utils.ClampH(ApR, ToOrbit.MaxApR);
                var ascO = AscendingOrbit(ApR0, hVdir, C.LaunchSlope);
                ToOrbit.Target = ascO.getRelativePositionAtUT(VSL.Physics.UT + ascO.timeToAp);
                AdvancedToOrbit.Target = ToOrbit.Target;
                configure_ascent_mode();
                stage = Stage.Start;
                goto case Multiplexer.Command.Resume;

            case Multiplexer.Command.Off:
                showOptions(false);
                Reset();
                break;
            }
        }

        void update_limits()
        {
            ToOrbit.UpdateLimits();
            AdvancedToOrbit.UpdateLimits();
            TargetOrbit.ApA.Min = ToOrbit.FirstApA.Min;
            TargetOrbit.ApA.Max = ToOrbit.FirstApA.Max;
            TargetOrbit.ApA.ClampValue();
            update_inclination_limits();
        }

        void update_inclination_limits()
        {
            //pos x [fwd x pos] = fwd(pos*pos) - pos(fwd*pos)
            var h = Vector3d.forward * VesselOrbit.pos.sqrMagnitude - VesselOrbit.pos * VesselOrbit.pos.z;
            TargetOrbit.Inclination.Min = (float)Math.Acos(h.z / h.magnitude) * Mathf.Rad2Deg;
            TargetOrbit.Inclination.Max = 180 - TargetOrbit.Inclination.Min;
            TargetOrbit.Inclination.ClampValue();
        }

        void configure_ascent_mode()
        {
            var local_native_inclination = TargetOrbit.RetrogradeOrbit
                ? 180 - TargetOrbit.Inclination.Min
                : TargetOrbit.Inclination.Min;
            ToOrbit.InPlane = Math.Abs(TargetOrbit.TargetInclination - local_native_inclination)
                              < C.TargetedAscentInclinationTolerance;
            ToOrbit.CorrectOnlyAltitude = ToOrbit.InPlane;
            AdvancedToOrbit.InPlane = ToOrbit.InPlane;
            AdvancedToOrbit.CorrectOnlyAltitude = ToOrbit.InPlane;
            AdvancedToOrbit.Target = ToOrbit.Target;
        }

        protected override void Reset()
        {
            base.Reset();
            update_limits();
            ToOrbit.Reset();
            AdvancedToOrbit.Reset();
            stage = Stage.None;
        }

        double inclination_error(double inclination)
        {
            var error = TargetOrbit.RetrogradeOrbit ?
                                   TargetOrbit.TargetInclination - inclination :
                                   inclination - TargetOrbit.TargetInclination;
            return TargetOrbit.DescendingNode ? -error : error;
        }

        Vector3d correct_dV(Vector3d dV, double UT)
        {
            var v = VesselOrbit.getOrbitalVelocityAtUT(UT);
            var nV = dV + v;
            return QuaternionD.AngleAxis(-inclination_error(VesselOrbit.inclination),
                                         VesselOrbit.getRelativePositionAtUT(UT)) * nV - v;
        }

        void change_ApR(double UT)
        {
            var dV = correct_dV(dV4Ap(VesselOrbit, ApR, UT), UT);
            ManeuverAutopilot.AddNode(VSL, dV, UT);
            CFG.AP1.On(Autopilot1.Maneuver);
            stage = Stage.ChangeApA;
        }

        void circularize(double UT)
        {
            var dV = correct_dV(dV4C(VesselOrbit, hV(UT), UT), UT);
            ManeuverAutopilot.AddNode(VSL, dV, UT);
            CFG.AP1.On(Autopilot1.Maneuver);
            stage = Stage.Circularize;
        }

        protected override void Update()
        {
            switch(stage)
            {
            case Stage.Start:
                if(VSL.LandedOrSplashed || VSL.VerticalSpeed.Absolute < 5)
                    stage = Stage.Liftoff;
                else
                {
                    ActiveToOrbit.StartGravityTurn();
                    stage = Stage.GravityTurn;
                }
                break;
            case Stage.Liftoff:
                if(ActiveToOrbit.Liftoff()) break;
                stage = Stage.GravityTurn;
                break;
            case Stage.GravityTurn:
                update_inclination_limits();
                correctTarget(ActiveToOrbit);
                if(gravity_turn())
                    break;
                CFG.BR.OffIfOn(BearingMode.Auto);
                var ApAUT = VSL.Physics.UT + VesselOrbit.timeToAp;
                if(ApR > ToOrbit.MaxApR) change_ApR(ApAUT);
                else circularize(ApAUT);
                break;
            case Stage.ChangeApA:
                TmpStatus(Loc.T("ToOrbit_AchievingApoapsis", "Achieving target apoapsis..."));
                if(CFG.AP1[Autopilot1.Maneuver]) break;
                circularize(VSL.Physics.UT + VesselOrbit.timeToAp);
                stage = Stage.Circularize;
                break;
            case Stage.Circularize:
                TmpStatus(Loc.T("ToOrbit_Circularization", "Circularization..."));
                if(CFG.AP1[Autopilot1.Maneuver]) break;
                Disable();
                ClearStatus();
                break;
            }
        }

        bool gravity_turn()
        {
            if(GuidanceMode == LaunchGuidanceMode.Advanced)
                return AdvancedToOrbit.AdvancedGravityTurn(C.Dtol, TargetOrbit.TargetInclination);
            return ToOrbit.InPlane
                       ? ToOrbit.GravityTurn(C.Dtol)
                       : ToOrbit.TargetedGravityTurn(C.Dtol);
        }

        private void correctTarget(TargetedToOrbitExecutor executor)
        {
            var orbitNormal = VesselOrbit.GetOrbitNormal();
            var vslToTargetNormal = Vector3d.Cross(VesselOrbit.pos, executor.Target);
            var norm2norm = Math.Abs(Utils.Angle2(orbitNormal, vslToTargetNormal) - 90);
            if(!(norm2norm > 60))
                return;
            // rotate target vector with current vessel orbital position
            var arcToTarget = Utils.Angle2(VesselOrbit.pos, executor.Target);
            if(arcToTarget < 30)
                executor.Target = QuaternionD.AngleAxis(30 - arcToTarget, vslToTargetNormal) * executor.Target;
            // correct inclination of the vessel-to-target plane using binary search
            var inclination = Math.Acos(Utils.Clamp(vslToTargetNormal.z / vslToTargetNormal.magnitude, -1, 1))
                              * Mathf.Rad2Deg;
            var inclinationError = inclination_error(inclination);
            var axis = VesselOrbit.pos;
            var vslToTargetTangent = Vector3d.Cross(vslToTargetNormal, VesselOrbit.pos);
            var inclinationErrorAbs = Math.Abs(inclinationError);
            var angle = vslToTargetTangent.z > 0 ? inclinationError : -inclinationError;
            var correctionAngle = double.NaN;
            while(inclinationErrorAbs > 1e-5 && Math.Abs(angle) > 1e-7)
            {
                var correctedNormal = QuaternionD.AngleAxis(angle, axis) * vslToTargetNormal;
                var error = Math.Abs(inclination_error(inclinationFromNormal(correctedNormal)));
                if(error < inclinationErrorAbs)
                {
                    correctionAngle = angle;
                    inclinationErrorAbs = error;
                }
                else
                    angle /= -2;
            }
            if(!double.IsNaN(correctionAngle))
                executor.Target = QuaternionD.AngleAxis(correctionAngle, axis) * executor.Target;
#if DEBUG
            DebugWindowController.PostMessage($"ToOrbit AP: {VSL.vessel.vesselName}",
                $"inclination: {inclination:F6}\n"
                + $"error: {inclinationError:e3}\n"
                + $"corrected: {getTargetInclinationError(executor.Target):e3}\n"
                + $"angle: {correctionAngle:F6}"); //debug
#endif
        }

        private static double inclinationFromNormal(Vector3d normal) =>
            Math.Acos(Utils.Clamp(normal.z / normal.magnitude, -1, 1)) * Mathf.Rad2Deg;

#if DEBUG
        private double getTargetInclinationError(Vector3d target) =>
            inclination_error(inclinationFromNormal(Vector3d.Cross(VesselOrbit.pos, target)));
#endif

        private void showOptions(bool show)
        {
            ShowOptions = show;
            if(ShowOptions)
                update_limits();
        }

        private void toggleOptions() => showOptions(!ShowOptions);

        public override void Draw()
        {
#if DEBUG
            if(ToOrbit != null)
            {
                Utils.GLVec(Body.position, ToOrbit.Target.xzy, Color.green);
                Utils.GLVec(Body.position, VesselOrbit.getRelativePositionAtUT(VSL.Physics.UT + VesselOrbit.timeToAp).xzy, Color.magenta);
                Utils.GLVec(Body.position, VesselOrbit.GetOrbitNormal().normalized.xzy * Body.Radius * 1.1, Color.cyan);
                Utils.GLVec(Body.position, Vector3d.Cross(VesselOrbit.pos, ToOrbit.Target).normalized.xzy * Body.Radius * 1.1, Color.red);
            }
#endif
            if(stage == Stage.None)
            {
                if(Utils.ButtonSwitch(Loc.T("ToOrbit_Button", "ToOrbit"), ShowOptions,
                                         Loc.T("ToOrbit_Button_Tooltip", "Achieve a circular orbit with desired radius and inclination"),
                                      GUILayout.ExpandWidth(true)))
                    toggleOptions();
            }
            else if(GUILayout.Button(Loc.Content("ToOrbit_Button", "ToOrbit", "ToOrbit_Abort_Tooltip", "Change target orbit or abort"),
                                     Styles.danger_button, GUILayout.ExpandWidth(true)))
                toggleOptions();
        }

        public void DrawOptions()
        {
            GUILayout.BeginVertical();
            TargetOrbit.Draw();
            draw_guidance_mode();
            ActiveToOrbit.DrawOptions();
            if(stage == Stage.GravityTurn)
            {
                ActiveToOrbit.DrawInfo(TargetOrbit.TargetInclination);
                if(GuidanceMode == LaunchGuidanceMode.Advanced)
                    AdvancedToOrbit.DrawAdvancedInfo();
            }
            GUILayout.BeginHorizontal();
            ShowOptions = !GUILayout.Button(Loc.T("Cancel", "Cancel"), Styles.active_button, GUILayout.ExpandWidth(true));
            if(stage != Stage.None &&
               GUILayout.Button(Loc.T("Abort", "Abort"), Styles.danger_button, GUILayout.ExpandWidth(true)))
            {
                ShowOptions = false;
                CFG.AP2.XOff();
            }
            if(GUILayout.Button(stage == Stage.None ? Loc.T("Launch", "Launch") : Loc.T("Change", "Change"),
                                Styles.confirm_button, GUILayout.ExpandWidth(true)))
            {
                TargetOrbit.UpdateValues();
                CFG.AP2.XOn(Autopilot2.ToOrbit);
            }
            GUILayout.EndHorizontal();
            GUILayout.EndVertical();
        }

        void draw_guidance_mode()
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(Loc.Content("ToOrbit_GuidanceMode", "Guidance:",
                "ToOrbit_GuidanceMode_Tooltip",
                "Normal uses the classic TCA gravity turn. Advanced uses predictive trajectory guidance with terminal orbit injection."),
                GUILayout.ExpandWidth(true));
            if(Utils.ButtonSwitch(Loc.T("ToOrbit_ModeNormal", "Normal"),
                   GuidanceMode == LaunchGuidanceMode.Normal,
                   Loc.T("ToOrbit_ModeNormal_Tooltip", "Use the classic TCA gravity-turn ascent."),
                   GUILayout.ExpandWidth(false)))
                GuidanceMode = LaunchGuidanceMode.Normal;
            if(Utils.ButtonSwitch(Loc.T("ToOrbit_ModeAdvanced", "Advanced"),
                   GuidanceMode == LaunchGuidanceMode.Advanced,
                   Loc.T("ToOrbit_ModeAdvanced_Tooltip", "Use predictive trajectory guidance and closed-loop orbital injection."),
                   GUILayout.ExpandWidth(false)))
                GuidanceMode = LaunchGuidanceMode.Advanced;
            GUILayout.EndHorizontal();
        }
    }

    public class TargetOrbitInfo : ConfigNodeObject
    {
        [Persistent] public FloatField ApA = new FloatField();
        [Persistent] public FloatField Inclination = new FloatField(format: "F3", min: 0, max: 180);
        [Persistent] public bool DescendingNode;
        [Persistent] public bool RetrogradeOrbit;

        public double TargetInclination => RetrogradeOrbit ? 180 - Inclination.Value : Inclination.Value;

        public void UpdateValues()
        {
            ApA.UpdateValue();
            Inclination.UpdateValue();
        }

        public void Draw()
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(Loc.Content("ToOrbit_Apoapsis", "Apoapsis:",
                    "ToOrbit_Apoapsis_Tooltip", "Apoapsis of the target circular orbit"),
                GUILayout.Width(150));
            GUILayout.Space(64);
            ApA.Draw("km", 5, "F1", suffix_width: 25);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label(Loc.Content("ToOrbit_Inclination", "Inclination:",
                    "ToOrbit_Inclination_Tooltip",
                    "Inclination of the prograde varian of a target orbit. In case of retrograde orbits the actual target inclination is 180-prograde_inclination."),
                GUILayout.Width(150));
            if(GUILayout.Button(Loc.Content(DescendingNode ? "ToOrbit_DN" : "ToOrbit_AN", DescendingNode ? "DN" : "AN", "ToOrbit_Node_Tooltip", "Launch from Ascending or Descending Node?"),
                                DescendingNode ? Styles.danger_button : Styles.enabled_button,
                                GUILayout.Width(30)))
                DescendingNode = !DescendingNode;
            if(GUILayout.Button(Loc.Content(RetrogradeOrbit ? "ToOrbit_RG" : "ToOrbit_PG", RetrogradeOrbit ? "RG" : "PG", "ToOrbit_OrbitDir_Tooltip", "Prograde or retrograde orbit?"),
                                RetrogradeOrbit ? Styles.danger_button : Styles.enabled_button,
                                GUILayout.Width(30)))
                RetrogradeOrbit = !RetrogradeOrbit;
            Inclination.Draw("°", 5, "F1", suffix_width: 25);
            GUILayout.EndHorizontal();
        }
    }
}
