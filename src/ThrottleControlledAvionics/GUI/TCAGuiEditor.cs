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
using System.Collections.Generic;
using UnityEngine;
using KSP.UI.Screens;
using AT_Utils;
using AT_Utils.UI;

namespace ThrottleControlledAvionics
{
    [KSPAddon(KSPAddon.Startup.EditorAny, false)]
    public class TCAGuiEditor : AddonWindowBase<TCAGuiEditor>
    {
        public static bool Available { get; private set; }
        Dictionary<Type, bool> Modules = new Dictionary<Type, bool>();
        static Texture2D CoM_Icon;

        TCAPartsEditor PartsEditor;

        ModuleTCA TCA;
        NamedConfig CFG;
        readonly List<EngineWrapper> Engines = new List<EngineWrapper>();
        readonly List<RCSWrapper> RCS = new List<RCSWrapper>();
        readonly EnginesDB ActiveEngines = new EnginesDB();

        static bool HaveSelectedPart
        {
            get
            {
                var selectedPart = EditorLogic.SelectedPart;
                return selectedPart != null
                       && EditorLogic.SelectedPart.potentialParent != null;
            }
        }

        HighlightSwitcher TCA_highlight, Engines_highlight;
        
        private readonly FloatField MinHorizontalAccel = new FloatField(min:0);
        private readonly FloatField RangeStartAltitude = new FloatField(min: 0);
        private readonly FloatField RangeTargetDistance = new FloatField(min: 0);
        private readonly FloatField RangeMaxCruiseSpeed = new FloatField(min: 1);
        private readonly FloatField RangeHoverReserve = new FloatField(min: 0);

        float WetMass, DryMass, MinTWR, MaxTWR, MinLimit;
        Vector3 CoM = Vector3.zero;
        Vector3 WetCoM = Vector3.zero;
        Vector3 DryCoM = Vector3.zero;
        Matrix3x3f InertiaTensor = new Matrix3x3f();
        Vector3 MoI { get { return new Vector3(InertiaTensor[0, 0], InertiaTensor[1, 1], InertiaTensor[2, 2]); } }
        private float Mass => use_wet_mass ? WetMass : DryMass;

        bool show_imbalance;
        bool use_wet_mass = true;
        bool show_range_planner;
        bool range_allow_parachutes = true;
        bool range_allow_staging = true;
        bool range_allow_atmo_assist = true;
        int range_body_index = -1;
        MissionScenario range_scenario = MissionScenario.TargetRange;
        Vector2 range_scroll;
        static Rect range_planner_pos = new Rect(250, 250, 460, 360);

        public override void Awake()
        {
            base.Awake();
            width = 600;
            height = 100;
            GameEvents.onEditorShipModified.Add(OnShipModified);
            GameEvents.onEditorPartEvent.Add(OnPartEvent);
            GameEvents.onEditorLoad.Add(OnShipLoad);
            GameEvents.onEditorRestart.Add(Reset);
            GameEvents.onEditorStarted.Add(Started);
            Available = false;
            show_imbalance = false;
            use_wet_mass = true;
            RangeStartAltitude.Value = 0;
            RangeTargetDistance.Value = 10000;
            RangeMaxCruiseSpeed.Value = 100;
            RangeHoverReserve.Value = LandingTrajectoryAutopilot.C.HoverTimeThreshold;
            //icons
            CoM_Icon = TextureCache.GetTexture(Globals.RADIATION_ICON);
            //highlighters
            TCA_highlight = new HighlightSwitcher(highlight_TCA, reset_TCA_highlighting);
            Engines_highlight = new HighlightSwitcher(highlight_engines, reset_engines_highlightig);
        }

        public override void OnDestroy()
        {
            GameEvents.onEditorShipModified.Remove(OnShipModified);
            GameEvents.onEditorPartEvent.Remove(OnPartEvent);
            GameEvents.onEditorLoad.Remove(OnShipLoad);
            GameEvents.onEditorRestart.Remove(Reset);
            GameEvents.onEditorStarted.Remove(Started);
            TCAMacroEditor.Exit();
            base.OnDestroy();
        }

        static void UpdatePartsInfo()
        {
            //update TCA part infos
            var info = TCAScenario.ModuleStatusString();
            foreach(var ap in PartLoader.LoadedPartsList)
            {
                foreach(var mi in ap.moduleInfos)
                {
                    if(mi.moduleName != ModuleTCA.TCA_NAME) continue;
                    mi.primaryInfo = "<b>TCA:</b> " + info;
                    mi.info = info;
                }
            }
        }

        void Started() { UpdatePartsInfo(); }

        void Reset() { reset = true; }

        void OnShipLoad(ShipConstruct ship, CraftBrowserDialog.LoadType load_type)
        { init_engines = load_type == CraftBrowserDialog.LoadType.Normal; }

        void OnShipModified(ShipConstruct ship)
        { update_engines = true; }

        void OnPartEvent(ConstructionEventType eventType, Part part)
        {
            update_engines |= eventType == ConstructionEventType.PartAttached
                              || eventType == ConstructionEventType.PartDetached
                              || eventType == ConstructionEventType.PartRootSelected;
        }

        void update_modules()
        {
            Modules.Clear();
            TCAModulesDatabase.ValidModules
                .ForEach(t => Modules.Add(t, TCAModulesDatabase.ModuleAvailable(t, CFG)));
        }
        public static void UpdateModules() { if(Instance) Instance.update_modules(); }

        bool GetCFG()
        {
            var ship = EditorLogic.fetch.ship;
            var TCA_Modules = ModuleTCA.AllTCA(ship);
            if(TCA_Modules.Count == 0) { Reset(); return false; }
            CFG = null;
            foreach(var tca in TCA_Modules)
            {
                if(tca.CFG == null) continue;
                CFG = NamedConfig.FromVesselConfig(ship.shipName, tca.CFG);
                break;
            }
            if(CFG == null)
            {
                CFG = NamedConfig.FromVesselConfig(ship.shipName, TCAScenario.GetDefaultConfig(ship.shipFacility));
                if(CFG.EnginesProfiles.Empty)
                    CFG.EnginesProfiles.AddProfile(Engines);
            }
            else CFG.ActiveProfile.Apply(Engines);
            UpdateCFG(TCA_Modules);
            return true;
        }

        void UpdateCFG(List<ModuleTCA> TCA_Modules)
        {
            if(CFG == null || TCA_Modules.Count == 0) return;
            //this.Log("UpdateCFG start: TCA Modules: {}", TCA_Modules);//debug
            TCA_highlight.Reset();
            TCA = TCA_Modules[0];
            if(string.IsNullOrEmpty(TCA.GID))
                TCA.ChangeGID();
            TCA_Modules.ForEach(m =>
            {
                m.CFG = null;
                m.EnableTCA(false);
                m.GroupMaster = false;
                m.SetGID(TCA.GID);
            });
            TCA.GroupMaster = true;
            TCA.CFG = CFG;
            TCA.EnableTCA(true);
            CFG.ActiveProfile.Update(Engines);
            PartsEditor.SetCFG(CFG);
            MinHorizontalAccel.Value = CFG.MinHorizontalAccel;
            update_modules();
            //this.Log("UpdateCFG end: TCA Modules: {}", TCA_Modules);//debug
        }
        void UpdateCFG() { UpdateCFG(ModuleTCA.AllTCA(EditorLogic.fetch.ship)); }

        void compute_inertia_tensor()
        {
            InertiaTensor = new Matrix3x3f();
            if(EditorLogic.RootPart)
                update_inertia_tensor(EditorLogic.RootPart);
        }

        void update_inertia_tensor(Part part)
        {
            if(!EditorLogic.RootPart) return;
            var partMass = part.mass;
            if(use_wet_mass) partMass += part.GetResourceMass();
            var T = part.transform;
            Vector3 partPosition = EditorLogic.RootPart.transform
                .InverseTransformDirection(T.position + T.rotation * part.CoMOffset - CoM);
            for(int i = 0; i < 3; i++)
            {
                InertiaTensor.Add(i, i, partMass * partPosition.sqrMagnitude);
                for(int j = 0; j < 3; j++)
                    InertiaTensor.Add(i, j, -partMass * partPosition[i] * partPosition[j]);
            }
            part.children.ForEach(update_inertia_tensor);
        }

        void update_mass_and_CoM(Part part)
        {
            if(part == null) return;
            var dryMass = part.mass;
            var wetMass = dryMass + part.GetResourceMass();
            Vector3 pos = Vector3.zero;
            if(part.physicalSignificance == Part.PhysicalSignificance.FULL)
            {
                var T = part.transform;
                pos = T.position + T.rotation * part.CoMOffset;
            }
            else if(part.parent != null)
            {
                var T = part.parent.transform;
                pos = T.position + T.rotation * part.parent.CoMOffset;
            }
            else if(part.potentialParent != null)
            {
                var T = part.potentialParent.transform;
                pos = T.position + T.rotation * part.potentialParent.CoMOffset;
            }
            WetCoM += pos * wetMass;
            DryCoM += pos * dryMass;
            WetMass += wetMass;
            DryMass += dryMass;
            part.children.ForEach(update_mass_and_CoM);
        }

        void find_engines_recursively(Part part, List<EngineWrapper> engines, List<RCSWrapper> rcs)
        {
            if(part.Modules != null)
            {
                engines.AddRange(part.Modules.GetModules<ModuleEngines>()
                                     .Select(m => new EngineWrapper(m)));
                rcs.AddRange(part.Modules.GetModules<ModuleRCS>()
                                     .Select(m => new RCSWrapper(m)));
            }
            part.children.ForEach(p => find_engines_recursively(p, engines, rcs));
        }

        void UpdateEngines()
        {
            Engines_highlight.Reset();
            Engines.Clear();
            RCS.Clear();
            if(TCAScenario.HasTCA && EditorLogic.RootPart)
                find_engines_recursively(EditorLogic.RootPart, Engines, RCS);
        }

        void process_active_engine(EngineWrapper e)
        {
            e.throttle = e.VSF = e.thrustMod = 1;
            e.UpdateThrustInfo();
            e.InitLimits();
            e.InitTorque(EditorLogic.RootPart.transform,
                CoM,
                Mass,
                MoI,
                EngineOptimizer.C.TorqueRatioFactor);
            e.UpdateCurrentTorque(1);
        }

        static List<Part> GetSelectedParts()
        {
            var selected_parts = new List<Part>();
            if(HaveSelectedPart && !EditorLogic.fetch.ship.Contains(EditorLogic.SelectedPart))
            {
                selected_parts.Add(EditorLogic.SelectedPart);
                selected_parts.AddRange(EditorLogic.SelectedPart.symmetryCounterparts);
            }
            return selected_parts;
        }

        void CalculateMassAndCoM(List<Part> selected_parts)
        {
            WetMass = DryMass = MinTWR = MaxTWR = 0;
            CoM = WetCoM = DryCoM = Vector3.zero;
            update_mass_and_CoM(EditorLogic.RootPart);
            if(selected_parts != null)
                selected_parts.ForEach(update_mass_and_CoM);
            WetCoM /= WetMass; DryCoM /= DryMass;
            CoM = use_wet_mass ? WetCoM : DryCoM;
        }

        void UpdateShipStats()
        {
            MinLimit = 0;
            var thrust = Vector3.zero;
            var selected_parts = GetSelectedParts();
            CalculateMassAndCoM(selected_parts);
            if(CFG != null && CFG.Enabled && Engines.Count > 0)
            {
                ActiveEngines.Clear();
                for(int i = 0, EnginesCount = Engines.Count; i < EnginesCount; i++)
                {
                    var e = Engines[i];
                    var ecfg = CFG.ActiveProfile.GetConfig(e);
                    if(ecfg == null || ecfg.On) ActiveEngines.Add(e);
                }
                if(selected_parts.Count > 0)
                {
                    var selected_engines = new List<EngineWrapper>();
                    var selected_rcs = new List<RCSWrapper>();
                    selected_parts.ForEach(p => find_engines_recursively(p, selected_engines, selected_rcs));
                    ActiveEngines.AddRange(selected_engines);
                }
                if(ActiveEngines.Count > 0)
                {
                    ActiveEngines.ForEach(process_active_engine);
                    compute_inertia_tensor();
                    selected_parts.ForEach(update_inertia_tensor);
                    ActiveEngines.SortByRole();
                    if(!ActiveEngines.OptimizeForZeroTorque(MoI))
                    {
                        ActiveEngines.Steering.ForEach(e => e.limit = 0);
                        ActiveEngines.Balanced.ForEach(e => e.limit = 0);
                    }
                    MinLimit = 1;
                    for(int i = 0, ActiveEnginesCount = ActiveEngines.Count; i < ActiveEnginesCount; i++)
                    {
                        var e = ActiveEngines[i];
                        thrust += e.wThrustDir * e.nominalCurrentThrust(e.limit);
                        if(e.Role != TCARole.MANUAL &&
                           e.Role != TCARole.MANEUVER &&
                           MinLimit > e.limit) MinLimit = e.limit;
                        e.forceThrustPercentage(e.limit * 100);
                    }
                    var T = thrust.magnitude / Utils.G0;
                    if(use_wet_mass)
                    {
                        MinTWR = T / WetMass;
                        MaxTWR = T / DryMass;
                    }
                    else
                        MinTWR = MaxTWR = T / DryMass;
                }
            }
        }

        void AutoconfigureProfile()
        {
            var EnginesCount = Engines.Count;
            if(EnginesCount == 0) return;
            CalculateMassAndCoM(GetSelectedParts());
            //reset groups; set CoM-coaxial engines to UnBalanced role
            for(int i = 0; i < EnginesCount; i++)
            {
                var e = Engines[i];
                e.info.SetGroup(0);
                if(e.Role == TCARole.MANUAL)
                    continue;
                if(e.Role == TCARole.MANEUVER)
                {
                    if(e.torqueRatio < EngineOptimizer.C.UnBalancedThreshold)
                        e.info.SetMode(ManeuverMode.TRANSLATION);
                    continue;
                }
                if(e.engine.throttleLocked)
                {
                    e.info.SetRole(TCARole.MANUAL);
                    e.forceThrustPercentage(100);
                    continue;
                }
                e.UpdateThrustInfo();
                e.InitTorque(EditorLogic.fetch.ship[0].transform, CoM, Mass, MoI, EngineOptimizer.C.TorqueRatioFactor);
                if(e.torqueRatio < EngineOptimizer.C.UnBalancedThreshold) e.info.SetRole(TCARole.UNBALANCE);
            }
            //group symmetry-clones
            var group = 1;
            for(int i = 0; i < EnginesCount; i++)
            {
                var e = Engines[i];
                if(e.Group > 0) continue;
                if(e.part.symmetryCounterparts.Count > 0)
                {
                    e.info.SetGroup(group);
                    e.part.symmetryCounterparts.ForEach(p => p.Modules.GetModule<TCAEngineInfo>().group = group);
                    group += 1;
                }
            }
            //update active profile
            CFG.ActiveProfile.Update(Engines);
        }


        bool reset, init_engines, update_engines, update_stats, autoconfigure_profile;

        RealTimer updateDamper = new RealTimer(0.1);
        void Update()
        {
            if(EditorLogic.fetch == null || EditorLogic.fetch.ship == null) return;
            update_stats |= HaveSelectedPart;
            if(reset)
            {
                Available = false;
                Modules.Clear();
                Engines.Clear();
                RCS.Clear();
                PartsEditor.SetCFG(null);
                CFG = null;
                reset = false;
            }
            if(init_engines)
            {
                UpdateEngines();
                GetCFG();
                init_engines = false;
                update_stats = true;
            }
            if(update_engines)
            {
                UpdateEngines();
                if(CFG != null) UpdateCFG();
                else GetCFG();
                update_engines = false;
                update_stats = true;
            }
            if(autoconfigure_profile)
            {
                AutoconfigureProfile();
                autoconfigure_profile = false;
                update_stats = true;
            }
            if(update_stats && doShow && updateDamper.TimePassed)
            {
                UpdateShipStats();
                update_stats = false;
                updateDamper.Reset();
            }
            Available |= CFG != null;
            TCA_highlight.Update(Available && doShow);
            Engines_highlight.Update(Available && doShow && show_imbalance && ActiveEngines.Count > 0);
        }

        void DrawMainWindow(int windowID)
        {
            //help button
            if(GUI.Button(new Rect(WindowPos.width - 23f, 2f, 20f, 18f),
                          Loc.Content("HelpButton", "?", "EditorHelpTip", "Help"))) TCAManual.ToggleInstance();
            GUILayout.BeginVertical();
            {
                GUILayout.BeginHorizontal();
                {
                    if(GUILayout.Button(Loc.Content("SelectModules", "Select Modules", "SelectModulesTip", "Select which TCA Modules should be installed on this ship"),
                                            Styles.active_button, GUILayout.ExpandWidth(true)))
                        PartsEditor.Toggle();
                    if(Modules[typeof(MacroProcessor)])
                    {

                        if(TCAMacroEditor.Editing)
                            GUILayout.Label(Loc.T("EditMacros", "Edit Macros"), Styles.inactive_button, GUILayout.ExpandWidth(true));
                        else if(GUILayout.Button(Loc.T("EditMacros", "Edit Macros"), Styles.active_button, GUILayout.ExpandWidth(true)))
                            TCAMacroEditor.Edit(CFG);
                    }
                    Utils.ButtonSwitch(Loc.T("RangePlanner_Button", "Range Planner"),
                                       ref show_range_planner,
                                       Loc.T("RangePlanner_ButtonTip", "Estimate hover time, cruise range and ballistic hops for selected bodies."),
                                       GUILayout.ExpandWidth(true));
                    if(GUILayout.Button(Loc.Content("SaveAsDefault", "Save As Default", "SaveAsDefaultTip", "Save current configuration as default for new ships in this facility (VAB/SPH)"),
                                        Styles.active_button, GUILayout.ExpandWidth(true)))
                    {
                        var facility = EditorLogic.fetch.ship.shipFacility;
                        DialogFactory.Danger(
                            Loc.F("SaveAsDefaultConfirm", "Are you sure you want to save current ship configuration as default for <<1>>?", facility),
                            () => TCAScenario.UpdateDefaultConfig(facility, CFG),
                            context: this
                        );
                    }
                }
                GUILayout.EndHorizontal();
                GUILayout.BeginHorizontal();
                {
                    GUILayout.BeginVertical();
                    {
                        GUILayout.BeginHorizontal();
                        {
                            if(Utils.ButtonSwitch(Loc.T("EnableTCA", "Enable TCA"), ref CFG.Enabled, "", GUILayout.ExpandWidth(true)))
                            {
                                if(!CFG.Enabled)
                                    Engines.ForEach(e => e.forceThrustPercentage(100));
                                CFG.GUIVisible = CFG.Enabled;
                            }
                            if(Modules[typeof(AltitudeControl)])
                            {
                                if(Utils.ButtonSwitch(Loc.T("Hover", "Hover"), CFG.VF[VFlight.AltitudeControl],
                                                      Loc.T("HoverTip", "Enable Altitude Control"), GUILayout.ExpandWidth(false)))
                                    CFG.VF.Toggle(VFlight.AltitudeControl);
                                Utils.ButtonSwitch(Loc.T("FollowTerrain", "Follow Terrain"), ref CFG.AltitudeAboveTerrain,
                                                   Loc.T("FollowTerrainTip", "Enable follow terrain mode"), GUILayout.ExpandWidth(false));
                            }
                            if(Modules[typeof(VTOLControl)])
                            {
                                if(Utils.ButtonSwitch(Loc.T("VTOLMode", "VTOL Mode"), CFG.CTRL[ControlMode.VTOL],
                                                      Loc.T("VTOLModeTip", "Keyboard controls thrust direction instead of torque"), GUILayout.ExpandWidth(false)))
                                    CFG.CTRL.XToggle(ControlMode.VTOL);
                            }
                            if(Modules[typeof(VTOLAssist)])
                                Utils.ButtonSwitch(Loc.T("VTOLAssist", "VTOL Assist"), ref CFG.VTOLAssistON,
                                                   Loc.T("VTOLAssistTip", "Automatic assistance with vertical takeof or landing"), GUILayout.ExpandWidth(false));
                            if(Modules[typeof(FlightStabilizer)])
                                Utils.ButtonSwitch(Loc.T("FlightStabilizer", "Flight Stabilizer"), ref CFG.StabilizeFlight,
                                                   Loc.T("FlightStabilizerTip", "Automatic flight stabilization when vessel is out of control"), GUILayout.ExpandWidth(false));
                            if(Modules[typeof(CollisionPreventionSystem)])
                                Utils.ButtonSwitch(Loc.T("CPS", "CPS"), ref CFG.UseCPS,
                                                   Loc.T("CPSTip", "Enable Collistion Prevention System"), GUILayout.ExpandWidth(false));
                        }
                        GUILayout.EndHorizontal();
                        GUILayout.BeginHorizontal();
                        {
                            Utils.ButtonSwitch(Loc.T("AutoThrottle", "AutoThrottle"), ref CFG.BlockThrottle,
                                               Loc.T("AutoThrottleTip", "Change altitude/vertical velocity using main throttle control"), GUILayout.ExpandWidth(true));
                            if(Utils.ButtonSwitch(Loc.T("SmartEngines", "SmartEngines"), ref CFG.UseSmartEngines,
                                                  Loc.T("SmartEnginesTip", "Group engines by thrust direction and automatically use appropriate group for a meneuver"), GUILayout.ExpandWidth(true)))
                            { if(CFG.UseSmartEngines) CFG.SmartEngines.OnIfNot(SmartEnginesMode.Best); }
                            Utils.ButtonSwitch(Loc.T("AutoGear", "AutoGear"), ref CFG.AutoGear,
                                               Loc.T("AutoGearTip", "Automatically deploy/retract landing gear when needed"), GUILayout.ExpandWidth(true));
                            Utils.ButtonSwitch(Loc.T("AutoBrakes", "AutoBrakes"), ref CFG.AutoBrakes,
                                               Loc.T("AutoBrakesTip", "Automatically enable/disable brakes when needed"), GUILayout.ExpandWidth(true));
                            Utils.ButtonSwitch(Loc.T("AutoStage", "AutoStage"), ref CFG.AutoStage,
                                               Loc.T("AutoStageTip", "Automatically activate next stage when previous falmeouted"), GUILayout.ExpandWidth(true));
                            Utils.ButtonSwitch(Loc.T("AutoChute", "AutoChute"), ref CFG.AutoParachutes,
                                               Loc.T("AutoChuteTip", "Automatically activate parachutes when needed"), GUILayout.ExpandWidth(true));
                        }
                        GUILayout.EndHorizontal();
                        GUILayout.BeginHorizontal();
                        if(Modules[typeof(HorizontalSpeedControl)])
                        {
                            Utils.ButtonSwitch(Loc.T("HorThrust", "Hor. Thrust"),
                                ref CFG.UseHorizontalThrust,
                                Loc.T("HorThrustTip", "Use maneuver engines to provide thrust for horizontal flight"),
                                GUILayout.ExpandWidth(true));
                            if(MinHorizontalAccel.Draw(Loc.T("KNPerT", "kN/t"),
                                field_width: 50,
                                suffix_tooltip:
                                Loc.T("MinHorizontalAccelTip", "Maneuver engines will be used as horizontal thrusters only if they produce more thrust than this.")
                            )
                            )
                                CFG.MinHorizontalAccel = MinHorizontalAccel;
                            if(Modules[typeof(TranslationControl)])
                                Utils.ButtonSwitch(Loc.T("RCSTranslation", "RCS Translation"),
                                    ref CFG.CorrectWithTranslation,
                                    Loc.T("RCSTranslationTip", "Use RCS to correct horizontal velocity"),
                                    GUILayout.ExpandWidth(true));
                        }
                        Utils.ButtonSwitch(Loc.T("RCSRotation", "RCS Rotation"), ref CFG.RotateWithRCS,
                            Loc.T("RCSRotationTip", "Use RCS for attitude control"), GUILayout.ExpandWidth(true));
                        GUILayout.EndHorizontal();
                    }
                    GUILayout.EndVertical();
                }
                GUILayout.EndHorizontal();
                if(Engines.Count > 0)
                {
                    if(GUILayout.Button(Loc.Content("AutoconfigureActiveProfile", "Autoconfigure Active Profile",
                                                       "AutoconfigureActiveProfileTip", "This will overwrite any existing groups and roles"),
                                        Styles.danger_button, GUILayout.ExpandWidth(true)))
                        autoconfigure_profile = true;
                    CFG.EnginesProfiles.Draw(height);
                    if(CFG.ActiveProfile.Changed)
                    {
                        CFG.ActiveProfile.Apply(Engines);
                        update_engines = true;
                    }
                }
                GUILayout.BeginHorizontal(Styles.white);
                {
                    GUILayout.Label(Loc.T("ShipInfo", "Ship Info:"));
                    GUILayout.FlexibleSpace();
                    GUILayout.Label(Loc.T("Mass", "Mass:"), Styles.boxed_label);
                    if(Utils.ButtonSwitch(Utils.formatMass(WetMass), use_wet_mass, Loc.T("WetMassTip", "Balance engines using Wet Mass")))
                    {
                        use_wet_mass = true;
                        update_stats = true;
                    }
                    GUILayout.Label("►");
                    if(Utils.ButtonSwitch(Utils.formatMass(DryMass), !use_wet_mass, Loc.T("DryMassTip", "Balance engines using Dry Mass")))
                    {
                        use_wet_mass = false;
                        update_stats = true;
                    }
                    if(CFG.Enabled)
                    {
                        if(ActiveEngines.Count > 0)
                        {
                            GUILayout.Label(new GUIContent(Loc.F("TMR", "TMR: <<1>> ► <<2>>", MinTWR.ToString("F2"), MaxTWR.ToString("F2")),
                                                           Loc.T("TMRTip", "Thrust ot Mass Ratio")),
                                            Styles.fracStyle(Utils.Clamp(MinTWR - 1, 0, 1)));
                            GUILayout.Label(new GUIContent(Loc.F("Balanced", "Balanced: <<1>>", MinLimit.ToString("P1")),
                                                           Loc.T("BalancedTip", "The efficacy of the least efficient of balanced engines")),
                                            Styles.fracStyle(MinLimit));
                            Utils.ButtonSwitch(Loc.T("HL", "HL"), ref show_imbalance, Loc.T("HLTip", "Highlight engines with low efficacy deu to balancing"));
                        }
                        else GUILayout.Label(Loc.T("NoActiveEngines", "No active engines"), Styles.boxed_label);
                    }
                    else GUILayout.Label(Loc.T("TCADisabledShort", "TCA is disabled"), Styles.boxed_label);
                }
                GUILayout.EndHorizontal();
            }
            GUILayout.EndVertical();
            TooltipsAndDragWindow();
        }

        CelestialBody range_body()
        {
            if(FlightGlobals.Bodies == null || FlightGlobals.Bodies.Count == 0)
                return null;
            if(range_body_index < 0)
                range_body_index = FlightGlobals.GetHomeBodyIndex();
            range_body_index = Mathf.Clamp(range_body_index, 0, FlightGlobals.Bodies.Count - 1);
            return FlightGlobals.Bodies[range_body_index];
        }

        void draw_body_selector()
        {
            var body = range_body();
            GUILayout.BeginHorizontal();
            GUILayout.Label(Loc.T("RangePlanner_Body", "Body:"), GUILayout.Width(90));
            if(GUILayout.Button("<", Styles.active_button, GUILayout.Width(25)))
                range_body_index = (range_body_index + FlightGlobals.Bodies.Count - 1) % FlightGlobals.Bodies.Count;
            GUILayout.Label(body != null ? body.GetName() : "N/A", Styles.boxed_label, GUILayout.ExpandWidth(true));
            if(GUILayout.Button(">", Styles.active_button, GUILayout.Width(25)))
                range_body_index = (range_body_index + 1) % FlightGlobals.Bodies.Count;
            GUILayout.EndHorizontal();
        }

        void draw_scenario_selector()
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(Loc.T("RangePlanner_Scenario", "Scenario:"), GUILayout.Width(90));
            if(GUILayout.Button("<", Styles.active_button, GUILayout.Width(25)))
                range_scenario = (MissionScenario)(((int)range_scenario + 4) % 5);
            GUILayout.Label(range_scenario_label(range_scenario), Styles.boxed_label, GUILayout.ExpandWidth(true));
            if(GUILayout.Button(">", Styles.active_button, GUILayout.Width(25)))
                range_scenario = (MissionScenario)(((int)range_scenario + 1) % 5);
            GUILayout.EndHorizontal();
        }

        static string range_scenario_label(MissionScenario scenario)
        {
            switch(scenario)
            {
            case MissionScenario.Hover:
                return Loc.T("RangePlanner_ScenarioHover", "Hover");
            case MissionScenario.PoweredCruise:
                return Loc.T("RangePlanner_ScenarioCruise", "Powered Cruise");
            case MissionScenario.BallisticHop:
                return Loc.T("RangePlanner_ScenarioHop", "Ballistic Hop");
            case MissionScenario.RepeatedHops:
                return Loc.T("RangePlanner_ScenarioRepeatedHops", "Repeated Hops");
            default:
                return Loc.T("RangePlanner_ScenarioTarget", "Target Range");
            }
        }

        static string format_range_time(float seconds)
        {
            if(seconds <= 0 || float.IsNaN(seconds) || float.IsInfinity(seconds))
                return "N/A";
            return KSPUtil.PrintDateDeltaCompact(seconds, true, true);
        }

        static string format_range_recommendation(MissionRecommendation recommendation)
        {
            switch(recommendation)
            {
            case MissionRecommendation.GoTo:
                return Loc.T("RangePlanner_RecommendGoTo", "Go To");
            case MissionRecommendation.BallisticJump:
                return Loc.T("RangePlanner_RecommendJump", "Jump To");
            default:
                return Loc.T("RangePlanner_RecommendNone", "No safe mode");
            }
        }

        EnginesDB range_planning_engines(out bool used_default_engines)
        {
            used_default_engines = false;
            if(ActiveEngines.Count > 0)
                return ActiveEngines;
            var fallback = new EnginesDB();
            for(int i = 0, count = Engines.Count; i < count; i++)
            {
                var e = Engines[i];
                if(e?.engine == null)
                    continue;
                e.throttle = e.VSF = e.thrustMod = 1;
                e.UpdateThrustInfo();
                e.InitLimits();
                if(!e.isThruster)
                    continue;
                e.limit = e.best_limit = 1f;
                fallback.Add(e);
            }
            used_default_engines = fallback.Count > 0;
            return fallback;
        }

        MissionPredictionResult editor_range_prediction()
        {
            if(EditorLogic.fetch == null || EditorLogic.fetch.ship == null)
                return null;
            var body = range_body();
            var area = Mathf.Max(Mathf.Pow(Mathf.Max(Mass, 1), 2f / 3f), 1);
            var range_engines = range_planning_engines(out var used_default_engines);
            var snapshot = MissionPerformanceSnapshot.FromEditor(EditorLogic.fetch.ship,
                                                                  range_engines,
                                                                  WetMass,
                                                                  DryMass,
                                                                  area);
            if(snapshot.EditorSnapshot)
                snapshot.Warnings.Add(Loc.T("RangePlanner_EditorAreaWarning", "Editor aerodynamic area is estimated from vessel mass until flight data is available."));
            if(used_default_engines)
                snapshot.Warnings.Add(Loc.T("RangePlanner_EditorAllEnginesWarning", "The range estimate uses all detected engines at full thrust."));
            var profile = new MissionProfile
            {
                Body = body,
                Scenario = range_scenario,
                StartAltitude = RangeStartAltitude,
                TargetDistance = RangeTargetDistance,
                MaxCruiseSpeed = RangeMaxCruiseSpeed,
                HoverReserveTime = RangeHoverReserve,
                AllowParachutes = range_allow_parachutes,
                AllowStaging = range_allow_staging,
                AllowAtmosphericAssist = range_allow_atmo_assist
            };
            return MissionRangePlanner.Evaluate(snapshot, profile);
        }

        void draw_range_results(MissionPredictionResult result)
        {
            if(result == null)
                return;
            if(!result.Valid)
            {
                GUILayout.Label(result.Status, Styles.warning, GUILayout.ExpandWidth(true));
                return;
            }
            GUILayout.Label(Loc.F("RangePlanner_ResultHover", "Hover time: <<1>>", format_range_time(result.HoverTime)),
                            Styles.boxed_label, GUILayout.ExpandWidth(true));
            GUILayout.Label(Loc.F("RangePlanner_ResultCruise", "Powered cruise range: <<1>>",
                                  Utils.formatBigValue(result.PoweredCruiseRange, "m")),
                            Styles.boxed_label, GUILayout.ExpandWidth(true));
            GUILayout.Label(Loc.F("RangePlanner_ResultHop", "Best hop: <<1>> using <<2>> fuel",
                                  Utils.formatBigValue(result.BestHopDistance, "m"),
                                  Utils.formatMass(result.BestHopFuel)),
                            Styles.boxed_label, GUILayout.ExpandWidth(true));
            GUILayout.Label(Loc.F("RangePlanner_ResultRepeatedHops", "Repeated hops: <<1>> for <<2>> total",
                                  result.HopCount,
                                  Utils.formatBigValue(result.TotalHopRange, "m")),
                            Styles.boxed_label, GUILayout.ExpandWidth(true));
            if(RangeTargetDistance > 0)
                GUILayout.Label(Loc.F("RangePlanner_ResultTarget", "Target recommendation: <<1>> | Go To <<2>> | Jump <<3>>",
                                      format_range_recommendation(result.Recommendation),
                                      Utils.formatMass(result.TargetGoToFuel),
                                      Utils.formatMass(result.TargetJumpFuel)),
                              Styles.boxed_label, GUILayout.ExpandWidth(true));
            GUILayout.Label(Loc.F("RangePlanner_ResultReserve", "Reserve fuel: <<1>> | Confidence: <<2>>",
                                  Utils.formatMass(result.ReserveFuel),
                                  result.Confidence.ToString("P0")),
                          Styles.boxed_label, GUILayout.ExpandWidth(true));
            if(result.MaxDynamicPressure > 0)
                GUILayout.Label(Loc.F("RangePlanner_ResultDynPressure", "Cruise dynamic pressure: <<1>> kPa",
                                      result.MaxDynamicPressure.ToString("F2")),
                              Styles.boxed_label, GUILayout.ExpandWidth(true));
            foreach(var warning in result.Warnings)
                GUILayout.Label(warning, Styles.warning, GUILayout.ExpandWidth(true));
        }

        void DrawRangePlannerWindow(int windowID)
        {
            GUILayout.BeginVertical();
            GUILayout.BeginHorizontal();
            GUILayout.Label(Loc.T("RangePlanner_Title", "Range Planner"), Styles.boxed_label, GUILayout.ExpandWidth(true));
            if(GUILayout.Button("X", Styles.close_button, GUILayout.Width(25)))
                show_range_planner = false;
            GUILayout.EndHorizontal();
            draw_body_selector();
            draw_scenario_selector();
            GUILayout.BeginHorizontal();
            GUILayout.Label(Loc.T("RangePlanner_StartAltitude", "Start altitude:"), GUILayout.Width(130));
            RangeStartAltitude.Draw("m", 100, "F0", suffix_width: 30);
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            GUILayout.Label(Loc.T("RangePlanner_TargetDistance", "Target distance:"), GUILayout.Width(130));
            RangeTargetDistance.Draw("m", 1000, "F0", suffix_width: 30);
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            GUILayout.Label(Loc.T("RangePlanner_MaxCruiseSpeed", "Max cruise speed:"), GUILayout.Width(130));
            RangeMaxCruiseSpeed.Draw("m/s", 10, "F0", suffix_width: 35);
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            GUILayout.Label(Loc.T("RangePlanner_HoverReserve", "Hover reserve:"), GUILayout.Width(130));
            RangeHoverReserve.Draw("s", 10, "F0", suffix_width: 30);
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            Utils.ButtonSwitch(Loc.T("RangePlanner_AllowChutes", "Chutes"), ref range_allow_parachutes,
                               Loc.T("RangePlanner_AllowChutesTip", "Allow parachutes as a landing reserve assumption."),
                               GUILayout.ExpandWidth(true));
            Utils.ButtonSwitch(Loc.T("RangePlanner_AllowStaging", "Staging"), ref range_allow_staging,
                               Loc.T("RangePlanner_AllowStagingTip", "Include conservative staged fuel segments in warnings and delta-v context."),
                               GUILayout.ExpandWidth(true));
            Utils.ButtonSwitch(Loc.T("RangePlanner_AllowAtmoAssist", "Atmo Assist"), ref range_allow_atmo_assist,
                               Loc.T("RangePlanner_AllowAtmoAssistTip", "Allow atmospheric drag/parachute assumptions where available."),
                               GUILayout.ExpandWidth(true));
            GUILayout.EndHorizontal();
            range_scroll = GUILayout.BeginScrollView(range_scroll, GUILayout.Height(150));
            draw_range_results(editor_range_prediction());
            GUILayout.EndScrollView();
            GUIWindowBase.TooltipsAndDragWindow();
            GUILayout.EndVertical();
        }

        protected override bool can_draw() { return Available; }

        static void highlight_engine(ThrusterWrapper e)
        {
            if(e.limit < 1)
            {
                var lim = e.limit * e.limit;
                e.part.HighlightAlways(Colors.FractionGradient.Evaluate(lim));
            }
        }

        void highlight_engines()
        {
            for(int i = 0, EnginesCount = Engines.Count; i < EnginesCount; i++)
            {
                var e = Engines[i];
                if(e.Role != TCARole.BALANCE && e.Role != TCARole.MAIN)
                    e.part.SetHighlightDefault();
            }
            ActiveEngines.Balanced.ForEach(highlight_engine);
            ActiveEngines.Main.ForEach(highlight_engine);
        }

        static void reset_engines_highlightig()
        {
            EditorLogic.fetch.ship.Parts
                .Where(p => p.Modules.Contains<ModuleEngines>())
                .ForEach(p => p.SetHighlightDefault());
        }

        void highlight_TCA()
        {
            if(TCA != null && TCA.part != null)
                TCA.part.HighlightAlways(Colors.Enabled);
        }

        void reset_TCA_highlighting()
        {
            if(TCA != null && TCA.part != null)
                TCA.part.SetHighlightDefault();
        }

        protected override void draw_gui()
        {
            LockControls();
            WindowPos =
                GUILayout.Window(GetInstanceID(),
                                 WindowPos,
                                 DrawMainWindow,
                                 Title,
                                 GUILayout.Width(width),
                                 GUILayout.Height(height)).clampToScreen();
            if(show_imbalance && ActiveEngines.Count > 0)
            {
                Markers.DrawWorldMarker(WetCoM, Colors.Active, Loc.T("CenterOfMass", "Center of Mass"), CoM_Icon);
                Markers.DrawWorldMarker(DryCoM, Colors.Danger, Loc.T("CenterOfDryMass", "Center of Dry Mass"), CoM_Icon);
            }
            if(show_range_planner)
                range_planner_pos =
                    GUILayout.Window(GetInstanceID() + 1001,
                                     range_planner_pos,
                                     DrawRangePlannerWindow,
                                     Loc.T("RangePlanner_Title", "Range Planner"),
                                     GUILayout.Width(460),
                                     GUILayout.Height(360)).clampToScreen();
        }

        class HighlightSwitcher
        {
            bool enabled;
            public Action Enable = delegate { };
            public Action Disable = delegate { };

            public HighlightSwitcher(Action highlight, Action disable_highliting)
            {
                Enable = highlight;
                Disable = disable_highliting;
            }

            public void Update(bool predicate)
            {
                if(predicate)
                {
                    Enable();
                    enabled = true;
                }
                else if(enabled)
                {
                    Disable();
                    enabled = false;
                }
            }

            public void Reset() { Update(false); }
        }
    }
}
