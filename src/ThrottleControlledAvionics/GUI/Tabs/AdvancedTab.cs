//   AdvancedTab.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2017 Allis Tauri

using System;
using System.Linq;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    public class AdvancedTab : ControlTab
    {
        public AdvancedTab(ModuleTCA tca) : base(tca) { }

        public override bool Valid { get { return true; } }

        ThrottleControl THR;
        VTOLControl VTOL;
        VTOLAssist VLA;
        FlightStabilizer STB;
        TranslationControl TRA;
        private HorizontalSpeedControl HSC;
        CollisionPreventionSystem CPS;

        public bool SelectingKey;

        NamedConfig selected_config;
        string config_name = string.Empty;
        string selected_config_name = string.Empty;
        readonly DropDownList named_configs = new DropDownList();
        const int NamedConfigChooserThreshold = 5;

        private readonly FloatField MinHorizontalAccel = new FloatField(min:0);

        public override void Init()
        {
            base.Init();
            MinHorizontalAccel.Value = CFG.MinHorizontalAccel;
        }

        #region Configs Selector
        void SyncSelectedConfig(string name)
        {
            if(string.IsNullOrEmpty(name) || !TCAScenario.NamedConfigs.ContainsKey(name))
            {
                selected_config = null;
                selected_config_name = string.Empty;
                return;
            }
            selected_config_name = name;
            selected_config = TCAScenario.GetConfig(name);
            config_name = name;
        }

        void EnsureSelectedConfigName()
        {
            if(!string.IsNullOrEmpty(selected_config_name) && TCAScenario.NamedConfigs.ContainsKey(selected_config_name))
                return;
            if(TCAScenario.NamedConfigs.Count == 0)
            {
                selected_config = null;
                selected_config_name = string.Empty;
                return;
            }
            SyncSelectedConfig(TCAScenario.NamedConfigs.Keys[0]);
        }

        public void UpdateNamedConfigs()
        {
            EnsureSelectedConfigName();
            if(TCAScenario.NamedConfigs.Count <= NamedConfigChooserThreshold)
                return;
            var configs = TCAScenario.NamedConfigs.Keys.ToList();
            configs.Add(string.Empty);
            named_configs.Items = configs;
            if(!string.IsNullOrEmpty(selected_config_name) && TCAScenario.NamedConfigs.ContainsKey(selected_config_name))
                named_configs.SelectItem(TCAScenario.NamedConfigs.IndexOfKey(selected_config_name));
            else
                named_configs.SelectItem(configs.Count - 1);
        }

        void SelectConfig()
        {
            if(TCAScenario.NamedConfigs.Count == 0)
            {
                GUILayout.Label("", Styles.white, GUILayout.ExpandWidth(true));
                selected_config = null;
                return;
            }
            if(TCAScenario.NamedConfigs.Count <= NamedConfigChooserThreshold)
            {
                EnsureSelectedConfigName();
                var name = Utils.LeftRightChooser(
                    selected_config_name,
                    TCAScenario.NamedConfigs,
                    Loc.T("NamedConfig_SelectTip", "Select a saved configuration to load, overwrite, or delete."));
                if(name != selected_config_name)
                    SyncSelectedConfig(name);
            }
            else
            {
                named_configs.DrawButton();
                if(named_configs.SelectedIndex < TCAScenario.NamedConfigs.Count)
                    SyncSelectedConfig(TCAScenario.NamedConfigs.Keys[named_configs.SelectedIndex]);
                else
                {
                    selected_config = null;
                    selected_config_name = string.Empty;
                }
            }
        }
        #endregion

        public void Toggles()
        {
            GUILayout.BeginHorizontal();
            if(VTOL != null)
            {
                if(Utils.ButtonSwitch(Loc.T("VTOLMode", "VTOL Mode"), CFG.CTRL[ControlMode.VTOL],
                                      Loc.T("VTOLModeTip", "Keyboard controls thrust direction instead of torque"), GUILayout.ExpandWidth(true)))
                    CFG.CTRL.XToggle(ControlMode.VTOL);
            }
            if(VLA != null)
                Utils.ButtonSwitch(Loc.T("VTOLAssist", "VTOL Assist"), ref CFG.VTOLAssistON,
                                   Loc.T("VTOLAssistTipAdv", "Assist with vertical takeoff and landing"), GUILayout.ExpandWidth(true));
            if(STB != null)
                Utils.ButtonSwitch(Loc.T("Stabilizer", "Stabilizer"), ref CFG.StabilizeFlight,
                                   Loc.T("StabilizerTip", "Try to stabilize flight if spinning uncontrollably"), GUILayout.ExpandWidth(true));
            if(CPS != null)
                Utils.ButtonSwitch(Loc.T("CPS", "CPS"), ref CFG.UseCPS,
                                   Loc.T("CPSCollisionTip", "Enable Collision Prevention System"), GUILayout.ExpandWidth(true));
            Utils.EnsureLayoutControl();
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            Utils.ButtonSwitch(Loc.T("AutoGear", "AutoGear"), ref CFG.AutoGear,
                               Loc.T("AutoGearTip", "Automatically deploy/retract landing gear when needed"), GUILayout.ExpandWidth(true));
            Utils.ButtonSwitch(Loc.T("AutoBrakes", "AutoBrakes"), ref CFG.AutoBrakes,
                               Loc.T("AutoBrakesTip", "Automatically enable/disable brakes when needed"), GUILayout.ExpandWidth(true));
            Utils.ButtonSwitch(Loc.T("AutoStage", "AutoStage"), ref CFG.AutoStage,
                               Loc.T("AutoStageTip", "Automatically activate next stage when previous falmeouted"), GUILayout.ExpandWidth(true));
            Utils.ButtonSwitch(Loc.T("AutoChute", "AutoChute"), ref CFG.AutoParachutes,
                               Loc.T("AutoChuteTip", "Automatically activate parachutes when needed"), GUILayout.ExpandWidth(true));
            if(GUILayout.Button(Loc.Content("Modules", "Modules", "ModulesTip", "Show TCA modules installed on this ship"),
                                Styles.active_button, GUILayout.ExpandWidth(true)))
                UI.ModulesGraph.Toggle();
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            if(HSC != null)
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
                if(TRA != null)
                    Utils.ButtonSwitch(Loc.T("RCSTranslation", "RCS Translation"),
                        ref CFG.CorrectWithTranslation,
                        Loc.T("RCSTranslationTip", "Use RCS to correct horizontal velocity"),
                        GUILayout.ExpandWidth(true));
            }
            Utils.ButtonSwitch(Loc.T("RCSRotation", "RCS Rotation"), ref CFG.RotateWithRCS,
                Loc.T("RCSRotationTip", "Use RCS for attitude control"), GUILayout.ExpandWidth(true));
            GUILayout.EndHorizontal();
        }

        void ConfigsGUI()
        {
            GUILayout.BeginVertical();
            GUILayout.Label(Loc.T("ManageNamedConfigs", "Manage named configurations"), Styles.label, GUILayout.ExpandWidth(true));
            GUILayout.BeginHorizontal();
            GUILayout.Label(Loc.T("Name", "Name:"), GUILayout.ExpandWidth(false));
            config_name = GUILayout.TextField(config_name, GUILayout.ExpandWidth(true), GUILayout.MinWidth(50));
            if(TCAScenario.NamedConfigs.ContainsKey(config_name))
            {
                if(GUILayout.Button(Loc.Content("Overwrite", "Overwrite", "OverwriteConfigTip", "Overwrite selected configuration with the current one"),
                                    Styles.danger_button, GUILayout.ExpandWidth(false)))
                    TCAScenario.SaveNamedConfig(config_name, CFG, true);
            }
            else if(GUILayout.Button(Loc.Content("Add", "Add", "AddConfigTip", "Save current configuration"),
                                     Styles.open_button, GUILayout.ExpandWidth(false))
                    && config_name != string.Empty)
            {
                TCAScenario.SaveNamedConfig(config_name, CFG);
                UpdateNamedConfigs();
                SyncSelectedConfig(config_name);
            }
            SelectConfig();
            if(GUILayout.Button(Loc.Content("Load", "Load", "LoadConfigTip", "Load selected configuration"),
                                Styles.active_button, GUILayout.ExpandWidth(false))
               && selected_config != null)
                CFG.Copy(selected_config);
            if(GUILayout.Button(Loc.Content("Delete", "Delete", "DeleteConfigTip", "Delete selected configuration"),
                                Styles.danger_button, GUILayout.ExpandWidth(false))
               && selected_config != null)
            {
                var removedIndex = TCAScenario.NamedConfigs.IndexOfKey(selected_config.Name);
                TCAScenario.NamedConfigs.Remove(selected_config.Name);
                if(TCAScenario.NamedConfigs.Count > 0)
                {
                    var keys = TCAScenario.NamedConfigs.Keys;
                    SyncSelectedConfig(keys[Utils.Clamp(removedIndex, 0, keys.Count - 1)]);
                }
                else
                {
                    selected_config = null;
                    selected_config_name = string.Empty;
                }
                UpdateNamedConfigs();
            }
            GUILayout.EndHorizontal();
            GUILayout.EndVertical();
        }

        void ControllerProperties()
        {
            Utils.ButtonSwitch(Loc.T("AutotuneControllers", "Autotune engines' controller parameters"), ref CFG.AutoTune, "", GUILayout.ExpandWidth(true));
            if(CFG.AutoTune) return;
            //steering modifiers
            GUILayout.BeginHorizontal();
            CFG.SteeringGain = Utils.FloatSlider(Loc.T("SteeringGain", "Steering Gain"), CFG.SteeringGain, 0, 1, "P1");
            CFG.PitchYawLinked = GUILayout.Toggle(CFG.PitchYawLinked, Loc.T("LinkPitchYaw", "Link Pitch&Yaw"), GUILayout.ExpandWidth(false));
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            if(CFG.PitchYawLinked && !CFG.AutoTune)
            {
                CFG.SteeringModifier.x = Utils.FloatSlider(Loc.T("PitchYaw", "Pitch&Yaw"), CFG.SteeringModifier.x, 0, 1, "P1");
                CFG.SteeringModifier.z = CFG.SteeringModifier.x;
            }
            else
            {
                CFG.SteeringModifier.x = Utils.FloatSlider(Loc.T("Pitch", "Pitch"), CFG.SteeringModifier.x, 0, 1, "P1");
                CFG.SteeringModifier.z = Utils.FloatSlider(Loc.T("Yaw", "Yaw"), CFG.SteeringModifier.z, 0, 1, "P1");
            }
            CFG.SteeringModifier.y = Utils.FloatSlider(Loc.T("Roll", "Roll"), CFG.SteeringModifier.y, 0, 1, "P1");
            GUILayout.EndHorizontal();
            //engines
            CFG.Engines.DrawControls(Loc.T("EnginesController", "Engines Controller"), EngineOptimizer.C.MaxP, EngineOptimizer.C.MaxI);
        }

        public override void Draw()
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(UI.Title, Styles.label, GUILayout.ExpandWidth(true));
            if(GUILayout.Button(Loc.Content("Reload", "Reload", "ReloadTip", "Reload TCA settings from file"),
                                Styles.active_button, GUILayout.ExpandWidth(true)))
            {
                Globals.Load();
                TCA.OnReloadGlobals();
            }
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            //change key binding
            if(GUILayout.Button(SelectingKey ? Loc.Content("HotKeyChoose", "HotKey: ?", "HotKeyChooseTip", "Choose new TCA hotkey") :
                                Loc.Content("HotKey", Loc.F("HotKeyFormat", "HotKey: <<1>>", UI.TCA_Key), "HotKeyTip", "Select TCA Hotkey"),
                                SelectingKey ? Styles.enabled_button : Styles.active_button,
                                GUILayout.ExpandWidth(true)))
            {
                SelectingKey = true;
                Utils.Message(Loc.T("HotKeyPrompt", "Press a key that will toggle TCA.\n" +
                              "Press BACKSPACE to remove TCA hotkey.\n" +
                              "Press ESCAPE to cancel."));
            }
            if(Utils.ButtonSwitch(Loc.T("AutoSave", "AutoSave"), ref Globals.Instance.AutosaveBeforeLanding,
                                  Loc.T("AutoSaveTip", "Automatically save the game before executing complex autopilot programs"),
                                  GUILayout.ExpandWidth(true)))
                Globals.Save(":AutosaveBeforeLanding");
            if(Utils.ButtonSwitch(Globals.Instance.UseStockAppLauncher ? Loc.T("Launcher", "Launcher") : Loc.T("Toolbar", "Toolbar"),
                                  ref Globals.Instance.UseStockAppLauncher,
                                  Loc.T("LauncherToolbarTip", "Use stock AppLauncher or Toolbar plugin?"),
                                  GUILayout.ExpandWidth(true)))
            {
                TCAAppToolbar.Init();
                Globals.Save(":UseStockAppLauncher");
            }
            Utils.ButtonSwitch(Loc.T("AutoShow", "AutoShow"), ref UI.ShowOnHover,
                               Loc.T("AutoShowTip", "Show collapsed TCA window when mouse hovers over it"), 
                               GUILayout.ExpandWidth(true));
            Utils.ButtonSwitch(Loc.T("ShowCoM", "Show CoM"), ref UI.ShowCoM,
                Loc.T("ShowCoMTip", "Show CoM of the current vessel"), 
                GUILayout.ExpandWidth(true));
            if(GUILayout.Button(Loc.Content("InfoPanel", "InfoPanel", 
                "InfoPanelTip", "Show test info message to change the position of the info panel"),
                GUILayout.ExpandWidth(true)) 
               && string.IsNullOrEmpty(TCAGui.StatusMessage))
                Status(InfoPanel.TEST_MSG);
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            GLB.HudUiScale = Utils.FloatSlider(Loc.T("HudUiScale", "HUD Scale"),
                GLB.HudUiScale, 0.5f, 1.5f, "P0", 130,
                Loc.T("HudUiScaleTip",
                    "Scale in-flight HUD panels relative to the main TCA window"));
            GUILayout.EndHorizontal();
            Toggles();
            if(THR != null)
            {
                GUILayout.BeginHorizontal();
                CFG.ControlSensitivity = Utils.FloatSlider(Loc.T("KeyboardSensitivity", "Keyboard sensitivity"), CFG.ControlSensitivity, 0.001f, 0.05f, "P2");
                GUILayout.EndHorizontal();
            }
            ControllerProperties();
            ConfigsGUI();
        }

        public override void Update()
        {
            var e = Event.current;
            if(e.isKey)
            {
                if(e.keyCode == KeyCode.Escape)
                {
                    Message(Loc.T("HotKeyCanceled", "TCA: hotkey selection canceled."));
                }
                else if(e.keyCode == KeyCode.Backspace || e.character == '\b')
                {
                    UI.TCA_Key = KeyCode.None;
                    Message(Loc.T("HotKeyRemoved", "TCA: hotkey removed!"));
                }
                else
                {
                    //try to get the keycode if the Unity provided us only with the character
                    if(e.keyCode == KeyCode.None && char.IsLetterOrDigit(e.character))
                    {
                        var ec = new string(e.character, 1).ToUpper();
                        if(char.IsDigit(e.character)) ec = "Alpha" + ec;
                        try { e.keyCode = (KeyCode)Enum.Parse(typeof(KeyCode), ec); }
                        catch(Exception ex)
                        { Utils.Log("TCA GUI: exception caught while trying to set hotkey:\n{}", ex); }
                    }
                    if(e.keyCode == KeyCode.None)
                        Utils.Message(Loc.F("HotKeyConvertFailed", "Unable to convert '<<1>>' to keycode.\n" +
                                      "Please, try an alphabet or numeric character.",
                                      e.character));
                    else
                        UI.TCA_Key = e.keyCode;
                    Utils.Log("TCA: new key slected: {}", UI.TCA_Key);
                }
                SelectingKey = false;
            }
            if(!CFG.MinHorizontalAccel.Equals(MinHorizontalAccel))
                MinHorizontalAccel.Value = CFG.MinHorizontalAccel;
        }
    }
}

